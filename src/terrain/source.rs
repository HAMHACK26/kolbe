//! In-process terrain elevation fetcher. Pure Rust — no GDAL or other C
//! system libraries.
//!
//! Pipeline:
//!   1. OAuth2 client-credentials -> access token
//!   2. STAC /search with a bbox around the target point
//!   3. Download GeoTIFF tiles, decode pixels + geotransform + EPSG in memory
//!   4. Reproject each output pixel into the tile's native CRS (`proj4rs`),
//!      bilinear-sample, merge tiles
//!   5. Subtract the minimum elevation (normalise to 0)
//!
//! Credentials/config are loaded from a `.env` file next to `Cargo.toml`
//! (see `.env.example`). This module replaces the former Python height_server.
//!
//! The raw-TIFF reader is adapted from a pure-Rust reference implementation and
//! is deliberately narrow: it targets the Lantmateriet `dtm-cog` product
//! (classic TIFF, float32/int16, deflate or uncompressed, horizontal predictor,
//! SWEREF99 / UTM). It is not a general GeoTIFF library.

use std::io::{Cursor, Read, Seek, SeekFrom};
use std::sync::{Arc, Mutex};

use bevy::log::{info, warn};

/// Where one phase sits on the single overall progress bar, as fractions of
/// the whole load. The phases below are contiguous and cover elevation *and*
/// vegetation, so the bar fills once end-to-end instead of each phase
/// restarting it from zero.
#[derive(Clone, Copy, Default)]
pub struct PhaseWeight {
    pub start: f32,
    pub width: f32,
}

const fn weight(start: f32, width: f32) -> PhaseWeight {
    PhaseWeight { start, width }
}

const PHASE_AUTH: PhaseWeight = weight(0.00, 0.03);
const PHASE_SEARCH: PhaseWeight = weight(0.03, 0.05);
const PHASE_DOWNLOAD: PhaseWeight = weight(0.08, 0.42);
const PHASE_REPROJECT: PhaseWeight = weight(0.50, 0.48);
const PHASE_TERRAIN_READY: PhaseWeight = weight(1.0, 0.0);

/// Live progress shared between the fetch task and the loading UI.
#[derive(Clone, Default)]
pub struct Progress {
    pub phase: String,
    pub done: usize,
    pub total: usize,
    pub current: String,
    pub weight: PhaseWeight,
}

impl Progress {
    /// Position on the single overall bar, in 0..=1. Phases with no known
    /// total sit at their own start offset rather than reading as empty.
    pub fn overall_fraction(&self) -> f32 {
        let within = if self.total > 0 {
            (self.done as f32 / self.total as f32).clamp(0.0, 1.0)
        } else {
            0.0
        };
        (self.weight.start + self.weight.width * within).clamp(0.0, 1.0)
    }
}

pub type ProgressHandle = Arc<Mutex<Progress>>;

fn set_phase(
    progress: &ProgressHandle,
    phase: &str,
    done: usize,
    total: usize,
    weight: PhaseWeight,
) {
    if let Ok(mut p) = progress.lock() {
        p.phase = phase.to_string();
        p.done = done;
        p.total = total;
        p.current = String::new();
        p.weight = weight;
    }
}

/// Final elevation grid. `heights_m` is row-major with row 0 at the north edge,
/// normalised so the lowest point is 0, in metres. Non-covered cells are 0.
pub struct TerrainGrid {
    pub heights_m: Vec<f32>,
    pub size: usize,
}

// ─── Config ─────────────────────────────────────────────────────────────────

struct FetchConfig {
    token_url: String,
    secret: String,
    search_url: String,
    collection: String,
    output_size: usize,
    timeout_secs: u64,
    download_workers: usize,
}

impl FetchConfig {
    fn from_env() -> Result<Self, String> {
        let var = |key: &str| std::env::var(key).map_err(|_| format!("{key} not set — check .env"));
        Ok(Self {
            token_url: var("STAC_TOKEN_URL")?,
            secret: var("SECRET")?,
            search_url: var("STAC_SEARCH_URL")?,
            collection: std::env::var("STAC_COLLECTION").unwrap_or_else(|_| "dtm-cog".to_string()),
            output_size: std::env::var("OUTPUT_SIZE")
                .ok()
                .and_then(|v| v.parse().ok())
                .filter(|&size: &usize| (2..=1025).contains(&size))
                .unwrap_or(129),
            timeout_secs: std::env::var("DEFAULT_TIMEOUT")
                .ok()
                .and_then(|v| v.parse().ok())
                .filter(|&seconds: &u64| (10..=300).contains(&seconds))
                .unwrap_or(120),
            download_workers: std::env::var("DOWNLOAD_WORKERS")
                .ok()
                .and_then(|v| v.parse().ok())
                .filter(|&w| (1..=16).contains(&w))
                .unwrap_or(8),
        })
    }
}

// ─── Auth ───────────────────────────────────────────────────────────────────

fn get_token(client: &reqwest::blocking::Client, config: &FetchConfig) -> Result<String, String> {
    let response = client
        .post(&config.token_url)
        .header("Authorization", format!("Basic {}", config.secret))
        .form(&[("grant_type", "client_credentials")])
        .send()
        .and_then(|r| r.error_for_status())
        .map_err(|e| format!("token request failed: {e}"))?;
    let json: serde_json::Value = response.json().map_err(|e| e.to_string())?;
    json["access_token"]
        .as_str()
        .map(String::from)
        .ok_or_else(|| format!("no access_token in response: {json}"))
}

// ─── STAC search ────────────────────────────────────────────────────────────

pub(super) fn bbox_from_center(lat: f64, lon: f64, radius_km: f64) -> Result<(f64, f64, f64, f64), String> {
    if !(54.0..=70.0).contains(&lat) || !(10.0..=25.0).contains(&lon) {
        return Err("coordinates must be inside Sweden".to_string());
    }
    let dlat = radius_km / 111.32;
    let dlon = radius_km / (111.32 * lat.to_radians().cos());
    Ok((lon - dlon, lat - dlat, lon + dlon, lat + dlat))
}

fn stac_search(
    client: &reqwest::blocking::Client,
    config: &FetchConfig,
    token: &str,
    bbox: (f64, f64, f64, f64),
) -> Result<Vec<String>, String> {
    let bbox_str = format!("{},{},{},{}", bbox.0, bbox.1, bbox.2, bbox.3);
    let mut urls: Vec<String> = Vec::new();
    let mut next_url: Option<String> = None;
    let mut first = true;

    loop {
        let mut request = client
            .get(next_url.as_deref().unwrap_or(&config.search_url))
            .header("Authorization", format!("Bearer {token}"));
        if first {
            request = request.query(&[
                ("bbox", bbox_str.as_str()),
                ("collections", config.collection.as_str()),
                ("limit", "100"),
            ]);
            first = false;
        }
        let data: serde_json::Value = request
            .send()
            .and_then(|r| r.error_for_status())
            .map_err(|e| format!("STAC search failed: {e}"))?
            .json()
            .map_err(|e| e.to_string())?;

        for feature in data["features"].as_array().into_iter().flatten() {
            let Some(assets) = feature["assets"].as_object() else {
                continue;
            };
            for asset in assets.values() {
                let Some(href) = asset["href"].as_str().filter(|h| !h.is_empty()) else {
                    continue;
                };
                let kind = asset["type"].as_str().unwrap_or("").to_lowercase();
                let lower = href.to_lowercase();
                if kind.contains("tiff") || lower.ends_with(".tif") || lower.ends_with(".tiff") {
                    urls.push(href.to_string());
                }
            }
        }

        next_url = data["links"]
            .as_array()
            .and_then(|links| links.iter().find(|l| l["rel"] == "next"))
            .and_then(|link| link["href"].as_str())
            .map(String::from);
        if next_url.is_none() {
            break;
        }
    }

    urls.sort();
    urls.dedup();
    Ok(urls)
}

// ─── Tile download ──────────────────────────────────────────────────────────

fn file_name(url: &str) -> String {
    url.rsplit('/')
        .next()
        .unwrap_or(url)
        .split('?')
        .next()
        .unwrap_or(url)
        .to_string()
}

/// Fetch a byte range `[start, end]` (inclusive) with an HTTP Range request.
/// Requires the server to honour ranges (206); a 200 would mean the whole file,
/// which is exactly what we avoid for these 286 MB tiles, so treat it as error.
fn http_range(
    client: &reqwest::blocking::Client,
    token: &str,
    url: &str,
    start: u64,
    end: u64,
) -> Result<Vec<u8>, String> {
    let response = client
        .get(url)
        .header("Authorization", format!("Bearer {token}"))
        .header("Range", format!("bytes={start}-{end}"))
        .send()
        .and_then(|r| r.error_for_status())
        .map_err(|e| format!("range request failed: {e}"))?;
    if response.status() != reqwest::StatusCode::PARTIAL_CONTENT {
        return Err("server ignored Range request".to_string());
    }
    let bytes = response.bytes().map_err(|e| e.to_string())?.to_vec();
    let requested = end
        .checked_sub(start)
        .and_then(|span| span.checked_add(1))
        .ok_or_else(|| "invalid byte range".to_string())?;
    if bytes.len() as u64 != requested {
        return Err(format!("short range response: expected {requested} bytes, got {}", bytes.len()));
    }
    Ok(bytes)
}

// ─── Raw TIFF reader ────────────────────────────────────────────────────────
//
// Decodes geospatial metadata AND pixels straight from bytes. Avoids the `tiff`
// crate, which drops unknown tags and lacks float32 predictor support.
// Supports: classic TIFF (magic=42), deflate/none, predictor=2 for 16/32-bit.

fn rd_u16(buf: &[u8], off: usize, le: bool) -> u16 {
    let b: [u8; 2] = buf[off..off + 2].try_into().unwrap();
    if le { u16::from_le_bytes(b) } else { u16::from_be_bytes(b) }
}
fn rd_u32(buf: &[u8], off: usize, le: bool) -> u32 {
    let b: [u8; 4] = buf[off..off + 4].try_into().unwrap();
    if le { u32::from_le_bytes(b) } else { u32::from_be_bytes(b) }
}
fn rd_i32(buf: &[u8], off: usize, le: bool) -> i32 {
    let b: [u8; 4] = buf[off..off + 4].try_into().unwrap();
    if le { i32::from_le_bytes(b) } else { i32::from_be_bytes(b) }
}
fn rd_f32(buf: &[u8], off: usize, le: bool) -> f32 {
    let b: [u8; 4] = buf[off..off + 4].try_into().unwrap();
    if le { f32::from_le_bytes(b) } else { f32::from_be_bytes(b) }
}
fn rd_f64(buf: &[u8], off: usize, le: bool) -> f64 {
    let b: [u8; 8] = buf[off..off + 8].try_into().unwrap();
    if le { f64::from_le_bytes(b) } else { f64::from_be_bytes(b) }
}

/// Read a tag's raw value bytes, following the offset for values > 4 bytes.
fn tag_bytes<R: Read + Seek>(
    r: &mut R,
    val_field: &[u8; 4],
    total: usize,
    always_offset: bool,
    le: bool,
) -> Option<Vec<u8>> {
    if !always_offset && total <= 4 {
        Some(val_field[..total].to_vec())
    } else {
        let offset = rd_u32(val_field, 0, le) as u64;
        let mut buf = vec![0u8; total];
        r.seek(SeekFrom::Start(offset)).ok()?;
        r.read_exact(&mut buf).ok()?;
        Some(buf)
    }
}

/// Read a TIFF tag as f64s. Handles DOUBLE/FLOAT/SHORT/LONG/(S)RATIONAL —
/// every type used by geotransform tags.
fn read_tag_f64s<R: Read + Seek>(
    r: &mut R,
    val_field: &[u8; 4],
    type_id: u16,
    count: usize,
    le: bool,
) -> Vec<f64> {
    let (bytes_each, always_offset) = match type_id {
        3 => (2usize, false), // SHORT
        4 => (4, false),      // LONG
        5 => (8, true),       // RATIONAL
        10 => (8, true),      // SRATIONAL
        11 => (4, false),     // FLOAT
        12 => (8, true),      // DOUBLE
        _ => return vec![],
    };
    let Some(raw) = tag_bytes(r, val_field, bytes_each * count, always_offset, le) else {
        return vec![];
    };
    (0..count)
        .map(|i| {
            let o = i * bytes_each;
            match type_id {
                3 => rd_u16(&raw, o, le) as f64,
                4 => rd_u32(&raw, o, le) as f64,
                5 => {
                    let den = rd_u32(&raw, o + 4, le) as f64;
                    if den == 0.0 { f64::NAN } else { rd_u32(&raw, o, le) as f64 / den }
                }
                10 => {
                    let den = rd_i32(&raw, o + 4, le) as f64;
                    if den == 0.0 { f64::NAN } else { rd_i32(&raw, o, le) as f64 / den }
                }
                11 => rd_f32(&raw, o, le) as f64,
                12 => rd_f64(&raw, o, le),
                _ => f64::NAN,
            }
        })
        .collect()
}

/// Read a SHORT-typed tag as u16s.
fn read_tag_u16s<R: Read + Seek>(
    r: &mut R,
    val_field: &[u8; 4],
    type_id: u16,
    count: usize,
    le: bool,
) -> Vec<u16> {
    if type_id != 3 {
        return vec![];
    }
    let Some(raw) = tag_bytes(r, val_field, count * 2, false, le) else {
        return vec![];
    };
    (0..count).map(|i| rd_u16(&raw, i * 2, le)).collect()
}

/// Read a SHORT- or LONG-typed tag as u32s.
fn read_tag_u32s<R: Read + Seek>(
    r: &mut R,
    val_field: &[u8; 4],
    type_id: u16,
    count: usize,
    le: bool,
) -> Vec<u32> {
    let bytes_each = match type_id {
        3 => 2usize,
        4 => 4,
        _ => return vec![],
    };
    let Some(raw) = tag_bytes(r, val_field, bytes_each * count, false, le) else {
        return vec![];
    };
    (0..count)
        .map(|i| {
            let o = i * bytes_each;
            if bytes_each == 2 { rd_u16(&raw, o, le) as u32 } else { rd_u32(&raw, o, le) }
        })
        .collect()
}

/// Read a NUL-terminated ASCII tag.
fn read_tag_ascii<R: Read + Seek>(
    r: &mut R,
    val_field: &[u8; 4],
    count: usize,
    le: bool,
) -> Option<String> {
    let raw = tag_bytes(r, val_field, count, false, le)?;
    String::from_utf8(raw).ok()
}

const GEO_KEY_GEOGRAPHIC_TYPE: u16 = 2048;
const GEO_KEY_PROJECTED_CS: u16 = 3072;

fn parse_epsg(geo_key_dir: &[u16]) -> Option<u32> {
    if geo_key_dir.len() < 4 {
        return None;
    }
    let n = geo_key_dir[3] as usize;
    for i in 0..n {
        let base = 4 + i * 4;
        if base + 3 >= geo_key_dir.len() {
            break;
        }
        let key_id = geo_key_dir[base];
        let location = geo_key_dir[base + 1]; // 0 = value inline
        let value = geo_key_dir[base + 3];
        if (key_id == GEO_KEY_PROJECTED_CS || key_id == GEO_KEY_GEOGRAPHIC_TYPE) && location == 0 {
            return Some(value as u32);
        }
    }
    None
}

struct TileData {
    width: usize,
    height: usize,
    /// Affine geotransform: [x_origin, px_w, x_skew, y_origin, y_skew, px_h].
    /// px_h is negative for north-up rasters.
    gt: [f64; 6],
    epsg: u32,
    nodata: Option<f32>,
    pixels: Vec<f32>,
}

fn decompress(data: &[u8], compression: u16) -> Result<Vec<u8>, String> {
    match compression {
        1 => Ok(data.to_vec()),
        8 | 32946 => {
            use flate2::read::ZlibDecoder;
            let mut out = Vec::new();
            ZlibDecoder::new(data)
                .read_to_end(&mut out)
                .map_err(|e| format!("deflate: {e}"))?;
            Ok(out)
        }
        c => Err(format!("unsupported compression {c}")),
    }
}

/// Undo horizontal differencing predictor for 32-bit samples.
fn undo_predictor_u32(data: &mut [u8], width: usize, le: bool) {
    for row in data.chunks_mut(width * 4) {
        let mut prev: u32 = 0;
        for chunk in row.chunks_mut(4) {
            if chunk.len() < 4 {
                break;
            }
            let diff = if le {
                u32::from_le_bytes(chunk.try_into().unwrap())
            } else {
                u32::from_be_bytes(chunk.try_into().unwrap())
            };
            let val = diff.wrapping_add(prev);
            chunk.copy_from_slice(&if le { val.to_le_bytes() } else { val.to_be_bytes() });
            prev = val;
        }
    }
}

/// Undo horizontal differencing predictor for 16-bit samples.
fn undo_predictor_u16(data: &mut [u8], width: usize, le: bool) {
    for row in data.chunks_mut(width * 2) {
        let mut prev: u16 = 0;
        for chunk in row.chunks_mut(2) {
            if chunk.len() < 2 {
                break;
            }
            let diff = if le {
                u16::from_le_bytes(chunk.try_into().unwrap())
            } else {
                u16::from_be_bytes(chunk.try_into().unwrap())
            };
            let val = diff.wrapping_add(prev);
            chunk.copy_from_slice(&if le { val.to_le_bytes() } else { val.to_be_bytes() });
            prev = val;
        }
    }
}

/// Undo the TIFF floating-point predictor (predictor=3), single band.
/// Bytes are stored as MSB-first byte planes with horizontal byte differencing;
/// reverse the differencing, then re-interleave into little-endian samples.
fn undo_float_predictor(data: &mut [u8], width: usize, bps: usize) {
    let row_bytes = width * bps;
    if row_bytes == 0 {
        return;
    }
    let mut tmp = vec![0u8; row_bytes];
    for row in data.chunks_mut(row_bytes) {
        if row.len() < row_bytes {
            break;
        }
        // Step 1: horizontal byte accumulation across the whole row (stride 1).
        for i in 1..row.len() {
            row[i] = row[i].wrapping_add(row[i - 1]);
        }
        // Step 2: de-plane MSB-first planes into little-endian samples.
        for s in 0..width {
            for b in 0..bps {
                tmp[s * bps + (bps - 1 - b)] = row[b * width + s];
            }
        }
        row.copy_from_slice(&tmp);
    }
}

/// Apply whatever horizontal predictor a tile/strip uses, in place.
fn apply_predictor(raw: &mut [u8], width: usize, bits: u16, predictor: u16, le: bool) {
    match predictor {
        2 => match bits {
            32 => undo_predictor_u32(raw, width, le),
            16 => undo_predictor_u16(raw, width, le),
            _ => {}
        },
        3 => undo_float_predictor(raw, width, (bits / 8) as usize),
        _ => {}
    }
}

/// Convert raw sample bytes to f32 given the TIFF sample type.
fn raw_to_f32(raw: &[u8], bits: u16, sample_format: u16, le: bool) -> Vec<f32> {
    match (bits, sample_format) {
        (32, 3) => raw.as_chunks::<4>().0.iter().map(|c| rd_f32(c, 0, le)).collect(),
        (32, _) => raw.as_chunks::<4>().0.iter().map(|c| rd_u32(c, 0, le) as f32).collect(),
        (16, 2) => raw.as_chunks::<2>().0.iter().map(|c| rd_u16(c, 0, le) as i16 as f32).collect(),
        (16, _) => raw.as_chunks::<2>().0.iter().map(|c| rd_u16(c, 0, le) as f32).collect(),
        (8, _) => raw.iter().map(|&b| b as f32).collect(),
        _ => vec![],
    }
}

/// Per-IFD tiling + codec metadata. Geotransform/CRS live only in IFD0, read
/// separately by [`read_cog_geo`].
struct IfdInfo {
    width: usize,
    height: usize,
    tile_w: usize,
    tile_h: usize,
    tile_offsets: Vec<u32>,
    tile_byte_counts: Vec<u32>,
    compression: u16,
    predictor: u16,
    bits: u16,
    sample_format: u16,
    next: u64,
}

/// Parse one IFD (dimensions, tiling, codec) and the offset of the next IFD.
fn parse_ifd<R: Read + Seek>(r: &mut R, offset: u64, le: bool) -> Result<IfdInfo, String> {
    r.seek(SeekFrom::Start(offset)).map_err(|e| e.to_string())?;
    let mut cnt = [0u8; 2];
    r.read_exact(&mut cnt).map_err(|e| e.to_string())?;
    let n = rd_u16(&cnt, 0, le) as usize;
    let mut ifd = vec![0u8; n * 12];
    r.read_exact(&mut ifd).map_err(|e| e.to_string())?;
    let mut next_buf = [0u8; 4];
    r.read_exact(&mut next_buf).map_err(|e| e.to_string())?;

    let first = |v: Vec<u32>, d: u32| v.into_iter().next().unwrap_or(d);
    let first16 = |v: Vec<u16>, d: u16| v.into_iter().next().unwrap_or(d);
    let mut info = IfdInfo {
        width: 0,
        height: 0,
        tile_w: 0,
        tile_h: 0,
        tile_offsets: vec![],
        tile_byte_counts: vec![],
        compression: 1,
        predictor: 1,
        bits: 32,
        sample_format: 3,
        next: rd_u32(&next_buf, 0, le) as u64,
    };
    for i in 0..n {
        let e = i * 12;
        let tag = rd_u16(&ifd, e, le);
        let ty = rd_u16(&ifd, e + 2, le);
        let count = rd_u32(&ifd, e + 4, le) as usize;
        let vf: [u8; 4] = ifd[e + 8..e + 12].try_into().unwrap();
        match tag {
            256 => info.width = first(read_tag_u32s(r, &vf, ty, 1, le), 0) as usize,
            257 => info.height = first(read_tag_u32s(r, &vf, ty, 1, le), 0) as usize,
            258 => info.bits = first16(read_tag_u16s(r, &vf, ty, 1, le), 32),
            259 => info.compression = first16(read_tag_u16s(r, &vf, ty, 1, le), 1),
            317 => info.predictor = first16(read_tag_u16s(r, &vf, ty, 1, le), 1),
            322 => info.tile_w = first(read_tag_u32s(r, &vf, ty, 1, le), 0) as usize,
            323 => info.tile_h = first(read_tag_u32s(r, &vf, ty, 1, le), 0) as usize,
            324 => info.tile_offsets = read_tag_u32s(r, &vf, ty, count, le),
            325 => info.tile_byte_counts = read_tag_u32s(r, &vf, ty, count, le),
            339 => info.sample_format = first16(read_tag_u16s(r, &vf, ty, 1, le), 3),
            _ => {}
        }
    }
    Ok(info)
}

/// Read geotransform, EPSG, and nodata from IFD0 (overviews inherit the CRS and
/// origin; only pixel size scales).
fn read_cog_geo<R: Read + Seek>(
    r: &mut R,
    offset: u64,
    le: bool,
) -> Result<([f64; 6], u32, Option<f32>), String> {
    r.seek(SeekFrom::Start(offset)).map_err(|e| e.to_string())?;
    let mut cnt = [0u8; 2];
    r.read_exact(&mut cnt).map_err(|e| e.to_string())?;
    let n = rd_u16(&cnt, 0, le) as usize;
    let mut ifd = vec![0u8; n * 12];
    r.read_exact(&mut ifd).map_err(|e| e.to_string())?;

    let mut scale: Vec<f64> = vec![];
    let mut tiepoint: Vec<f64> = vec![];
    let mut model_xform: Vec<f64> = vec![];
    let mut geo_key_dir: Vec<u16> = vec![];
    let mut nodata_str: Option<String> = None;
    for i in 0..n {
        let e = i * 12;
        let tag = rd_u16(&ifd, e, le);
        let ty = rd_u16(&ifd, e + 2, le);
        let count = rd_u32(&ifd, e + 4, le) as usize;
        let vf: [u8; 4] = ifd[e + 8..e + 12].try_into().unwrap();
        match tag {
            33550 => scale = read_tag_f64s(r, &vf, ty, count, le),
            33922 => tiepoint = read_tag_f64s(r, &vf, ty, count, le),
            34264 => model_xform = read_tag_f64s(r, &vf, ty, count, le),
            34735 => geo_key_dir = read_tag_u16s(r, &vf, ty, count, le),
            42113 => nodata_str = read_tag_ascii(r, &vf, count, le),
            _ => {}
        }
    }

    let gt = if scale.len() >= 2 && tiepoint.len() >= 6 {
        let px_w = scale[0];
        let px_h = -scale[1];
        [
            tiepoint[3] - tiepoint[0] * px_w,
            px_w,
            0.0,
            tiepoint[4] - tiepoint[1] * px_h,
            0.0,
            px_h,
        ]
    } else if model_xform.len() >= 16 {
        [
            model_xform[3],
            model_xform[0],
            model_xform[1],
            model_xform[7],
            model_xform[4],
            model_xform[5],
        ]
    } else {
        return Err("no geotransform tags".to_string());
    };
    let epsg = parse_epsg(&geo_key_dir).unwrap_or(0);
    let nodata = nodata_str
        .as_deref()
        .and_then(|s| s.trim_matches('\0').trim().parse::<f32>().ok());
    Ok((gt, epsg, nodata))
}

/// Bytes of front metadata to pull in one range request. COGs front-load all
/// IFDs and their tile-index arrays, so this comfortably covers the metadata
/// without touching the (hundreds of MB of) pixel data.
const COG_META_BYTES: u64 = 262_144;

/// Read a COG at a resolution suitable for `output_size` using HTTP Range: pull
/// the metadata, pick the smallest overview still at least `output_size` wide,
/// then fetch only that overview's tiles. Avoids downloading the full raster.
fn fetch_cog(
    client: &reqwest::blocking::Client,
    token: &str,
    url: &str,
    output_size: usize,
) -> Result<TileData, String> {
    let meta = http_range(client, token, url, 0, COG_META_BYTES - 1)?;
    let mut cur = Cursor::new(meta.as_slice());

    let mut hdr = [0u8; 8];
    cur.read_exact(&mut hdr).map_err(|e| e.to_string())?;
    let le = match &hdr[0..2] {
        b"II" => true,
        b"MM" => false,
        _ => return Err("not a TIFF".to_string()),
    };
    if rd_u16(&hdr, 2, le) != 42 {
        return Err("BigTIFF / unsupported magic".to_string());
    }
    let ifd0_off = rd_u32(&hdr, 4, le) as u64;

    let mut ifds = Vec::new();
    let mut off = ifd0_off;
    while off != 0 && ifds.len() < 32 {
        let info = parse_ifd(&mut cur, off, le)?;
        off = info.next;
        ifds.push(info);
    }
    if ifds.is_empty() {
        return Err("no IFDs".to_string());
    }
    let (gt0, epsg, nodata) = read_cog_geo(&mut cur, ifd0_off, le)?;

    // ifds[0] is full resolution; the rest are progressively smaller overviews.
    // Pick the smallest whose width still covers the output grid.
    let mut pick = 0usize;
    for (i, f) in ifds.iter().enumerate() {
        if f.width >= output_size {
            pick = i;
        }
    }
    let ifd = &ifds[pick];
    if ifd.width == 0 || ifd.height == 0 || ifd.tile_w == 0 || ifd.tile_h == 0 {
        return Err("chosen overview is not tiled".to_string());
    }

    let ratio_x = ifds[0].width as f64 / ifd.width as f64;
    let ratio_y = ifds[0].height as f64 / ifd.height as f64;
    let gt = [gt0[0], gt0[1] * ratio_x, 0.0, gt0[3], 0.0, gt0[5] * ratio_y];

    let (w, h, tw, th) = (ifd.width, ifd.height, ifd.tile_w, ifd.tile_h);
    // Metadata is supplied by the remote server. Reject unreasonable values
    // before they can overflow or allocate a giant pixel buffer.
    const MAX_COG_PIXELS: usize = 64 * 1024 * 1024;
    let pixel_count = w
        .checked_mul(h)
        .filter(|&count| count <= MAX_COG_PIXELS)
        .ok_or_else(|| "COG dimensions are unreasonable".to_string())?;
    let n_tiles_x = w.div_ceil(tw);
    let n_tiles_y = h.div_ceil(th);
    let mut pixels = vec![0.0f32; pixel_count];
    for ty in 0..n_tiles_y {
        for tx in 0..n_tiles_x {
            let idx = ty * n_tiles_x + tx;
            let (Some(&offset), Some(&byte_count)) =
                (ifd.tile_offsets.get(idx), ifd.tile_byte_counts.get(idx))
            else {
                continue;
            };
            if byte_count == 0 {
                continue;
            }
            let end = (offset as u64)
                .checked_add(byte_count as u64)
                .and_then(|end| end.checked_sub(1))
                .ok_or_else(|| "invalid COG tile range".to_string())?;
            let compressed = http_range(client, token, url, offset as u64, end)?;
            let mut raw = decompress(&compressed, ifd.compression)?;
            apply_predictor(&mut raw, tw, ifd.bits, ifd.predictor, le);
            let tile_px = raw_to_f32(&raw, ifd.bits, ifd.sample_format, le);
            let dst_x = tx * tw;
            let dst_y = ty * th;
            let copy_w = tw.min(w.saturating_sub(dst_x));
            let copy_h = th.min(h.saturating_sub(dst_y));
            for row in 0..copy_h {
                let src = row * tw;
                let dst = (dst_y + row) * w + dst_x;
                if src + copy_w <= tile_px.len() {
                    pixels[dst..dst + copy_w].copy_from_slice(&tile_px[src..src + copy_w]);
                }
            }
        }
    }

    Ok(TileData { width: w, height: h, gt, epsg, nodata, pixels })
}

// ─── CRS lookup ─────────────────────────────────────────────────────────────

/// proj4 strings for the EPSG codes seen in Scandinavian elevation data.
/// `proj4rs` uses radians for geographic (+proj=longlat) input/output.
pub(super) fn epsg_to_proj4(epsg: u32) -> Option<&'static str> {
    Some(match epsg {
        4326 | 4258 => "+proj=longlat +datum=WGS84 +no_defs",
        // WGS84 / Pseudo-Mercator — the canopy-height tiles' CRS.
        3857 => "+proj=merc +a=6378137 +b=6378137 +lat_ts=0 +lon_0=0 +x_0=0 +y_0=0 +k=1 +units=m +nadgrids=@null +no_defs",
        // SWEREF99 TM — Swedish national grid. 5845 = SWEREF99 TM + RH2000
        // height (compound); the horizontal component is identical to 3006.
        3006 | 5845 => "+proj=utm +zone=33 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs",
        // SWEREF99 local zones 12 00 – 23 15
        3007 => "+proj=tmerc +lat_0=0 +lon_0=12 +k=1 +x_0=150000 +y_0=0 +ellps=GRS80 +units=m +no_defs",
        3008 => "+proj=tmerc +lat_0=0 +lon_0=13.5 +k=1 +x_0=150000 +y_0=0 +ellps=GRS80 +units=m +no_defs",
        3009 => "+proj=tmerc +lat_0=0 +lon_0=15 +k=1 +x_0=150000 +y_0=0 +ellps=GRS80 +units=m +no_defs",
        3010 => "+proj=tmerc +lat_0=0 +lon_0=16.5 +k=1 +x_0=150000 +y_0=0 +ellps=GRS80 +units=m +no_defs",
        3011 => "+proj=tmerc +lat_0=0 +lon_0=18 +k=1 +x_0=150000 +y_0=0 +ellps=GRS80 +units=m +no_defs",
        3012 => "+proj=tmerc +lat_0=0 +lon_0=14.25 +k=1 +x_0=150000 +y_0=0 +ellps=GRS80 +units=m +no_defs",
        3013 => "+proj=tmerc +lat_0=0 +lon_0=15.75 +k=1 +x_0=150000 +y_0=0 +ellps=GRS80 +units=m +no_defs",
        3014 => "+proj=tmerc +lat_0=0 +lon_0=17.25 +k=1 +x_0=150000 +y_0=0 +ellps=GRS80 +units=m +no_defs",
        3015 => "+proj=tmerc +lat_0=0 +lon_0=18.75 +k=1 +x_0=150000 +y_0=0 +ellps=GRS80 +units=m +no_defs",
        3016 => "+proj=tmerc +lat_0=0 +lon_0=20.25 +k=1 +x_0=150000 +y_0=0 +ellps=GRS80 +units=m +no_defs",
        3017 => "+proj=tmerc +lat_0=0 +lon_0=21.75 +k=1 +x_0=150000 +y_0=0 +ellps=GRS80 +units=m +no_defs",
        3018 => "+proj=tmerc +lat_0=0 +lon_0=23.25 +k=1 +x_0=150000 +y_0=0 +ellps=GRS80 +units=m +no_defs",
        25832 | 32632 => "+proj=utm +zone=32 +datum=WGS84 +units=m +no_defs",
        25833 | 32633 => "+proj=utm +zone=33 +datum=WGS84 +units=m +no_defs",
        25834 | 32634 => "+proj=utm +zone=34 +datum=WGS84 +units=m +no_defs",
        3044 => "+proj=utm +zone=32 +ellps=GRS80 +units=m +no_defs",
        3045 => "+proj=utm +zone=33 +ellps=GRS80 +units=m +no_defs",
        3046 => "+proj=utm +zone=34 +ellps=GRS80 +units=m +no_defs",
        _ => return None,
    })
}

// ─── Merge + reproject ──────────────────────────────────────────────────────

/// Reproject and bilinear-sample every tile into a single WGS84 output grid.
/// Output is row-major, row 0 = north. Earlier tiles win on overlap.
fn merge_and_reproject(
    tiles: &[TileData],
    bbox: (f64, f64, f64, f64),
    output_size: usize,
    progress: &ProgressHandle,
) -> Result<Vec<f32>, String> {
    use proj4rs::{Proj, transform::transform};

    let (west, south, east, north) = bbox;
    let span_lon = east - west;
    let span_lat = north - south;

    let mut output = vec![f32::NAN; output_size * output_size];
    let mut filled = vec![false; output_size * output_size];

    let wgs84 = Proj::from_proj_string("+proj=longlat +datum=WGS84 +no_defs")
        .map_err(|e| format!("WGS84 proj init: {e}"))?;

    for (tile_index, tile) in tiles.iter().enumerate() {
        set_phase(progress, "Reprojecting terrain", tile_index, tiles.len(), PHASE_REPROJECT);

        let is_geographic = matches!(tile.epsg, 4326 | 4258 | 0);
        let native_proj: Option<Proj> = if is_geographic {
            None
        } else {
            match epsg_to_proj4(tile.epsg).map(Proj::from_proj_string) {
                Some(Ok(p)) => Some(p),
                Some(Err(e)) => {
                    warn!("[terrain] proj init EPSG {}: {e}", tile.epsg);
                    continue;
                }
                None => {
                    warn!("[terrain] unsupported EPSG {}", tile.epsg);
                    continue;
                }
            }
        };

        let gt = &tile.gt;
        let is_nodata =
            |v: f32| v.is_nan() || tile.nodata.is_some_and(|nd| (v - nd).abs() < 1e-3);

        // Tile bounds in WGS84 degrees.
        let tx1 = gt[0] + tile.width as f64 * gt[1];
        let ty1 = gt[3] + tile.height as f64 * gt[5];
        let corners = [(gt[0], gt[3]), (tx1, gt[3]), (gt[0], ty1), (tx1, ty1)];
        let (mut w84, mut e84, mut s84, mut n84) =
            (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
        for &(cx, cy) in &corners {
            let (lon, lat) = match &native_proj {
                Some(native) => {
                    let mut pt = (cx, cy);
                    if transform(native, &wgs84, &mut pt).is_err() {
                        continue;
                    }
                    (pt.0.to_degrees(), pt.1.to_degrees())
                }
                None => (cx, cy),
            };
            w84 = w84.min(lon);
            e84 = e84.max(lon);
            s84 = s84.min(lat);
            n84 = n84.max(lat);
        }

        let clamp = |v: f64| v.clamp(0.0, output_size as f64);
        let ox_start = clamp(((w84 - west) / span_lon * output_size as f64).floor()) as usize;
        let ox_end = clamp(((e84 - west) / span_lon * output_size as f64).ceil()) as usize;
        let oy_start = clamp(((north - n84) / span_lat * output_size as f64).floor()) as usize;
        let oy_end = clamp(((north - s84) / span_lat * output_size as f64).ceil()) as usize;
        if ox_start >= ox_end || oy_start >= oy_end {
            continue;
        }

        for oy in oy_start..oy_end {
            for ox in ox_start..ox_end {
                let idx = oy * output_size + ox;
                if filled[idx] {
                    continue;
                }
                let lon = west + (ox as f64 + 0.5) * span_lon / output_size as f64;
                let lat = north - (oy as f64 + 0.5) * span_lat / output_size as f64;
                let (nx, ny) = match &native_proj {
                    Some(native) => {
                        let mut pt = (lon.to_radians(), lat.to_radians());
                        if transform(&wgs84, native, &mut pt).is_err() {
                            continue;
                        }
                        pt
                    }
                    None => (lon, lat),
                };
                let px = (nx - gt[0]) / gt[1];
                let py = (ny - gt[3]) / gt[5];
                if px < 0.0
                    || py < 0.0
                    || px >= tile.width as f64 - 1.0
                    || py >= tile.height as f64 - 1.0
                {
                    continue;
                }
                let x0 = px as usize;
                let y0 = py as usize;
                let fx = (px - x0 as f64) as f32;
                let fy = (py - y0 as f64) as f32;
                let v00 = tile.pixels[y0 * tile.width + x0];
                let v10 = tile.pixels[y0 * tile.width + x0 + 1];
                let v01 = tile.pixels[(y0 + 1) * tile.width + x0];
                let v11 = tile.pixels[(y0 + 1) * tile.width + x0 + 1];
                if is_nodata(v00) || is_nodata(v10) || is_nodata(v01) || is_nodata(v11) {
                    continue;
                }
                output[idx] = v00 * (1.0 - fx) * (1.0 - fy)
                    + v10 * fx * (1.0 - fy)
                    + v01 * (1.0 - fx) * fy
                    + v11 * fx * fy;
                filled[idx] = true;
            }
        }
    }

    Ok(output)
}

// ─── Public entry point ─────────────────────────────────────────────────────

/// Fetch, decode, and reproject terrain for a point. Runs synchronously; call
/// from a background task. Reports progress via `progress`.
pub fn fetch_terrain(
    lat: f64,
    lon: f64,
    radius_km: f64,
    progress: &ProgressHandle,
) -> Result<TerrainGrid, String> {
    let _ = dotenvy::dotenv();
    let config = FetchConfig::from_env()?;

    let client = reqwest::blocking::Client::builder()
        .timeout(std::time::Duration::from_secs(config.timeout_secs))
        .build()
        .map_err(|e| e.to_string())?;

    set_phase(progress, "Authenticating with Lantmateriet", 0, 0, PHASE_AUTH);
    let token = get_token(&client, &config)?;

    set_phase(progress, "Searching elevation catalogue", 0, 0, PHASE_SEARCH);
    let bbox = bbox_from_center(lat, lon, radius_km)?;
    let urls = stac_search(&client, &config, &token, bbox)?;
    if urls.is_empty() {
        return Err("no elevation raster covers the selected area".to_string());
    }
    let url_count = urls.len();

    // Fetch overviews with a bounded worker pool. A shared atomic index hands
    // out tiles so a few connections stay busy instead of opening one socket
    // per tile. Results land in per-index slots to keep tile order (hence
    // overlap priority) deterministic. reqwest::blocking::Client is Send + Sync.
    use std::sync::atomic::{AtomicUsize, Ordering};

    set_phase(progress, "Downloading elevation tiles", 0, urls.len(), PHASE_DOWNLOAD);
    let workers = urls.len().min(config.download_workers);
    let output_size = config.output_size;
    let next = AtomicUsize::new(0);
    let slots: Vec<Mutex<Option<TileData>>> =
        (0..urls.len()).map(|_| Mutex::new(None)).collect();

    std::thread::scope(|scope| {
        for _ in 0..workers {
            let (next, slots, urls, client, token) = (&next, &slots, &urls, &client, &token);
            scope.spawn(move || loop {
                let i = next.fetch_add(1, Ordering::Relaxed);
                if i >= urls.len() {
                    break;
                }
                let url = &urls[i];
                if let Ok(mut p) = progress.lock() {
                    p.current = file_name(url);
                }
                match fetch_cog(client, token, url, output_size) {
                    Ok(tile) => *slots[i].lock().unwrap() = Some(tile),
                    Err(e) => warn!("[terrain] skip {url}: {e}"),
                }
                if let Ok(mut p) = progress.lock() {
                    p.done += 1;
                }
            });
        }
    });

    let tiles: Vec<TileData> = slots
        .into_iter()
        .filter_map(|slot| slot.into_inner().unwrap())
        .collect();
    if tiles.is_empty() {
        return Err("all tile downloads/reads failed".to_string());
    }

    let mut heights = merge_and_reproject(&tiles, bbox, config.output_size, progress)?;
    if !heights.iter().any(|v| v.is_finite()) {
        return Err("elevation grid contains no valid samples".to_string());
    }

    // Normalise to 0 at the lowest point; fill gaps with 0 (matches the old
    // Python behaviour so downstream mesh/sampling never see NaN).
    let min_h = heights.iter().copied().filter(|v| v.is_finite()).fold(f32::INFINITY, f32::min);
    let covered = heights.iter().filter(|v| v.is_finite()).count();
    let max_h = heights.iter().copied().filter(|v| v.is_finite()).fold(f32::NEG_INFINITY, f32::max);
    for h in &mut heights {
        *h = if h.is_finite() { *h - min_h } else { 0.0 };
    }

    // Single summary line for the whole fetch.
    info!(
        "[terrain] {}/{} tiles, {}x{} grid, {:.0}% covered, elevation {:.0}-{:.0} m",
        tiles.len(),
        url_count,
        config.output_size,
        config.output_size,
        100.0 * covered as f32 / heights.len() as f32,
        min_h,
        max_h,
    );

    set_phase(progress, "Terrain ready", 1, 1, PHASE_TERRAIN_READY);
    Ok(TerrainGrid { heights_m: heights, size: config.output_size })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Live end-to-end fetch. Network + .env required, so ignored by default.
    /// Run: `cargo test --lib terrain::source::tests::smoke_fetch -- --ignored --nocapture`
    #[test]
    #[ignore]
    fn smoke_fetch_stockholm() {
        let progress: ProgressHandle = Arc::new(Mutex::new(Progress::default()));
        let grid = fetch_terrain(59.3293, 18.0686, 10.0, &progress).expect("fetch");
        let finite: Vec<f32> = grid.heights_m.iter().copied().filter(|v| v.is_finite()).collect();
        let covered = finite.len();
        let max = finite.iter().copied().fold(f32::NEG_INFINITY, f32::max);
        eprintln!(
            "grid {}x{}, covered {}/{}, max relief {:.1} m",
            grid.size,
            grid.size,
            covered,
            grid.heights_m.len(),
            max
        );
        assert_eq!(grid.heights_m.len(), grid.size * grid.size);
        // Every cell is finite (gaps filled with 0) and Stockholm relief is
        // gentle — a correct decode lands well under 200 m, garbage would not.
        assert_eq!(covered, grid.heights_m.len());
        assert!(max > 1.0, "terrain is suspiciously flat: {max}");
        assert!(max < 200.0, "relief implausible — decode likely wrong: {max}");
    }
}
