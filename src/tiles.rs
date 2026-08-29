//! OSM slippy-map tiles: Web Mercator projection + async tile fetch/cache.
//!
//! Fetches standard XYZ tile PNGs (`https://tile.openstreetmap.org/{z}/{x}/{y}.png`)
//! over HTTP, decodes them, and hands back Bevy `Image`s — the same
//! background-task shape `crate::terrain::source` already uses for
//! Lantmäteriet GeoTIFFs (fetch on `IoTaskPool`, poll once a frame, never
//! block the main thread).
//!
//! Replaces the old hand-drawn, rasterized Sweden outline (`sweden_geo`'s
//! `rasterize`/`CITIES`/`lonlat_to_pixel`) — that data still holds Sweden's
//! coastline polygon (kept for `point_in_sweden`, which still gates where a
//! network area may be drawn), but the *displayed* map is now live OSM
//! tiles, so it has real roads, place names, and detail at any zoom instead
//! of a fixed-resolution texture.

use std::collections::HashMap;
use std::time::{Duration, Instant};

use bevy::{
    asset::RenderAssetUsages,
    prelude::*,
    render::render_resource::{Extent3d, TextureDimension, TextureFormat},
    tasks::{IoTaskPool, Task, futures_lite::future},
};

/// Pixel size of one OSM tile (standard, non-retina).
pub const TILE_SIZE: f32 = 256.0;

/// Shallowest (most zoomed-out) tile zoom this app will fetch.
pub const MIN_ZOOM: u8 = 4;
/// Deepest (most zoomed-in) tile zoom this app will fetch — 19 is standard
/// OSM max, individual buildings/driveways visible.
pub const MAX_ZOOM: u8 = 19;

/// Identifies one tile: zoom level + tile-grid column/row.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct TileKey {
    pub z: u8,
    pub x: u32,
    pub y: u32,
}

impl TileKey {
    /// `true` if `(x, y)` is a valid tile index at this zoom (`0..2^z`).
    /// Web Mercator only covers ±85.05° latitude and doesn't wrap in `y`;
    /// `x` in principle wraps around the antimeridian, but this app only
    /// ever looks at Sweden, so out-of-range is simplest treated as "skip".
    pub fn in_range(&self) -> bool {
        let n = 1u32 << self.z;
        self.x < n && self.y < n
    }
}

// ─── Web Mercator projection ────────────────────────────────────────────────

/// Project lon/lat to *world pixel* space at `zoom` — origin at the
/// northwest corner of the map (180°W, ~85.05°N), +X east, +Y south, total
/// span `TILE_SIZE * 2^zoom` in each axis. Standard XYZ tile convention.
pub fn lonlat_to_world_px(lon: f64, lat: f64, zoom: u8) -> Vec2 {
    let n = (1u64 << zoom) as f64 * TILE_SIZE as f64;
    let x = (lon + 180.0) / 360.0 * n;
    let lat_rad = lat.to_radians();
    let y = (1.0 - (lat_rad.tan() + 1.0 / lat_rad.cos()).ln() / std::f64::consts::PI) / 2.0 * n;
    Vec2::new(x as f32, y as f32)
}

/// Inverse of `lonlat_to_world_px`.
pub fn world_px_to_lonlat(px: Vec2, zoom: u8) -> (f64, f64) {
    let n = (1u64 << zoom) as f64 * TILE_SIZE as f64;
    let lon = px.x as f64 / n * 360.0 - 180.0;
    let y_frac = px.y as f64 / n;
    let lat = (std::f64::consts::PI * (1.0 - 2.0 * y_frac)).sinh().atan().to_degrees();
    (lon, lat)
}

// ─── Fetch / cache ──────────────────────────────────────────────────────────

type FetchResult = Result<(TileKey, Vec<u8>, u32), (TileKey, String)>;

/// How long a failed tile sits out before it's eligible to be retried.
/// Without this, one transient hiccup (a timeout, a momentary server-side
/// throttle) permanently blacklists that tile for the rest of the session —
/// it never gets a second chance, so its spot stays a blurry placeholder
/// forever even once the network is fine again. This is what "zooming
/// becomes hazy and stays that way" turned out to be.
const RETRY_COOLDOWN: Duration = Duration::from_secs(8);

/// How many tile fetches may be in flight at once. Firing one request per
/// wanted tile with no cap means a single pan/zoom can burst 15-20+
/// simultaneous connections at OSM's tile server — exactly the kind of
/// bulk/bursty use their tile usage policy asks clients not to do, and
/// almost certainly why fetches were failing with "error sending request"
/// under normal use. Real map clients (browsers, Leaflet) cap this the same
/// way; 6 matches a typical browser's per-host connection limit.
const MAX_CONCURRENT_FETCHES: usize = 6;

/// Live per-tile fetch state, and the cache of tiles already decoded.
#[derive(Resource, Default)]
pub struct TileCache {
    /// Tiles already decoded and uploaded — ready to draw.
    pub ready: HashMap<TileKey, Handle<Image>>,
    /// Tiles currently being fetched on a background task.
    in_flight: HashMap<TileKey, Task<FetchResult>>,
    /// Tiles wanted but not yet started, because `MAX_CONCURRENT_FETCHES`
    /// was already in flight — picked up as slots free up in `request`.
    queued: Vec<TileKey>,
    /// Tiles whose fetch failed, and when — eligible for a retry once
    /// `RETRY_COOLDOWN` has passed (see its doc comment).
    failed: HashMap<TileKey, Instant>,
}

impl TileCache {
    /// Request `key`, unless it's already ready, in flight, queued, or
    /// still within its retry cooldown. Starts fetching immediately if a
    /// concurrency slot is free, otherwise queues it for when one opens up.
    pub fn request(&mut self, key: TileKey) {
        if !key.in_range() || self.ready.contains_key(&key) || self.in_flight.contains_key(&key) {
            return;
        }
        if let Some(&failed_at) = self.failed.get(&key) {
            if failed_at.elapsed() < RETRY_COOLDOWN {
                return;
            }
        }
        if self.queued.contains(&key) {
            return;
        }
        if self.in_flight.len() < MAX_CONCURRENT_FETCHES {
            self.start_fetch(key);
        } else {
            self.queued.push(key);
        }
    }

    fn start_fetch(&mut self, key: TileKey) {
        self.failed.remove(&key);
        self.in_flight.insert(key, IoTaskPool::get().spawn(async move { fetch_tile(key) }));
    }

    /// Pull queued tiles into `in_flight` while concurrency slots are free.
    /// Called once a frame after polling finished fetches, so a slot that
    /// just freed up gets reused immediately instead of sitting idle.
    fn fill_free_slots(&mut self) {
        while self.in_flight.len() < MAX_CONCURRENT_FETCHES {
            let Some(key) = self.queued.pop() else { break };
            self.start_fetch(key);
        }
    }
}

/// Blocking HTTP fetch + PNG decode for one tile. Runs on a background task
/// (`IoTaskPool`) — never the main thread.
fn fetch_tile(key: TileKey) -> FetchResult {
    let url = format!("https://tile.openstreetmap.org/{}/{}/{}.png", key.z, key.x, key.y);
    let client = reqwest::blocking::Client::builder()
        .timeout(std::time::Duration::from_secs(15))
        // OpenStreetMap's tile usage policy requires a descriptive
        // User-Agent identifying the application:
        // https://operations.osmfoundation.org/policies/tiles/
        .user_agent("Kolbe/0.1 (github.com — EDTH Hamburg 2026 hackathon; drone-mesh-sim area picker)")
        .build()
        .map_err(|e| (key, e.to_string()))?;
    let bytes = client
        .get(&url)
        .send()
        .and_then(|r| r.error_for_status())
        .map_err(|e| (key, format!("{url}: {e}")))?
        .bytes()
        .map_err(|e| (key, e.to_string()))?;
    let rgba = image::load_from_memory(&bytes).map_err(|e| (key, e.to_string()))?.to_rgba8();
    let width = rgba.width();
    Ok((key, rgba.into_raw(), width))
}

/// Poll in-flight tile fetches; any that finished this frame get uploaded
/// into `Assets<Image>` and moved into `TileCache::ready`.
pub fn poll_tile_fetches(mut cache: ResMut<TileCache>, mut images: ResMut<Assets<Image>>) {
    let pending: Vec<TileKey> = cache.in_flight.keys().copied().collect();
    for key in pending {
        let Some(task) = cache.in_flight.get_mut(&key) else { continue };
        let Some(result) = future::block_on(future::poll_once(task)) else { continue };
        cache.in_flight.remove(&key);
        match result {
            Ok((key, rgba, width)) => {
                let height = (rgba.len() as u32 / 4) / width.max(1);
                let handle = images.add(Image::new(
                    Extent3d { width, height, depth_or_array_layers: 1 },
                    TextureDimension::D2,
                    rgba,
                    TextureFormat::Rgba8UnormSrgb,
                    RenderAssetUsages::default(),
                ));
                cache.ready.insert(key, handle);
            }
            Err((key, message)) => {
                warn!("[tiles] failed to fetch {key:?}: {message}");
                cache.failed.insert(key, Instant::now());
            }
        }
    }
    cache.fill_free_slots();
}
