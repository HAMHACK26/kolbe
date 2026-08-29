//! Tree-height raster detection and terrain vegetation rendering.

use super::TerrainHeightMap;
use crate::area::ScenarioArea;
use bevy::{log::info, prelude::*};
use std::{cmp::Ordering, collections::HashMap, io::Cursor, time::Duration};
use tiff::decoder::{Decoder, DecodingResult};

const DEFAULT_MIN_HEIGHT_M: f32 = 5.0;
const DEFAULT_MIN_SPACING_M: f32 = 8.0;
const DEFAULT_MAX_TREES: usize = 10_000;

/// Tree heights in decimetres. Row zero is the north edge.
#[derive(Clone, Debug)]
pub struct TreeHeightRaster {
    pub heights_dm: Vec<i16>,
    pub width: usize,
    pub height: usize,
    pub pixel_size_m: f32,
    pub area_width_m: f32,
    pub area_height_m: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TreeInstance {
    pub x_km: f32,
    pub z_km: f32,
    pub height_m: f32,
    pub crown_radius_m: f32,
}

#[derive(Resource, Default)]
pub struct VegetationMap {
    pub trees: Vec<TreeInstance>,
}

#[derive(Clone, Copy, Debug)]
pub struct DetectionConfig {
    pub min_height_m: f32,
    pub min_spacing_m: f32,
    pub max_trees: usize,
}

impl DetectionConfig {
    pub fn from_env() -> Self {
        Self {
            min_height_m: env_f32("TREE_MIN_HEIGHT_M", DEFAULT_MIN_HEIGHT_M, 0.1, 100.0),
            min_spacing_m: env_f32("TREE_MIN_SPACING_M", DEFAULT_MIN_SPACING_M, 1.0, 100.0),
            max_trees: std::env::var("TREE_MAX_COUNT")
                .ok()
                .and_then(|v| v.parse().ok())
                .unwrap_or(DEFAULT_MAX_TREES)
                .min(100_000),
        }
    }
}

/// Optional offline raster source used by the asynchronous terrain load. The
/// first three whitespace-separated fields are width, height, and pixel size
/// in metres; all remaining fields are signed tree heights in decimetres.
pub fn load_configured(area: &ScenarioArea) -> Result<VegetationMap, String> {
    if !vegetation_enabled() {
        return Ok(VegetationMap::default());
    }
    let raster = if let Ok(path) = std::env::var("TREE_HEIGHT_RASTER_PATH") {
        read_text_raster(&path, area.size_km)?
    } else {
        fetch_tree_height_raster(area)?
    };
    let width = raster.width;
    let height = raster.height;
    let trees = detect_trees(&raster, DetectionConfig::from_env())?;
    info!(
        "tree-height raster {width}x{height}: detected {} trees",
        trees.len()
    );
    Ok(VegetationMap { trees })
}

fn read_text_raster(path: &str, area_size_km: f32) -> Result<TreeHeightRaster, String> {
    let text = std::fs::read_to_string(&path)
        .map_err(|error| format!("could not read tree-height raster {path}: {error}"))?;
    let mut fields = text.split_whitespace();
    let width = parse_field::<usize>(&mut fields, "width")?;
    let height = parse_field::<usize>(&mut fields, "height")?;
    let pixel_size_m = parse_field::<f32>(&mut fields, "pixel size")?;
    let heights_dm = fields
        .map(|value| {
            value
                .parse::<i16>()
                .map_err(|_| format!("invalid tree height: {value}"))
        })
        .collect::<Result<Vec<_>, _>>()?;
    Ok(TreeHeightRaster {
        heights_dm,
        width,
        height,
        pixel_size_m,
        area_width_m: area_size_km * 1_000.0,
        area_height_m: area_size_km * 1_000.0,
    })
}

fn fetch_tree_height_raster(area: &ScenarioArea) -> Result<TreeHeightRaster, String> {
    let endpoint = std::env::var("TREE_HEIGHT_SERVICE_URL").unwrap_or_else(|_| {
        "https://geodata.skogsstyrelsen.se/arcgis/rest/services/Publikt/Tradhojd_3_1/ImageServer/exportImage".into()
    });
    let resolution_m = env_f32("TREE_RASTER_RESOLUTION_M", 5.0, 2.0, 100.0);
    let pixels = ((area.size_km * 1_000.0 / resolution_m).ceil() as usize).clamp(2, 4_000);
    let [west, south, east, north] = area.wgs84_bbox();
    let bbox = format!("{west},{south},{east},{north}");
    let size = format!("{pixels},{pixels}");
    let client = reqwest::blocking::Client::builder()
        .timeout(Duration::from_secs(120))
        .build()
        .map_err(|e| e.to_string())?;
    let bytes = client
        .get(endpoint)
        .query(&[
            ("bbox", bbox.as_str()),
            ("bboxSR", "4326"),
            ("imageSR", "4326"),
            ("size", size.as_str()),
            ("format", "tiff"),
            ("pixelType", "S16"),
            ("interpolation", "RSP_NearestNeighbor"),
            ("f", "image"),
        ])
        .send()
        .and_then(|response| response.error_for_status())
        .map_err(|e| format!("tree-height service request failed: {e}"))?
        .bytes()
        .map_err(|e| format!("tree-height download failed: {e}"))?;
    decode_tiff(&bytes, area.size_km)
}

fn decode_tiff(bytes: &[u8], area_size_km: f32) -> Result<TreeHeightRaster, String> {
    let mut decoder =
        Decoder::new(Cursor::new(bytes)).map_err(|e| format!("invalid tree-height TIFF: {e}"))?;
    let (width, height) = decoder.dimensions().map_err(|e| e.to_string())?;
    let heights_dm = match decoder.read_image().map_err(|e| e.to_string())? {
        DecodingResult::I16(values) => values,
        DecodingResult::U16(values) => values
            .into_iter()
            .map(|v| v.min(i16::MAX as u16) as i16)
            .collect(),
        DecodingResult::U8(values) => values.into_iter().map(i16::from).collect(),
        other => return Err(format!("unsupported tree-height TIFF pixels: {other:?}")),
    };
    let width = width as usize;
    let height = height as usize;
    Ok(TreeHeightRaster {
        heights_dm,
        width,
        height,
        pixel_size_m: area_size_km * 1_000.0 / width as f32,
        area_width_m: area_size_km * 1_000.0,
        area_height_m: area_size_km * 1_000.0,
    })
}

fn parse_field<T: std::str::FromStr>(
    fields: &mut std::str::SplitWhitespace<'_>,
    name: &str,
) -> Result<T, String> {
    fields
        .next()
        .ok_or_else(|| format!("tree-height raster is missing {name}"))?
        .parse()
        .map_err(|_| format!("tree-height raster has an invalid {name}"))
}

#[derive(Component)]
pub struct TerrainTree;

/// Find canopy maxima, then resolve overlapping crowns tallest-first.
pub fn detect_trees(
    raster: &TreeHeightRaster,
    config: DetectionConfig,
) -> Result<Vec<TreeInstance>, String> {
    if config.max_trees == 0 {
        return Ok(Vec::new());
    }
    if raster.width == 0 || raster.height == 0 {
        return Ok(Vec::new());
    }
    if raster.heights_dm.len() != raster.width * raster.height {
        return Err("tree-height raster dimensions do not match its data".into());
    }
    if !raster.pixel_size_m.is_finite() || raster.pixel_size_m <= 0.0 {
        return Err("tree-height raster pixel size must be positive".into());
    }
    let radius = ((config.min_spacing_m * 0.5) / raster.pixel_size_m)
        .ceil()
        .max(1.0) as isize;
    let minimum_dm = (config.min_height_m * 10.0).ceil() as i16;
    let mut candidates = Vec::new();
    for row in 0..raster.height {
        for column in 0..raster.width {
            let value = raster.heights_dm[row * raster.width + column];
            if value >= minimum_dm && is_local_maximum(raster, column, row, value, radius) {
                candidates.push(to_tree(raster, column, row, value));
            }
        }
    }
    candidates.sort_by(|a, b| {
        b.height_m
            .partial_cmp(&a.height_m)
            .unwrap_or(Ordering::Equal)
            .then_with(|| a.z_km.total_cmp(&b.z_km))
            .then_with(|| a.x_km.total_cmp(&b.x_km))
    });
    let spacing_sq = (config.min_spacing_m / 1_000.0).powi(2);
    let spacing_km = config.min_spacing_m / 1_000.0;
    let mut accepted: Vec<TreeInstance> =
        Vec::with_capacity(config.max_trees.min(candidates.len()));
    let mut spatial: HashMap<(i32, i32), Vec<TreeInstance>> = HashMap::new();
    for candidate in candidates {
        let cell = (
            (candidate.x_km / spacing_km).floor() as i32,
            (candidate.z_km / spacing_km).floor() as i32,
        );
        let overlaps = (-1..=1).any(|dx| {
            (-1..=1).any(|dz| {
                spatial
                    .get(&(cell.0 + dx, cell.1 + dz))
                    .is_some_and(|trees| {
                        trees.iter().any(|tree| {
                            (tree.x_km - candidate.x_km).powi(2)
                                + (tree.z_km - candidate.z_km).powi(2)
                                < spacing_sq
                        })
                    })
            })
        });
        if overlaps {
            continue;
        }
        spatial.entry(cell).or_default().push(candidate);
        accepted.push(candidate);
        if accepted.len() == config.max_trees {
            break;
        }
    }
    Ok(accepted)
}

fn is_local_maximum(
    raster: &TreeHeightRaster,
    column: usize,
    row: usize,
    value: i16,
    radius: isize,
) -> bool {
    for dy in -radius..=radius {
        for dx in -radius..=radius {
            if dx == 0 && dy == 0 {
                continue;
            }
            let (x, y) = (column as isize + dx, row as isize + dy);
            if x < 0 || y < 0 || x >= raster.width as isize || y >= raster.height as isize {
                continue;
            }
            let other = raster.heights_dm[y as usize * raster.width + x as usize];
            // A deterministic north-west winner collapses flat plateaus.
            if other > value || (other == value && (dy < 0 || (dy == 0 && dx < 0))) {
                return false;
            }
        }
    }
    true
}

fn to_tree(raster: &TreeHeightRaster, column: usize, row: usize, height_dm: i16) -> TreeInstance {
    let x_m = (column as f32 + 0.5) * raster.pixel_size_m - raster.area_width_m * 0.5;
    // Raster rows run north to south, while Bevy +Z points north.
    let z_m = raster.area_height_m * 0.5 - (row as f32 + 0.5) * raster.pixel_size_m;
    let height_m = height_dm as f32 / 10.0;
    TreeInstance {
        x_km: x_m / 1_000.0,
        z_km: z_m / 1_000.0,
        height_m,
        crown_radius_m: (height_m * 0.2).clamp(1.0, 8.0),
    }
}

/// Render two directly batched entities per tree, without parent entities.
pub fn spawn_trees(
    mut commands: Commands,
    height_map: Res<TerrainHeightMap>,
    vegetation: Res<VegetationMap>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    if !vegetation_enabled() || vegetation.trees.is_empty() {
        return;
    }
    let trunk_mesh = meshes.add(Cylinder::new(1.0, 1.0));
    let crown_mesh = meshes.add(Cone::new(1.0, 1.0));
    let trunk_mat = materials.add(StandardMaterial {
        base_color: Color::srgb_u8(0x6f, 0x4e, 0x37),
        perceptual_roughness: 1.0,
        ..default()
    });
    let crown_mat = materials.add(StandardMaterial {
        base_color: Color::srgb_u8(0x2f, 0x6f, 0x3e),
        perceptual_roughness: 0.95,
        ..default()
    });
    let visibility_range =
        VisibilityRange::abrupt(0.0, env_f32("TREE_LOD_DISTANCE_KM", 40.0, 0.1, 1_000.0));
    for tree in &vegetation.trees {
        let (height, crown_radius) = (tree.height_m / 1_000.0, tree.crown_radius_m / 1_000.0);
        let (trunk_height, crown_height) = (height * 0.42, height * 0.78);
        let ground = height_map.height_at(tree.x_km, tree.z_km);
        let trunk_radius = (crown_radius * 0.16).max(0.0008);
        commands.spawn((
            Mesh3d(trunk_mesh.clone()),
            MeshMaterial3d(trunk_mat.clone()),
            Transform::from_xyz(tree.x_km, ground + trunk_height * 0.5, tree.z_km)
                .with_scale(Vec3::new(trunk_radius, trunk_height, trunk_radius)),
            visibility_range.clone(),
            TerrainTree,
        ));
        commands.spawn((
            Mesh3d(crown_mesh.clone()),
            MeshMaterial3d(crown_mat.clone()),
            Transform::from_xyz(tree.x_km, ground + height - crown_height * 0.5, tree.z_km)
                .with_scale(Vec3::new(crown_radius, crown_height, crown_radius)),
            visibility_range.clone(),
            TerrainTree,
        ));
    }
    info!("spawned {} terrain trees", vegetation.trees.len());
}

pub fn cleanup_trees(mut commands: Commands, trees: Query<Entity, With<TerrainTree>>) {
    for entity in &trees {
        commands.entity(entity).despawn();
    }
    commands.remove_resource::<VegetationMap>();
}

fn vegetation_enabled() -> bool {
    std::env::var("VEGETATION_ENABLED")
        .map(|v| !matches!(v.to_ascii_lowercase().as_str(), "0" | "false" | "no"))
        .unwrap_or(true)
}
fn env_f32(name: &str, default: f32, min: f32, max: f32) -> f32 {
    std::env::var(name)
        .ok()
        .and_then(|v| v.parse().ok())
        .filter(|v: &f32| v.is_finite())
        .unwrap_or(default)
        .clamp(min, max)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn raster(values: &[i16], width: usize, height: usize) -> TreeHeightRaster {
        TreeHeightRaster {
            heights_dm: values.to_vec(),
            width,
            height,
            pixel_size_m: 2.0,
            area_width_m: width as f32 * 2.0,
            area_height_m: height as f32 * 2.0,
        }
    }
    fn config(spacing: f32, max_trees: usize) -> DetectionConfig {
        DetectionConfig {
            min_height_m: 5.0,
            min_spacing_m: spacing,
            max_trees,
        }
    }

    #[test]
    fn finds_maximum_above_threshold() {
        let trees = detect_trees(
            &raster(&[0, 20, 0, 40, 120, 30, 0, 20, 0], 3, 3),
            config(2.0, 10),
        )
        .unwrap();
        assert_eq!(trees.len(), 1);
        assert_eq!(trees[0].height_m, 12.0);
    }
    #[test]
    fn north_row_maps_to_positive_z() {
        let trees = detect_trees(&raster(&[100, 0, 0, 80], 2, 2), config(2.0, 10)).unwrap();
        let north = trees.iter().find(|t| t.height_m == 10.0).unwrap();
        assert!(north.z_km > 0.0);
        assert!(north.x_km < 0.0);
    }
    #[test]
    fn spacing_prefers_taller_tree() {
        let trees = detect_trees(&raster(&[0, 120, 0, 110, 0], 5, 1), config(5.0, 10)).unwrap();
        assert_eq!(trees.len(), 1);
        assert_eq!(trees[0].height_m, 12.0);
    }
    #[test]
    fn thinning_is_deterministic_and_tallest_first() {
        let input = raster(&[100, 0, 130, 0, 120], 5, 1);
        let a = detect_trees(&input, config(2.0, 2)).unwrap();
        let b = detect_trees(&input, config(2.0, 2)).unwrap();
        assert_eq!(a, b);
        assert_eq!(
            a.iter().map(|t| t.height_m).collect::<Vec<_>>(),
            vec![13.0, 12.0]
        );
    }
    #[test]
    fn rejects_bad_dimensions() {
        assert!(detect_trees(&raster(&[100], 2, 2), config(2.0, 10)).is_err());
    }
}
