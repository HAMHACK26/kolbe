//! Procedural vegetation.
//!
//! Real canopy data was tried and dropped deliberately. Nothing in the
//! simulation reads tree positions — link budget is distance and antenna gain
//! only (see `networking::detect_links_and_send_headers`), so trees are purely
//! scenery. At the densities that actually look like forest, per-tree fidelity
//! from a height raster is invisible, while the fetch cost was minutes. A
//! deterministic scatter gives the same picture for free.
//!
//! Every tree is the same height by design: this is ground cover for scale and
//! parallax, not a forestry model.

use super::TerrainHeightMap;
use std::collections::HashMap;

use bevy::{asset::RenderAssetUsages, log::info, mesh::Indices, prelude::*};

/// Distance between trees before jitter, in metres. The dominant cost knob:
/// tree count scales with its inverse square, so halving this quadruples both
/// triangle count and vertex memory. Lower it for a thicker forest if your GPU
/// has the headroom.
const DEFAULT_SPACING_M: f32 = 40.0;
const DEFAULT_TREE_HEIGHT_M: f32 = 50.0;
/// Fraction of the spacing a tree may wander from its grid slot. Enough to
/// break up the lattice without opening gaps.
const JITTER_FRACTION: f32 = 0.45;
/// Crown radius as a fraction of total height. At the default spacing this puts
/// neighbouring crowns just about in contact, so the canopy reads as closed.
const CROWN_RADIUS_FRACTION: f32 = 0.28;
const TRUNK_HEIGHT_FRACTION: f32 = 0.34;
const TRUNK_RADIUS_FRACTION: f32 = 0.030;
const CROWN_SIDES: usize = 5;
const TRUNK_SIDES: usize = 3;
/// Side length of one merged forest mesh, in kilometres. Trees are welded into
/// per-chunk meshes so a dense forest costs a few hundred entities rather than
/// a few hundred thousand, while staying small enough to frustum-cull well.
const CHUNK_KM: f32 = 1.0;
/// A tree is dropped when the ground under it is both near the terrain floor
/// and locally flat — that combination is lake or sea, not low ground.
const WATER_MAX_HEIGHT_M: f32 = 0.75;
const WATER_MAX_RELIEF_M: f32 = 0.40;
/// Horizontal offset used to measure local relief for the water test.
const RELIEF_SAMPLE_KM: f32 = 0.03;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TreeInstance {
    pub x_km: f32,
    pub z_km: f32,
    pub height_m: f32,
    pub crown_radius_m: f32,
}

#[derive(Component)]
pub struct TerrainTree;

/// Compact canopy data used by radio line-of-sight checks. Rendering remains
/// chunked, while radio queries use this spatial index instead of a mesh read.
#[derive(Clone, Copy)]
pub struct RadioCanopy {
    pub position: Vec3,
    pub radius_km: f32,
}

#[derive(Resource, Default)]
pub struct RadioCanopies(pub HashMap<(i32, i32), Vec<RadioCanopy>>);

const RADIO_CELL_KM: f32 = 0.05;

impl RadioCanopies {
    fn cell(position: Vec3) -> (i32, i32) {
        ((position.x / RADIO_CELL_KM).floor() as i32, (position.z / RADIO_CELL_KM).floor() as i32)
    }

    pub fn blocks_path(&self, from: Vec3, to: Vec3) -> bool {
        let horizontal = Vec2::new(to.x - from.x, to.z - from.z).length();
        let steps = (horizontal / (RADIO_CELL_KM * 0.5)).ceil().max(1.0) as usize;
        for step in 0..=steps {
            let point = from.lerp(to, step as f32 / steps as f32);
            let (cx, cz) = Self::cell(point);
            for x in cx - 1..=cx + 1 {
                for z in cz - 1..=cz + 1 {
                    let Some(canopies) = self.0.get(&(x, z)) else { continue };
                    for canopy in canopies {
                        let horizontal_gap = Vec2::new(
                            point.x - canopy.position.x,
                            point.z - canopy.position.z,
                        )
                        .length();
                        if horizontal_gap <= canopy.radius_km && point.y <= canopy.position.y {
                            return true;
                        }
                    }
                }
            }
        }
        false
    }
}

/// Density multipliers the area-selection slider offers, relative to
/// `DEFAULT_SPACING_M`. The ceiling is where a 20 km area stops fitting
/// comfortably in GPU memory on an integrated card.
pub const MIN_DENSITY: f32 = 0.25;
pub const MAX_DENSITY: f32 = 2.0;
pub const DENSITY_STEP: f32 = 0.25;

/// Chosen before generation on the area-selection screen. Trees and contour
/// lines are mutually exclusive: a forest hides the ground the contours
/// describe, so the two would only fight for the same pixels.
#[derive(Resource, Clone, Copy, Debug)]
pub struct VegetationSettings {
    pub enabled: bool,
    /// Trees per unit area, relative to the default scatter.
    pub density: f32,
}

impl Default for VegetationSettings {
    fn default() -> Self {
        Self {
            enabled: vegetation_enabled(),
            density: 1.0,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct ScatterConfig {
    pub spacing_m: f32,
    pub height_m: f32,
    pub area_size_km: f32,
}

impl ScatterConfig {
    pub fn for_settings(settings: &VegetationSettings, area_size_km: f32) -> Self {
        // Density counts trees per unit area, so spacing goes as its inverse
        // square root: twice the density gives each tree half the ground.
        let base = env_f32("TREE_SPACING_M", DEFAULT_SPACING_M, 5.0, 500.0);
        let density = settings.density.clamp(MIN_DENSITY, MAX_DENSITY);
        Self {
            spacing_m: (base / density.sqrt()).clamp(5.0, 500.0),
            height_m: env_f32("TREE_HEIGHT_M", DEFAULT_TREE_HEIGHT_M, 1.0, 150.0),
            area_size_km,
        }
    }
}

/// Deterministic hash so a given area always grows the same forest — otherwise
/// trees would jump on every reload of the same scenario.
fn hash_to_unit(x: u32, y: u32, salt: u32) -> f32 {
    let mut h = x
        .wrapping_mul(0x9E37_79B9)
        .wrapping_add(y.wrapping_mul(0x85EB_CA6B))
        .wrapping_add(salt.wrapping_mul(0xC2B2_AE35));
    h ^= h >> 15;
    h = h.wrapping_mul(0x2545_F491);
    h ^= h >> 13;
    // Top 24 bits into [0, 1).
    (h >> 8) as f32 / (1u32 << 24) as f32
}

/// Lay trees on a jittered grid across the whole area.
pub fn scatter(config: ScatterConfig) -> Vec<TreeInstance> {
    let extent_m = config.area_size_km * 1_000.0;
    let half_m = extent_m * 0.5;
    let steps = (extent_m / config.spacing_m).floor().max(1.0) as u32;
    let jitter = config.spacing_m * JITTER_FRACTION;
    let crown_radius_m = config.height_m * CROWN_RADIUS_FRACTION;

    let mut trees = Vec::with_capacity((steps as usize).saturating_mul(steps as usize));
    for row in 0..steps {
        for column in 0..steps {
            let base_x = (column as f32 + 0.5) * config.spacing_m - half_m;
            let base_z = (row as f32 + 0.5) * config.spacing_m - half_m;
            let dx = (hash_to_unit(column, row, 1) - 0.5) * 2.0 * jitter;
            let dz = (hash_to_unit(column, row, 2) - 0.5) * 2.0 * jitter;
            trees.push(TreeInstance {
                x_km: (base_x + dx) / 1_000.0,
                z_km: (base_z + dz) / 1_000.0,
                height_m: config.height_m,
                crown_radius_m,
            });
        }
    }
    trees
}

/// Accumulates many trees into one mesh.
#[derive(Default)]
struct MeshBuilder {
    positions: Vec<[f32; 3]>,
    normals: Vec<[f32; 3]>,
}

impl MeshBuilder {
    fn push_triangle(&mut self, a: Vec3, b: Vec3, c: Vec3) {
        let normal = (b - a).cross(c - a).normalize_or_zero();
        for vertex in [a, b, c] {
            self.positions.push(vertex.to_array());
            self.normals.push(normal.to_array());
        }
    }

    /// Append one tree, in kilometres, with its base at `base`.
    fn push_tree(&mut self, base: Vec3, height_km: f32, crown_radius_km: f32, yaw: f32) {
        let trunk_height = height_km * TRUNK_HEIGHT_FRACTION;
        let trunk_radius = (height_km * TRUNK_RADIUS_FRACTION).max(0.000_2);
        let ring = |sides: usize, side: usize, radius: f32, y: f32| {
            let angle = side as f32 / sides as f32 * std::f32::consts::TAU + yaw;
            base + Vec3::new(angle.cos() * radius, y, angle.sin() * radius)
        };

        // Trunk: a low-sided prism from the ground to the base of the crown.
        for side in 0..TRUNK_SIDES {
            let p0 = ring(TRUNK_SIDES, side, trunk_radius, 0.0);
            let p1 = ring(TRUNK_SIDES, side + 1, trunk_radius, 0.0);
            let top = Vec3::Y * trunk_height;
            self.push_triangle(p0, p1, p1 + top);
            self.push_triangle(p0, p1 + top, p0 + top);
        }

        // Crown: a cone from the top of the trunk to the tip.
        let apex = base + Vec3::Y * height_km;
        for side in 0..CROWN_SIDES {
            let p0 = ring(CROWN_SIDES, side, crown_radius_km, trunk_height);
            let p1 = ring(CROWN_SIDES, side + 1, crown_radius_km, trunk_height);
            self.push_triangle(p0, p1, apex);
        }
    }

    fn is_empty(&self) -> bool {
        self.positions.is_empty()
    }

    fn build(self) -> Mesh {
        let indices: Vec<u32> = (0..self.positions.len() as u32).collect();
        let mut mesh = Mesh::new(
            bevy::render::mesh::PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        );
        mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, self.positions);
        mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, self.normals);
        mesh.insert_indices(Indices::U32(indices));
        mesh
    }
}

/// Trees only belong on land. Water in this terrain is whatever sits on the
/// normalised floor and is locally flat — real ground that low still undulates.
fn is_land(height_map: &TerrainHeightMap, x_km: f32, z_km: f32) -> bool {
    let ground_m = height_map.height_at(x_km, z_km) * 1_000.0;
    if ground_m > WATER_MAX_HEIGHT_M {
        return true;
    }
    let d = RELIEF_SAMPLE_KM;
    let samples = [
        height_map.height_at(x_km + d, z_km),
        height_map.height_at(x_km - d, z_km),
        height_map.height_at(x_km, z_km + d),
        height_map.height_at(x_km, z_km - d),
    ];
    let highest = samples.iter().copied().fold(f32::NEG_INFINITY, f32::max) * 1_000.0;
    let lowest = samples.iter().copied().fold(f32::INFINITY, f32::min) * 1_000.0;
    (highest - lowest) > WATER_MAX_RELIEF_M
}

/// Grow and spawn the forest. Runs once on entering the simulation.
pub fn spawn_trees(
    mut commands: Commands,
    height_map: Res<TerrainHeightMap>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    settings: Res<VegetationSettings>,
) {
    if !settings.enabled {
        return;
    }
    let config = ScatterConfig::for_settings(&settings, height_map.size_km);
    let candidates = scatter(config);
    if candidates.is_empty() {
        return;
    }

    let half_km = config.area_size_km * 0.5;
    let chunks_per_side = (config.area_size_km / CHUNK_KM).ceil().max(1.0) as usize;
    let mut builders: Vec<MeshBuilder> = (0..chunks_per_side * chunks_per_side)
        .map(|_| MeshBuilder::default())
        .collect();

    let mut planted = 0usize;
    let mut radio_canopies = RadioCanopies::default();
    for tree in candidates {
        if !is_land(&height_map, tree.x_km, tree.z_km) {
            continue;
        }
        let chunk_x = (((tree.x_km + half_km) / CHUNK_KM) as usize).min(chunks_per_side - 1);
        let chunk_z = (((tree.z_km + half_km) / CHUNK_KM) as usize).min(chunks_per_side - 1);
        let origin = Vec3::new(
            chunk_x as f32 * CHUNK_KM - half_km,
            0.0,
            chunk_z as f32 * CHUNK_KM - half_km,
        );

        // Vary yaw and size so a shared silhouette does not read as clones.
        let key = (tree.x_km.to_bits(), tree.z_km.to_bits());
        let yaw = hash_to_unit(key.0, key.1, 3) * std::f32::consts::TAU;
        let scale = 0.8 + hash_to_unit(key.0, key.1, 4) * 0.45;
        let ground = height_map.height_at(tree.x_km, tree.z_km);
        let canopy = RadioCanopy {
            position: Vec3::new(tree.x_km, ground + tree.height_m / 1_000.0 * scale, tree.z_km),
            radius_km: tree.crown_radius_m / 1_000.0 * scale,
        };
        radio_canopies
            .0
            .entry(RadioCanopies::cell(canopy.position))
            .or_default()
            .push(canopy);
        builders[chunk_z * chunks_per_side + chunk_x].push_tree(
            Vec3::new(tree.x_km, ground, tree.z_km) - origin,
            tree.height_m / 1_000.0 * scale,
            tree.crown_radius_m / 1_000.0 * scale,
            yaw,
        );
        planted += 1;
    }

    let material = materials.add(StandardMaterial {
        base_color: Color::srgb_u8(0x2f, 0x6f, 0x3e),
        perceptual_roughness: 0.95,
        ..default()
    });
    // No `VisibilityRange`: the whole 20 km area is on screen when you zoom out,
    // and a distance cut-off puts a visible circular edge across the forest.
    // Frustum culling per chunk is the only culling these meshes get.
    let mut chunk_count = 0;
    for (index, builder) in builders.into_iter().enumerate() {
        if builder.is_empty() {
            continue;
        }
        let (chunk_x, chunk_z) = (index % chunks_per_side, index / chunks_per_side);
        let origin = Vec3::new(
            chunk_x as f32 * CHUNK_KM - half_km,
            0.0,
            chunk_z as f32 * CHUNK_KM - half_km,
        );
        commands.spawn((
            Mesh3d(meshes.add(builder.build())),
            MeshMaterial3d(material.clone()),
            Transform::from_translation(origin),
            TerrainTree,
        ));
        chunk_count += 1;
    }

    info!(
        "spawned {} procedural trees ({:.0} m tall, {:.0} m spacing) in {chunk_count} chunks",
        planted,
        config.height_m,
        config.spacing_m
    );
    commands.insert_resource(radio_canopies);
}

pub fn cleanup_trees(mut commands: Commands, trees: Query<Entity, With<TerrainTree>>) {
    for entity in &trees {
        commands.entity(entity).despawn();
    }
    commands.remove_resource::<RadioCanopies>();
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
