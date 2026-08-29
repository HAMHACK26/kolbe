use std::sync::{Arc, Mutex};

use bevy::{
    asset::RenderAssetUsages,
    mesh::{Indices, PrimitiveTopology},
    picking::prelude::*,
    prelude::*,
    tasks::{IoTaskPool, Task, futures_lite::future},
};

use crate::{
    AppState,
    area::ScenarioArea,
    camera::OrbitCamera,
    drone::SelectedDrone,
    theme::ThemeRole,
};

mod source;
mod vegetation;

use source::{Progress, ProgressHandle};

const DEFAULT_VERTICAL_EXAGGERATION: f32 = 1.0;
const CONTOUR_INTERVAL_KM: f32 = 0.005;
const CONTOUR_SURFACE_OFFSET: f32 = 0.008;

fn parse_vertical_exaggeration(value: Option<&str>) -> f32 {
    value
        .and_then(|value| value.parse::<f32>().ok())
        .filter(|value| value.is_finite())
        .unwrap_or(DEFAULT_VERTICAL_EXAGGERATION)
        .clamp(1.0, 20.0)
}

/// Shared, live progress for the in-process terrain fetch. The fetch task
/// updates it; [`update_progress`] reads it to drive the loading bar.
#[derive(Resource, Clone)]
pub struct TerrainProgress(ProgressHandle);

#[derive(Resource)]
pub struct TerrainHeightMap {
    heights_km: Vec<f32>,
    resolution: usize,
    size_km: f32,
    vertical_exaggeration: f32,
}

impl TerrainHeightMap {
    /// Side length (km) of the fetched terrain — the actual requested
    /// network-area size, which can differ a lot from the fixed
    /// `world::WORLD_SIZE` (20km) the hand-placed drone ring and grid
    /// overlay are scaled from.
    pub fn size_km(&self) -> f32 {
        self.size_km
    }

    pub fn height_at(&self, x_km: f32, z_km: f32) -> f32 {
        let half = self.size_km * 0.5;
        let u = ((x_km + half) / self.size_km).clamp(0.0, 1.0);
        let v = ((z_km + half) / self.size_km).clamp(0.0, 1.0);
        let grid_x = u * (self.resolution - 1) as f32;
        let grid_z = v * (self.resolution - 1) as f32;
        let x0 = grid_x.floor() as usize;
        let z0 = grid_z.floor() as usize;
        let x1 = (x0 + 1).min(self.resolution - 1);
        let z1 = (z0 + 1).min(self.resolution - 1);
        let tx = grid_x - x0 as f32;
        let tz = grid_z - z0 as f32;
        let top = self.heights_km[z0 * self.resolution + x0]
            .lerp(self.heights_km[z0 * self.resolution + x1], tx);
        let bottom = self.heights_km[z1 * self.resolution + x0]
            .lerp(self.heights_km[z1 * self.resolution + x1], tx);
        top.lerp(bottom, tz) * self.vertical_exaggeration
    }
}

struct TerrainData {
    height_map: TerrainHeightMap,
}

#[derive(Resource)]
pub struct TerrainLoadTask(Task<Result<TerrainData, String>>);

#[derive(Resource)]
pub struct TerrainLoadError(pub String);

#[derive(Component)]
pub(crate) struct LoadingRoot;

#[derive(Component)]
pub(crate) struct LoadingHeading;

#[derive(Component)]
pub(crate) struct LoadingStatus;

#[derive(Component)]
pub(crate) struct LoadingTrack;

#[derive(Component)]
pub(crate) struct LoadingBarFill;

pub fn start_loading(
    mut commands: Commands,
    area: Res<ScenarioArea>,
    theme: Res<crate::theme::Theme>,
) {
    let p = theme.palette();
    let request_area = area.clone();
    let progress: ProgressHandle = Arc::new(Mutex::new(Progress::default()));
    let task_progress = progress.clone();
    let task = IoTaskPool::get()
        .spawn(async move { fetch_terrain_data(&request_area, &task_progress) });
    commands.insert_resource(TerrainLoadTask(task));
    commands.insert_resource(TerrainProgress(progress));

    commands
        .spawn((
            Node {
                width: Val::Percent(100.0),
                height: Val::Percent(100.0),
                justify_content: JustifyContent::Center,
                align_items: AlignItems::Center,
                flex_direction: FlexDirection::Column,
                row_gap: Val::Px(16.0),
                ..default()
            },
            BackgroundColor(p.bg),
            LoadingRoot,
        ))
        .with_children(|root| {
            root.spawn((
                Text::new(format!("Loading terrain around {}", area.name)),
                TextFont {
                    font_size: FontSize::Px(30.0),
                    ..default()
                },
                TextColor(p.text),
                LoadingHeading,
            ));
            root.spawn((
                Text::new("Querying Lantmateriet and building the height map..."),
                TextFont {
                    font_size: FontSize::Px(17.0),
                    ..default()
                },
                TextColor(p.subtext),
                LoadingStatus,
            ));
            root.spawn((
                Node {
                    width: Val::Px(420.0),
                    height: Val::Px(18.0),
                    border_radius: BorderRadius::all(Val::Px(9.0)),
                    padding: UiRect::all(Val::Px(2.0)),
                    ..default()
                },
                BackgroundColor(p.surface),
                LoadingTrack,
            ))
            .with_child((
                Node {
                    width: Val::Percent(2.0),
                    height: Val::Percent(100.0),
                    border_radius: BorderRadius::all(Val::Px(7.0)),
                    ..default()
                },
                BackgroundColor(p.accent),
                LoadingBarFill,
            ));
        });
}

pub fn update_progress(
    progress: Option<Res<TerrainProgress>>,
    mut status: Query<&mut Text, With<LoadingStatus>>,
    mut bars: Query<&mut Node, With<LoadingBarFill>>,
) {
    let Some(progress) = progress else {
        return;
    };
    let Some(snapshot) = progress.0.lock().ok().map(|p| p.clone()) else {
        return;
    };

    let percent = snapshot.overall_fraction() * 100.0;
    if let Ok(mut bar) = bars.single_mut() {
        bar.width = Val::Percent(percent.clamp(2.0, 100.0));
    }
    if let Ok(mut text) = status.single_mut() {
        let count = if snapshot.total > 0 {
            format!(" ({}/{})", snapshot.done, snapshot.total)
        } else {
            String::new()
        };
        let file = if snapshot.current.is_empty() {
            String::new()
        } else {
            format!("\n{}", snapshot.current)
        };
        **text = format!("{}{count}{file}", snapshot.phase);
    }
}

pub fn poll_loading(
    mut commands: Commands,
    mut task: ResMut<TerrainLoadTask>,
    mut status: Query<&mut Text, With<LoadingStatus>>,
    mut next_state: ResMut<NextState<AppState>>,
) {
    let Some(result) = future::block_on(future::poll_once(&mut task.0)) else {
        return;
    };
    match result {
        Ok(data) => {
            commands.remove_resource::<TerrainLoadTask>();
            commands.insert_resource(data.height_map);
            next_state.set(AppState::Simulation);
        }
        Err(message) => {
            if let Ok(mut text) = status.single_mut() {
                **text = format!(
                    "Could not load terrain:\n{message}\n\nCheck your .env credentials and network, then restart Kolbe."
                );
            }
            commands.remove_resource::<TerrainLoadTask>();
            commands.insert_resource(TerrainLoadError(message));
            next_state.set(AppState::AreaSelection);
        }
    }
}

pub fn cleanup_loading(mut commands: Commands, roots: Query<Entity, With<LoadingRoot>>) {
    commands.remove_resource::<TerrainProgress>();
    for entity in &roots {
        commands.entity(entity).despawn();
    }
}

pub fn spawn_mesh(
    mut commands: Commands,
    height_map: Res<TerrainHeightMap>,
    theme: Res<crate::theme::Theme>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Placeholder color from the palette; `apply_theme` re-syncs via ThemeRole.
    commands
        .spawn((
            Mesh3d(meshes.add(mesh_from_height_map(&height_map))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: theme.palette().ground,
                perceptual_roughness: 1.0,
                ..default()
            })),
            Transform::IDENTITY,
            ThemeRole::Ground,
            crate::SimulationEntity,
        ))
        .observe(
            |_: On<Pointer<Click>>, orbit: Res<OrbitCamera>, mut selected: ResMut<SelectedDrone>| {
                if orbit.drag_total < 5.0 {
                    selected.0 = None;
                }
            },
        );
}

pub use vegetation::spawn_trees;
pub use vegetation::cleanup_trees;
pub use vegetation::{DENSITY_STEP, MAX_DENSITY, MIN_DENSITY, VegetationSettings};

/// A thin sea-level reference plane at the data's zero elevation (the lowest
/// point in the fetched area) — flat terrain near that level is otherwise
/// indistinguishable from flat terrain sitting on a raised plateau.
const WATER_SURFACE_OFFSET: f32 = 0.004;

pub fn spawn_water(
    mut commands: Commands,
    height_map: Res<TerrainHeightMap>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let half = height_map.size_km * 0.5;
    let mut mesh = Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());
    mesh.insert_attribute(
        Mesh::ATTRIBUTE_POSITION,
        vec![
            [-half, 0.0, -half],
            [half, 0.0, -half],
            [half, 0.0, half],
            [-half, 0.0, half],
        ],
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, vec![[0.0, 1.0, 0.0]; 4]);
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
    mesh.insert_indices(Indices::U32(vec![0, 2, 1, 0, 3, 2]));

    commands.spawn((
        Mesh3d(meshes.add(mesh)),
        MeshMaterial3d(materials.add(StandardMaterial {
            perceptual_roughness: 0.05,
            alpha_mode: AlphaMode::Blend,
            double_sided: true,
            cull_mode: None,
            ..default()
        })),
        Transform::from_xyz(0.0, WATER_SURFACE_OFFSET, 0.0),
        ThemeRole::Water,
        Pickable::IGNORE,
        crate::SimulationEntity,
    ));
}

/// Marks the picked polygon, its (possibly rotated) minimum bounding square
/// — the network area — and the larger axis-aligned box actually fetched.
/// Lines are subdivided and dropped onto the height field so they follow
/// the terrain instead of floating as flat chords over sloped ground.
pub fn draw_network_area(
    mut gizmos: Gizmos,
    net: Option<Res<crate::area::NetworkArea>>,
    area: Res<crate::area::ScenarioArea>,
    terrain: Res<TerrainHeightMap>,
) {
    let Some(net) = net else { return };
    if !net.valid {
        return;
    }

    let to_local = |lat: f64, lon: f64| -> (f32, f32) {
        let x = ((lon - area.longitude) * 111.320 * area.latitude.to_radians().cos()) as f32;
        let z = ((lat - area.latitude) * 110.574) as f32;
        (x, z)
    };

    let fetch_color = Color::srgba(1.0, 1.0, 1.0, 0.25);
    let square_color = Color::srgb(1.0, 0.85, 0.2);
    let polygon_color = Color::srgb(0.3, 0.9, 1.0);

    draw_ring(&mut gizmos, &terrain, net.fetch_corners.map(|(lat, lon)| to_local(lat, lon)), fetch_color);
    draw_ring(&mut gizmos, &terrain, net.corners.map(|(lat, lon)| to_local(lat, lon)), square_color);
    let points: Vec<(f32, f32)> = net.points.iter().map(|&(lat, lon)| to_local(lat, lon)).collect();
    draw_ring(&mut gizmos, &terrain, points, polygon_color);
}

fn draw_ring(
    gizmos: &mut Gizmos,
    terrain: &TerrainHeightMap,
    points: impl IntoIterator<Item = (f32, f32)>,
    color: Color,
) {
    const SEGMENTS: usize = 12;
    const SURFACE_OFFSET: f32 = 0.02;

    let points: Vec<(f32, f32)> = points.into_iter().collect();
    if points.len() < 2 {
        return;
    }
    let n = points.len();
    for i in 0..n {
        let (x0, z0) = points[i];
        let (x1, z1) = points[(i + 1) % n];
        for s in 0..SEGMENTS {
            let ta = s as f32 / SEGMENTS as f32;
            let tb = (s + 1) as f32 / SEGMENTS as f32;
            let (ax, az) = (x0.lerp(x1, ta), z0.lerp(z1, ta));
            let (bx, bz) = (x0.lerp(x1, tb), z0.lerp(z1, tb));
            gizmos.line(
                Vec3::new(ax, terrain.height_at(ax, az) + SURFACE_OFFSET, az),
                Vec3::new(bx, terrain.height_at(bx, bz) + SURFACE_OFFSET, bz),
                color,
            );
        }
    }
}

/// Draw 20 m contour lines from the local height grid. Heights in the grid are
/// relative to the lowest point in the selected area, which is sufficient for
/// unlabeled terrain-shape contours.
pub fn draw_contours(
    mut gizmos: Gizmos,
    height_map: Res<TerrainHeightMap>,
    theme: Res<crate::theme::Theme>,
) {
    let color = theme.palette().text.with_alpha(0.38);
    let step = height_map.size_km / (height_map.resolution - 1) as f32;
    let half = height_map.size_km * 0.5;

    for z in 0..height_map.resolution - 1 {
        for x in 0..height_map.resolution - 1 {
            let index = z * height_map.resolution + x;
            let values = [
                height_map.heights_km[index],
                height_map.heights_km[index + 1],
                height_map.heights_km[index + height_map.resolution + 1],
                height_map.heights_km[index + height_map.resolution],
            ];
            let minimum = values.iter().copied().fold(f32::INFINITY, f32::min);
            let maximum = values.iter().copied().fold(f32::NEG_INFINITY, f32::max);
            let first_level = ((minimum / CONTOUR_INTERVAL_KM).ceil() as i32).max(1);
            let last_level = (maximum / CONTOUR_INTERVAL_KM).floor() as i32;
            if first_level > last_level {
                continue;
            }

            let x0 = x as f32 * step - half;
            let z0 = z as f32 * step - half;
            let corners = [
                Vec3::new(x0, 0.0, z0),
                Vec3::new(x0 + step, 0.0, z0),
                Vec3::new(x0 + step, 0.0, z0 + step),
                Vec3::new(x0, 0.0, z0 + step),
            ];
            for level_index in first_level..=last_level {
                let level = level_index as f32 * CONTOUR_INTERVAL_KM;
                let crossings = contour_crossings(
                    corners,
                    values,
                    level,
                    height_map.vertical_exaggeration,
                );
                if crossings.len() >= 2 {
                    gizmos.line(crossings[0], crossings[1], color);
                }
                if crossings.len() == 4 {
                    gizmos.line(crossings[2], crossings[3], color);
                }
            }
        }
    }
}

fn contour_crossings(
    corners: [Vec3; 4],
    values: [f32; 4],
    level: f32,
    vertical_exaggeration: f32,
) -> Vec<Vec3> {
    let mut crossings = Vec::with_capacity(4);
    for (start, end) in [(0, 1), (1, 2), (2, 3), (3, 0)] {
        let a = values[start];
        let b = values[end];
        if (a < level && b >= level) || (b < level && a >= level) {
            let t = (level - a) / (b - a);
            let mut point = corners[start].lerp(corners[end], t);
            point.y = level * vertical_exaggeration + CONTOUR_SURFACE_OFFSET;
            crossings.push(point);
        }
    }
    crossings
}

/// Fetch terrain in-process and adapt it to the mesh/sampling grid. Runs on a
/// background task; reports live status through `progress`.
fn fetch_height_map(
    area: &ScenarioArea,
    progress: &ProgressHandle,
) -> Result<TerrainHeightMap, String> {
    let radius_km = area.size_km as f64 * 0.5;
    let grid = source::fetch_terrain(area.latitude, area.longitude, radius_km, progress)?;
    let size = grid.size;
    if size < 2 {
        return Err(format!("terrain grid too small: {size}x{size}"));
    }

    // The engine grid has row 0 at the north edge. Flip rows so positive Bevy Z
    // points north, and convert metres to kilometres.
    let mut heights_km = Vec::with_capacity(size * size);
    for z in 0..size {
        let source_row = size - 1 - z;
        for x in 0..size {
            heights_km.push(grid.heights_m[source_row * size + x] / 1000.0);
        }
    }

    Ok(TerrainHeightMap {
        heights_km,
        resolution: size,
        size_km: area.size_km,
        vertical_exaggeration: parse_vertical_exaggeration(
            std::env::var("TERRAIN_VERTICAL_EXAGGERATION").ok().as_deref(),
        ),
    })
}

fn fetch_terrain_data(area: &ScenarioArea, progress: &ProgressHandle) -> Result<TerrainData, String> {
    // Vegetation is generated procedurally at spawn time, so the only thing the
    // background load does is fetch elevation.
    let height_map = fetch_height_map(area, progress)?;
    Ok(TerrainData { height_map })
}

fn mesh_from_height_map(map: &TerrainHeightMap) -> Mesh {
    let count = map.resolution * map.resolution;
    let mut positions = Vec::with_capacity(count);
    let mut normals = Vec::with_capacity(count);
    let mut uvs = Vec::with_capacity(count);
    let half = map.size_km * 0.5;
    for z in 0..map.resolution {
        for x in 0..map.resolution {
            let u = x as f32 / (map.resolution - 1) as f32;
            let v = z as f32 / (map.resolution - 1) as f32;
            positions.push([
                u * map.size_km - half,
                map.heights_km[z * map.resolution + x] * map.vertical_exaggeration,
                v * map.size_km - half,
            ]);
            normals.push([0.0, 1.0, 0.0]);
            uvs.push([u, v]);
        }
    }
    let mut indices = Vec::with_capacity((map.resolution - 1).pow(2) * 6);
    for z in 0..map.resolution - 1 {
        for x in 0..map.resolution - 1 {
            let a = (z * map.resolution + x) as u32;
            let b = a + 1;
            let c = a + map.resolution as u32;
            let d = c + 1;
            indices.extend_from_slice(&[a, c, b, b, c, d]);
        }
    }
    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
    mesh.insert_indices(Indices::U32(indices));
    mesh.compute_smooth_normals();
    mesh
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_close(actual: f32, expected: f32) {
        assert!((actual - expected).abs() < 1e-6, "{actual} != {expected}");
    }

    #[test]
    fn samples_height_map_corners_with_vertical_exaggeration() {
        let map = TerrainHeightMap {
            heights_km: vec![0.01, 0.02, 0.03, 0.04],
            resolution: 2,
            size_km: 20.0,
            vertical_exaggeration: 5.0,
        };

        assert_close(map.height_at(-10.0, -10.0), 0.05);
        assert_close(map.height_at(10.0, -10.0), 0.10);
        assert_close(map.height_at(-10.0, 10.0), 0.15);
        assert_close(map.height_at(10.0, 10.0), 0.20);
    }

    #[test]
    fn clamps_height_samples_to_map_edges() {
        let map = TerrainHeightMap {
            heights_km: vec![1.0, 2.0, 3.0, 4.0],
            resolution: 2,
            size_km: 20.0,
            vertical_exaggeration: 1.0,
        };

        assert_eq!(map.height_at(-100.0, -100.0), 1.0);
        assert_eq!(map.height_at(100.0, 100.0), 4.0);
    }

    #[test]
    fn bilinearly_interpolates_between_height_samples() {
        let map = TerrainHeightMap {
            heights_km: vec![0.0, 2.0, 4.0, 6.0],
            resolution: 2,
            size_km: 20.0,
            vertical_exaggeration: 2.0,
        };

        assert_close(map.height_at(0.0, 0.0), 6.0);
    }

    #[test]
    fn validates_vertical_exaggeration_setting() {
        assert_eq!(parse_vertical_exaggeration(None), 1.0);
        assert_eq!(parse_vertical_exaggeration(Some("8")), 8.0);
        assert_eq!(parse_vertical_exaggeration(Some("0")), 1.0);
        assert_eq!(parse_vertical_exaggeration(Some("50")), 20.0);
        assert_eq!(parse_vertical_exaggeration(Some("invalid")), 1.0);
    }

    #[test]
    fn contour_crosses_a_sloped_cell_at_the_expected_position() {
        let corners = [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 1.0),
            Vec3::new(0.0, 0.0, 1.0),
        ];
        let crossings = contour_crossings(corners, [0.0, 0.04, 0.04, 0.0], 0.02, 5.0);

        assert_eq!(crossings.len(), 2);
        assert_close(crossings[0].x, 0.5);
        assert_close(crossings[1].x, 0.5);
        assert_close(crossings[0].y, 0.108);
        assert_close(crossings[1].y, 0.108);
    }
}
