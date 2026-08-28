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

const DEFAULT_VERTICAL_EXAGGERATION: f32 = 5.0;
const CONTOUR_INTERVAL_KM: f32 = 0.020;
const CONTOUR_SURFACE_OFFSET: f32 = 0.008;

fn parse_vertical_exaggeration(value: Option<&str>) -> f32 {
    value
        .and_then(|value| value.parse::<f32>().ok())
        .filter(|value| value.is_finite())
        .unwrap_or(DEFAULT_VERTICAL_EXAGGERATION)
        .clamp(1.0, 20.0)
}

#[derive(Resource)]
pub struct LocalHeightServer(std::process::Child);

impl Drop for LocalHeightServer {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

/// Start the bundled Python service. Set `HEIGHT_SERVER_URL` to use an
/// externally managed service instead, or `KOLBE_PYTHON` to select Python.
pub fn start_local_server(mut commands: Commands) {
    if std::env::var_os("HEIGHT_SERVER_URL").is_some() {
        return;
    }

    let server_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("height_server");
    let mut candidates = Vec::new();
    if let Some(python) = std::env::var_os("KOLBE_PYTHON") {
        candidates.push(std::path::PathBuf::from(python));
    }
    #[cfg(windows)]
    candidates.push(server_dir.join(".venv").join("Scripts").join("python.exe"));
    #[cfg(not(windows))]
    candidates.push(server_dir.join(".venv").join("bin").join("python"));
    candidates.push(std::path::PathBuf::from("python3"));
    candidates.push(std::path::PathBuf::from("python"));

    for python in candidates {
        match std::process::Command::new(&python)
            .arg("height.py")
            .current_dir(&server_dir)
            .env("RADIUS_KM", "10")
            .env("OUTPUT_SIZE", "129")
            .spawn()
        {
            Ok(child) => {
                info!("Started height server with {}", python.display());
                commands.insert_resource(LocalHeightServer(child));
                return;
            }
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => continue,
            Err(error) => {
                warn!(
                    "Could not start height server with {}: {error}",
                    python.display()
                );
            }
        }
    }

    error!("No Python executable found. Create height_server/.venv or set KOLBE_PYTHON.");
}

#[derive(Resource)]
pub struct TerrainHeightMap {
    heights_km: Vec<f32>,
    resolution: usize,
    size_km: f32,
    vertical_exaggeration: f32,
}

impl TerrainHeightMap {
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

#[derive(Resource)]
pub struct HeightLoadTask(Task<Result<TerrainHeightMap, String>>);

#[derive(Resource)]
pub struct ProgressPollTask(Task<Option<String>>);

#[derive(Resource)]
pub struct TerrainLoadError(pub String);

#[derive(Component)]
pub(crate) struct LoadingRoot;

#[derive(Component)]
pub(crate) struct LoadingStatus;

#[derive(Component)]
pub(crate) struct LoadingBarFill;

pub fn start_loading(mut commands: Commands, area: Res<ScenarioArea>) {
    let request_area = area.clone();
    let server =
        std::env::var("HEIGHT_SERVER_URL").unwrap_or_else(|_| "http://127.0.0.1:8000".to_owned());
    let task = IoTaskPool::get().spawn(async move { fetch_height_map(&server, &request_area) });
    commands.insert_resource(HeightLoadTask(task));

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
            BackgroundColor(Color::srgb(0.07, 0.09, 0.13)),
            LoadingRoot,
        ))
        .with_children(|root| {
            root.spawn((
                Text::new(format!("Loading terrain around {}", area.name)),
                TextFont {
                    font_size: FontSize::Px(30.0),
                    ..default()
                },
                TextColor(Color::WHITE),
            ));
            root.spawn((
                Text::new("Querying Lantmateriet and building the height map..."),
                TextFont {
                    font_size: FontSize::Px(17.0),
                    ..default()
                },
                TextColor(Color::srgb(0.55, 0.75, 0.9)),
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
                BackgroundColor(Color::srgb(0.15, 0.18, 0.23)),
            ))
            .with_child((
                Node {
                    width: Val::Percent(2.0),
                    height: Val::Percent(100.0),
                    border_radius: BorderRadius::all(Val::Px(7.0)),
                    ..default()
                },
                BackgroundColor(Color::srgb(0.2, 0.65, 0.95)),
                LoadingBarFill,
            ));
        });
}

pub fn update_progress(
    mut commands: Commands,
    time: Res<Time>,
    mut elapsed: Local<f32>,
    poll_task: Option<ResMut<ProgressPollTask>>,
    mut status: Query<&mut Text, With<LoadingStatus>>,
    mut bars: Query<&mut Node, With<LoadingBarFill>>,
) {
    if let Some(mut poll_task) = poll_task {
        let Some(body) = future::block_on(future::poll_once(&mut poll_task.0)) else {
            return;
        };
        commands.remove_resource::<ProgressPollTask>();
        if let Some(body) = body {
            apply_progress(&body, &mut status, &mut bars);
        }
        return;
    }

    *elapsed += time.delta_secs();
    if *elapsed < 0.25 {
        return;
    }
    *elapsed = 0.0;

    let server =
        std::env::var("HEIGHT_SERVER_URL").unwrap_or_else(|_| "http://127.0.0.1:8000".to_owned());
    let task = IoTaskPool::get().spawn(async move {
        reqwest::blocking::Client::builder()
            .timeout(std::time::Duration::from_millis(500))
            .build()
            .and_then(|client| {
                client
                    .get(format!("{}/progress", server.trim_end_matches('/')))
                    .send()
            })
            .and_then(|response| response.error_for_status())
            .and_then(|response| response.text())
            .ok()
    });
    commands.insert_resource(ProgressPollTask(task));
}

fn apply_progress(
    body: &str,
    status: &mut Query<&mut Text, With<LoadingStatus>>,
    bars: &mut Query<&mut Node, With<LoadingBarFill>>,
) {
    let value = |key: &str| {
        body.lines()
            .find_map(|line| line.strip_prefix(&format!("{key}=")))
            .unwrap_or("")
    };
    let phase = value("phase");
    let done = value("done").parse::<usize>().unwrap_or(0);
    let total = value("total").parse::<usize>().unwrap_or(0);
    let current = value("current");
    let percent = if total > 0 {
        done as f32 / total as f32 * 100.0
    } else {
        2.0
    };

    if let Ok(mut bar) = bars.single_mut() {
        bar.width = Val::Percent(percent.clamp(2.0, 100.0));
    }
    if let Ok(mut text) = status.single_mut() {
        let count = if total > 0 {
            format!(" ({done}/{total})")
        } else {
            String::new()
        };
        let file = if current.is_empty() {
            String::new()
        } else {
            format!("\n{current}")
        };
        **text = format!("{phase}{count}{file}");
    }
}

pub fn poll_loading(
    mut commands: Commands,
    mut task: ResMut<HeightLoadTask>,
    mut status: Query<&mut Text, With<LoadingStatus>>,
    mut next_state: ResMut<NextState<AppState>>,
) {
    let Some(result) = future::block_on(future::poll_once(&mut task.0)) else {
        return;
    };
    match result {
        Ok(height_map) => {
            commands.remove_resource::<HeightLoadTask>();
            commands.insert_resource(height_map);
            next_state.set(AppState::Simulation);
        }
        Err(message) => {
            if let Ok(mut text) = status.single_mut() {
                **text = format!(
                    "Could not load terrain:\n{message}\n\nStart height_server/height.py and restart Kolbe to try again."
                );
            }
            commands.remove_resource::<HeightLoadTask>();
            commands.insert_resource(TerrainLoadError(message));
            next_state.set(AppState::AreaSelection);
        }
    }
}

pub fn cleanup_loading(mut commands: Commands, roots: Query<Entity, With<LoadingRoot>>) {
    commands.remove_resource::<ProgressPollTask>();
    for entity in &roots {
        commands.entity(entity).despawn();
    }
}

pub fn spawn_mesh(
    mut commands: Commands,
    height_map: Res<TerrainHeightMap>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands
        .spawn((
            Mesh3d(meshes.add(mesh_from_height_map(&height_map))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.18, 0.38, 0.20),
                perceptual_roughness: 1.0,
                ..default()
            })),
            Transform::IDENTITY,
            ThemeRole::Ground,
        ))
        .observe(
            |_: On<Pointer<Click>>, orbit: Res<OrbitCamera>, mut selected: ResMut<SelectedDrone>| {
                if orbit.drag_total < 5.0 {
                    selected.0 = None;
                }
            },
        );
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

fn fetch_height_map(server: &str, area: &ScenarioArea) -> Result<TerrainHeightMap, String> {
    let url = format!(
        "{}/fetch?lat={}&lon={}",
        server.trim_end_matches('/'),
        area.latitude,
        area.longitude
    );
    let client = reqwest::blocking::Client::builder()
        .timeout(std::time::Duration::from_secs(300))
        .build()
        .map_err(|error| error.to_string())?;
    let mut attempts = 0;
    let response = loop {
        match client.get(&url).send() {
            Ok(response) => break response,
            Err(error) => {
                attempts += 1;
                if error.is_connect() && attempts < 40 {
                    std::thread::sleep(std::time::Duration::from_millis(250));
                } else {
                    return Err(format!("Height server is unavailable: {error}"));
                }
            }
        }
    };
    if !response.status().is_success() {
        let status = response.status();
        let message = response.text().unwrap_or_default();
        return Err(format!("Height server returned {status}: {message}"));
    }
    let width = header_usize(&response, "X-Width")?;
    let height = header_usize(&response, "X-Height")?;
    if width != height || width < 2 {
        return Err(format!(
            "Expected a square height grid, received {width}x{height}"
        ));
    }
    let bytes = response.bytes().map_err(|error| error.to_string())?;
    if bytes.len() != width * height * 2 {
        return Err(format!(
            "Expected {} height bytes, received {}",
            width * height * 2,
            bytes.len()
        ));
    }
    let mut heights_km = Vec::with_capacity(width * height);
    // Raster row zero is north. Flip rows so positive Bevy Z points north.
    for z in 0..height {
        let source_row = height - 1 - z;
        for x in 0..width {
            let index = (source_row * width + x) * 2;
            let bits = u16::from_le_bytes([bytes[index], bytes[index + 1]]);
            heights_km.push(f16_to_f32(bits) / 1000.0);
        }
    }
    Ok(TerrainHeightMap {
        heights_km,
        resolution: width,
        size_km: area.size_km,
        vertical_exaggeration: parse_vertical_exaggeration(
            std::env::var("TERRAIN_VERTICAL_EXAGGERATION")
                .ok()
                .as_deref(),
        ),
    })
}

fn header_usize(response: &reqwest::blocking::Response, name: &str) -> Result<usize, String> {
    response
        .headers()
        .get(name)
        .ok_or_else(|| format!("Height server omitted {name}"))?
        .to_str()
        .map_err(|_| format!("Height server returned invalid {name}"))?
        .parse()
        .map_err(|_| format!("Height server returned invalid {name}"))
}

fn f16_to_f32(bits: u16) -> f32 {
    let sign = ((bits & 0x8000) as u32) << 16;
    let exponent = (bits >> 10) & 0x1f;
    let fraction = bits & 0x03ff;
    let value = match exponent {
        0 if fraction == 0 => sign,
        0 => {
            let mut fraction = fraction as u32;
            let mut shift = 0u32;
            while fraction & 0x0400 == 0 {
                fraction <<= 1;
                shift += 1;
            }
            sign | ((113 - shift) << 23) | ((fraction & 0x03ff) << 13)
        }
        31 => sign | 0x7f80_0000 | ((fraction as u32) << 13),
        _ => sign | (((exponent as u32) + 112) << 23) | ((fraction as u32) << 13),
    };
    f32::from_bits(value)
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
    fn decodes_common_float16_values() {
        assert_eq!(f16_to_f32(0x0000), 0.0);
        assert_eq!(f16_to_f32(0x3c00), 1.0);
        assert_eq!(f16_to_f32(0xc000), -2.0);
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
        assert_eq!(parse_vertical_exaggeration(None), 5.0);
        assert_eq!(parse_vertical_exaggeration(Some("8")), 8.0);
        assert_eq!(parse_vertical_exaggeration(Some("0")), 1.0);
        assert_eq!(parse_vertical_exaggeration(Some("50")), 20.0);
        assert_eq!(parse_vertical_exaggeration(Some("invalid")), 5.0);
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
