use bevy::{picking::prelude::*, prelude::*};

use crate::{
    antenna::{Antenna, Antennas, angles_toward},
    base::{Base, CommandQueue},
    camera::OrbitCamera,
    drone::{Drone, DroneType, SelectedDrone, drone_id, make_antenna},
    factories::{DroneAi, movement::DroneKinematics},
    navigation::Orbit,
    networking::NetworkingBundle,
    radar::{RadarCone, cone_mesh_for, cone_transform_for},
    recovery::{ContactMemory, RecoveryState},
    seeking::SeekState,
    theme::{Theme, ThemeRole},
    ui::{
        InfoPopup, InfoPopupTable, InfoPopupTitle, NetworkTableButton, NetworkTablePanelText,
        NetworkTablePopup,
    },
};

pub const DRONE_COUNT: usize = 5;
pub const DRONE_RADIUS: f32 = 0.01125;

/// Hard cap on the distance between two ring neighbors, km.
///
/// The link budget in `make_antenna` closes at ~3.5 km on boresight
/// (`Antenna::max_range_km`), so holding neighbors at 3 km keeps every mesh
/// hop inside range with margin for terrain relief and drift.
pub const MAX_NEIGHBOR_SPACING_KM: f32 = 3.0;

/// Keep the formation ring this far inside the terrain edge, km, so no drone
/// spawns off the fetched height map.
const RING_TERRAIN_MARGIN_KM: f32 = 0.5;

/// The startup formation: `DRONE_COUNT` drones on a ring around the base,
/// spread as wide as the radio allows.
///
/// The ring sits at the largest radius whose neighbor spacing still respects
/// [`MAX_NEIGHBOR_SPACING_KM`]. A ring of N at radius R has a neighbor chord
/// of 2·R·sin(π/N), so the cap inverts to R = spacing / (2·sin(π/N)); that is
/// then clamped so the ring fits inside the fetched terrain, whose side length
/// is whatever area was picked on the map and can be far smaller than the ring
/// the link budget alone would allow.
///
/// It is centered on the base because the base is a radio node in the same
/// mesh: putting the ring around it holds every drone one radius from the
/// ground station, inside the same link budget that governs the drone-to-drone
/// hops. A base close to the terrain edge would push the ring off the map, so
/// the *center* slides inward to whatever keeps the full-width ring inside the
/// fetched area — the base then sits off-center in its own ring rather than
/// the formation collapsing.
///
/// `height_at` is the terrain sampler (`TerrainHeightMap::height_at`), taken as
/// a closure so the geometry can be exercised against synthetic terrain.
pub fn ring_formation(
    terrain_size_km: f32,
    base_pos: Vec3,
    height_at: impl Fn(f32, f32) -> f32,
) -> Vec<Vec3> {
    let n = DRONE_COUNT as f32;
    let chord_per_radius = 2.0 * (std::f32::consts::PI / n).sin();
    let terrain_limit = (terrain_size_km * 0.5 - RING_TERRAIN_MARGIN_KM).max(0.1);
    let mut radius = (MAX_NEIGHBOR_SPACING_KM / chord_per_radius).min(terrain_limit);

    let center_limit = (terrain_limit - radius).max(0.0);
    let center = Vec2::new(
        base_pos.x.clamp(-center_limit, center_limit),
        base_pos.z.clamp(-center_limit, center_limit),
    );

    let point_at = |radius: f32, angle: f32| {
        let (x, z) = (center.x + radius * angle.sin(), center.y + radius * angle.cos());
        Vec3::new(x, height_at(x, z) + DRONE_RADIUS, z)
    };
    let slot_angle = |i: usize| i as f32 / n * std::f32::consts::TAU;

    // The chord above is a *flat* distance, but the drones sit on the terrain,
    // so relief stretches the real 3-D neighbor distance past the cap. Shrink
    // until the true spacing is inside it — iteratively, because moving the
    // ring inward also changes which terrain it lands on.
    //
    // Checked at *every* rotation of the formation, not just the spawn
    // angles: the drones orbit the base (`navigation::orbit_base`), so a pair
    // of hills anywhere on the circle would otherwise break the link the
    // moment the ring turned onto them.
    const SAMPLES_PER_SLOT: usize = 12;
    let worst_over_the_orbit = |radius: f32| {
        let samples = DRONE_COUNT * SAMPLES_PER_SLOT;
        (0..samples)
            .map(|k| {
                let angle = k as f32 / samples as f32 * std::f32::consts::TAU;
                point_at(radius, angle).distance(point_at(radius, angle + slot_angle(1)))
            })
            .fold(0.0f32, f32::max)
    };
    for _ in 0..32 {
        let worst = worst_over_the_orbit(radius);
        if worst <= MAX_NEIGHBOR_SPACING_KM {
            break;
        }
        radius *= (MAX_NEIGHBOR_SPACING_KM / worst) * 0.999;
    }

    (0..DRONE_COUNT).map(|i| point_at(radius, slot_angle(i))).collect()
}

/// The three antennas drone `i` spawns with, already aimed at its mesh
/// targets: #1 at the next ring neighbor, #2 at the base, #3 at the previous
/// neighbor.
///
/// Aiming them here is what makes the mesh live on frame 0 rather than after a
/// spiral search stumbles onto a 1°-wide beam. It is the same slot layout
/// [`crate::tracking::maintain_mesh_antennas`] maintains once the drones move.
///
/// [`angles_toward`] gives a world-frame bearing while `Antenna::azimuth_deg`
/// is heading-relative; a drone spawns with `DroneKinematics::default()`
/// (heading 0), so at spawn the two frames coincide.
pub fn formation_antennas(ring: &[Vec3], i: usize, base_pos: Vec3) -> Vec<Antenna> {
    let self_pos = ring[i];
    let next = ring[(i + 1) % ring.len()];
    let prev = ring[(i + ring.len() - 1) % ring.len()];
    let (az_next, el_next) = angles_toward(self_pos, next);
    let (az_base, el_base) = angles_toward(self_pos, base_pos);
    let (az_prev, el_prev) = angles_toward(self_pos, prev);
    vec![
        make_antenna(az_next, el_next, i),
        make_antenna(az_base, el_base, i + 100),
        make_antenna(az_prev, el_prev, i + 200),
    ]
}

pub fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    terrain: Res<crate::terrain::TerrainHeightMap>,
    theme: Res<Theme>,
    bases: Query<&Base>,
) {
    let pal = theme.palette();
    // `AmbientLight` is a per-camera override of `GlobalAmbientLight` and so
    // `#[require(Camera)]`s one. Spawned on its own it lights nothing and leaves
    // a camera entity with no render graph, which Bevy warns about every run.
    commands.spawn((
        Camera3d::default(),
        Transform::default(),
        AmbientLight { brightness: 300.0, ..default() },
        crate::SimulationEntity,
    ));

    commands.spawn((
        DirectionalLight { illuminance: 8000.0, shadow_maps_enabled: false, ..default() },
        Transform::from_xyz(8.0, 16.0, 8.0).looking_at(Vec3::ZERO, Vec3::Y),
        crate::SimulationEntity,
    ));

    let drone_mesh = meshes.add(Sphere::new(DRONE_RADIUS));
    // Initial colors come from the palette; `apply_theme` keeps them in sync on
    // theme toggles (these entities carry ThemeRole markers).
    let drone_mat = materials.add(StandardMaterial {
        base_color: pal.drone,
        emissive: LinearRgba::new(2.0, 0.0, 0.0, 1.0),
        ..default()
    });
    // Same yellow as the base's cones — `apply_theme` keeps it in sync via
    // `ThemeRole::DroneCone`.
    let cone_mat = materials.add(StandardMaterial {
        base_color: pal.base.with_alpha(0.30),
        emissive: LinearRgba::new(0.6, 0.5, 0.0, 0.0),
        alpha_mode: AlphaMode::Blend,
        double_sided: true,
        cull_mode: None,
        ..default()
    });

    // The base spawns before this system (see `main`), so this is the real
    // base position — both the ring's center and what every drone's antenna #2
    // points at from frame one.
    let base_pos = bases.iter().next().map(|b| b.position).unwrap_or(Vec3::ZERO);
    let ring = ring_formation(terrain.size_km(), base_pos, |x, z| terrain.height_at(x, z));

    for i in 0..DRONE_COUNT {
        let drone_pos = ring[i];
        let drone_type = if i % 3 == 0 { DroneType::Attack } else { DroneType::Node };
        let antennas = formation_antennas(&ring, i, base_pos);

        let drone_entity = commands
            .spawn((
                Mesh3d(drone_mesh.clone()),
                MeshMaterial3d(drone_mat.clone()),
                Transform::from_translation(drone_pos),
                Drone { id: drone_id(i), drone_type },
                Antennas(antennas.clone()),
                DroneKinematics::default(),
                // Every drone orbits the base on the same circle it spawned
                // on, so the formation's angular spacing — and its links —
                // stay put all the way around.
                Orbit { radius_km: (drone_pos.xz() - base_pos.xz()).length() },
                DroneAi::default(),
                CommandQueue::default(),
                NetworkingBundle::random(i),
                SeekState::default(),
                RecoveryState::default(),
                ContactMemory::default(),
                ThemeRole::Drone,
                crate::SimulationEntity,
            ))
            .observe(
                |mut t: On<Pointer<Click>>,
                 orbit: Res<OrbitCamera>,
                 mut sel: ResMut<SelectedDrone>| {
                    t.propagate(false);
                    if orbit.drag_total < 5.0 {
                        sel.0 = Some(t.original_event_target());
                    }
                },
            )
            .id();

        for (antenna_index, antenna) in antennas.iter().enumerate() {
            commands.spawn((
                Mesh3d(cone_mesh_for(antenna, &mut meshes)),
                MeshMaterial3d(cone_mat.clone()),
                // heading 0.0 matches the DroneKinematics::default() this
                // drone spawns with; `radar::sync_radar_transforms` re-derives
                // this every frame as the drone moves and re-aims.
                cone_transform_for(antenna, 0.0, drone_pos),
                Visibility::Hidden,
                RadarCone { drone_entity, antenna_index },
                ThemeRole::DroneCone,
                crate::SimulationEntity,
            ));
        }
    }

    // Info popup
    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(0.0),
                top: Val::Px(0.0),
                padding: UiRect::axes(Val::Px(10.0), Val::Px(8.0)),
                border_radius: BorderRadius::all(Val::Px(4.0)),
                flex_direction: FlexDirection::Column,
                row_gap: Val::Px(2.0),
                ..default()
            },
            BackgroundColor(pal.surface.with_alpha(0.88)),
            Visibility::Hidden,
            InfoPopup,
            crate::SimulationEntity,
        ))
        .with_children(|p| {
            p.spawn((
                Text::new(""),
                TextFont { font_size: FontSize::Px(13.0), ..default() },
                TextColor(pal.text),
                InfoPopupTitle,
            ));
            p.spawn((
                Node {
                    display: Display::Grid,
                    grid_template_columns: vec![RepeatedGridTrack::auto(3)],
                    column_gap: Val::Px(10.0),
                    row_gap: Val::Px(2.0),
                    margin: UiRect::top(Val::Px(4.0)),
                    ..default()
                },
                InfoPopupTable,
            ));
            p.spawn((
                Button,
                Node {
                    margin: UiRect::top(Val::Px(6.0)),
                    padding: UiRect::axes(Val::Px(6.0), Val::Px(3.0)),
                    border_radius: BorderRadius::all(Val::Px(3.0)),
                    align_self: AlignSelf::FlexStart,
                    ..default()
                },
                BackgroundColor(pal.text.with_alpha(0.12)),
                NetworkTableButton,
            ))
            .observe(
                |mut t: On<Pointer<Click>>, mut open: ResMut<crate::ui::NetworkTablePanelOpen>| {
                    t.propagate(false);
                    open.0 = !open.0;
                },
            )
            .with_children(|b| {
                b.spawn((
                    Text::new("View Network Table"),
                    TextFont { font_size: FontSize::Px(11.0), ..default() },
                    TextColor(pal.text),
                ));
            });
        });

    // Network table window — a separate popup, same look as the info popup.
    // Its lifecycle is tied to the info popup (see `update_popup_position`):
    // closing the info popup closes this too.
    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(0.0),
                top: Val::Px(0.0),
                padding: UiRect::axes(Val::Px(10.0), Val::Px(8.0)),
                border_radius: BorderRadius::all(Val::Px(4.0)),
                flex_direction: FlexDirection::Column,
                row_gap: Val::Px(2.0),
                max_width: Val::Px(280.0),
                ..default()
            },
            BackgroundColor(pal.surface.with_alpha(0.88)),
            Visibility::Hidden,
            NetworkTablePopup,
            crate::SimulationEntity,
        ))
        .with_children(|p| {
            p.spawn((
                Text::new("Network Table"),
                TextFont { font_size: FontSize::Px(13.0), ..default() },
                TextColor(pal.text),
            ));
            p.spawn((
                Text::new(""),
                TextFont { font_size: FontSize::Px(11.0), ..default() },
                TextColor(pal.subtext),
                NetworkTablePanelText,
            ));
        });
}

pub fn draw_grid(
    mut gizmos: Gizmos,
    theme: Res<Theme>,
    terrain: Res<crate::terrain::TerrainHeightMap>,
) {
    // Scaled to the *actual* fetched terrain — a hardcoded 5km step only
    // covered a 20km world; anything bigger left the outer terrain grid-less.
    let half = terrain.size_km() * 0.5;
    let step = terrain.size_km() / 4.0; // 5 lines (0..=4) spanning the full terrain
    let color = theme.palette().grid.with_alpha(0.25);
    const SEGMENTS: usize = 64;
    for i in 0..=4 {
        let offset = -half + i as f32 * step;
        for segment in 0..SEGMENTS {
            let a = -half + terrain.size_km() * segment as f32 / SEGMENTS as f32;
            let b = -half + terrain.size_km() * (segment + 1) as f32 / SEGMENTS as f32;
            gizmos.line(
                Vec3::new(a, terrain.height_at(a, offset) + 0.01, offset),
                Vec3::new(b, terrain.height_at(b, offset) + 0.01, offset),
                color,
            );
            gizmos.line(
                Vec3::new(offset, terrain.height_at(offset, a) + 0.01, a),
                Vec3::new(offset, terrain.height_at(offset, b) + 0.01, b),
                color,
            );
        }
    }
}
