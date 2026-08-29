use bevy::{picking::prelude::*, prelude::*};

use crate::{
    antenna::{Antenna, angles_toward},
    base::{Base, CommandQueue},
    camera::OrbitCamera,
    drone::{Drone, DroneType, SelectedDrone, drone_id, make_antenna},
    factories::{DroneAi, movement::DroneKinematics},
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
pub const DRONE_RADIUS: f32 = 0.045;

/// Hard cap on the distance between two ring neighbors, km.
///
/// The link budget in `make_antenna` closes at ~3.5 km on boresight
/// (`Antenna::max_range_km`), so holding neighbors at 3 km keeps every mesh
/// hop inside range with margin for terrain relief and drift.
pub const MAX_NEIGHBOR_SPACING_KM: f32 = 3.0;

/// Keep the formation ring this far inside the terrain edge, km, so no drone
/// spawns off the fetched height map.
const RING_TERRAIN_MARGIN_KM: f32 = 0.5;

/// The startup formation: `DRONE_COUNT` drones on a ring centered on the
/// terrain, spread as wide as the radio allows.
///
/// The ring sits at the largest radius whose neighbor spacing still respects
/// [`MAX_NEIGHBOR_SPACING_KM`]. A ring of N at radius R has a neighbor chord
/// of 2·R·sin(π/N), so the cap inverts to R = spacing / (2·sin(π/N)); that is
/// then clamped so the ring fits inside the fetched terrain, whose side length
/// is whatever area was picked on the map and can be far smaller than the ring
/// the link budget alone would allow.
///
/// `height_at` is the terrain sampler (`TerrainHeightMap::height_at`), taken as
/// a closure so the geometry can be exercised against synthetic terrain.
pub fn ring_formation(terrain_size_km: f32, height_at: impl Fn(f32, f32) -> f32) -> Vec<Vec3> {
    let n = DRONE_COUNT as f32;
    let chord_per_radius = 2.0 * (std::f32::consts::PI / n).sin();
    let terrain_limit = (terrain_size_km * 0.5 - RING_TERRAIN_MARGIN_KM).max(0.1);
    let mut radius = (MAX_NEIGHBOR_SPACING_KM / chord_per_radius).min(terrain_limit);

    let ring_at = |radius: f32, i: usize| {
        let angle = i as f32 / n * std::f32::consts::TAU;
        let (x, z) = (radius * angle.sin(), radius * angle.cos());
        Vec3::new(x, height_at(x, z) + DRONE_RADIUS, z)
    };

    // The chord above is a *flat* distance, but the drones sit on the terrain,
    // so relief stretches the real 3-D neighbor distance past the cap. Shrink
    // until the true spacing is inside it — iteratively, because moving the
    // ring inward also changes which terrain it lands on.
    for _ in 0..32 {
        let worst = (0..DRONE_COUNT)
            .map(|i| ring_at(radius, i).distance(ring_at(radius, (i + 1) % DRONE_COUNT)))
            .fold(0.0f32, f32::max);
        if worst <= MAX_NEIGHBOR_SPACING_KM {
            break;
        }
        radius *= (MAX_NEIGHBOR_SPACING_KM / worst) * 0.999;
    }

    (0..DRONE_COUNT).map(|i| ring_at(radius, i)).collect()
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
    let cone_mat = materials.add(StandardMaterial {
        base_color: pal.drone_cone.with_alpha(0.30),
        emissive: LinearRgba::new(0.0, 0.4, 0.8, 0.0),
        alpha_mode: AlphaMode::Blend,
        double_sided: true,
        cull_mode: None,
        ..default()
    });

    let ring = ring_formation(terrain.size_km(), |x, z| terrain.height_at(x, z));
    // The base spawns before this system (see `main`), so this is the real
    // base position every drone's antenna #2 points at from frame one.
    let base_pos = bases.iter().next().map(|b| b.position).unwrap_or(Vec3::ZERO);

    for i in 0..DRONE_COUNT {
        let drone_pos = ring[i];
        let drone_type = if i % 3 == 0 { DroneType::Attack } else { DroneType::Node };
        let antennas = formation_antennas(&ring, i, base_pos);

        let drone_entity = commands
            .spawn((
                Mesh3d(drone_mesh.clone()),
                MeshMaterial3d(drone_mat.clone()),
                Transform::from_translation(drone_pos),
                Drone { id: drone_id(i), drone_type, antennas: antennas.clone() },
                DroneKinematics::default(),
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

#[cfg(test)]
mod tests {
    use super::*;

    /// Flat ground at a fixed elevation.
    fn flat(elevation_km: f32) -> impl Fn(f32, f32) -> f32 {
        move |_, _| elevation_km
    }

    /// A ridge running east–west: elevation swings with Z, so the ring's
    /// north and south arcs sit far apart vertically.
    fn ridge(amplitude_km: f32) -> impl Fn(f32, f32) -> f32 {
        move |_, z| amplitude_km * (z * 0.5).sin()
    }

    fn worst_neighbor_spacing(ring: &[Vec3]) -> f32 {
        (0..ring.len())
            .map(|i| ring[i].distance(ring[(i + 1) % ring.len()]))
            .fold(0.0f32, f32::max)
    }

    /// Neighbors never exceed the cap, and on flat ground the formation
    /// actually *uses* it — "spread as wide as possible" is the point, so a
    /// timid ring is as wrong as an over-wide one.
    #[test]
    fn flat_ring_spreads_right_up_to_the_spacing_cap() {
        let ring = ring_formation(20.0, flat(0.3));
        assert_eq!(ring.len(), DRONE_COUNT);

        let worst = worst_neighbor_spacing(&ring);
        assert!(worst <= MAX_NEIGHBOR_SPACING_KM, "spacing {worst} exceeds cap");
        assert!(
            worst > MAX_NEIGHBOR_SPACING_KM - 0.01,
            "spacing {worst} leaves the formation needlessly bunched up"
        );
    }

    /// Terrain relief stretches the true 3-D distance past the flat chord —
    /// the ring has to shrink until the real spacing is back inside the cap.
    #[test]
    fn hilly_ring_shrinks_until_true_spacing_fits() {
        let ring = ring_formation(20.0, ridge(1.5));
        let worst = worst_neighbor_spacing(&ring);
        assert!(worst <= MAX_NEIGHBOR_SPACING_KM, "spacing {worst} exceeds cap on rough terrain");

        // It shrank because of the relief, not because the terrain was small.
        let flat_radius = ring_formation(20.0, flat(0.0))[0].xz().length();
        let hilly_radius = ring[0].xz().length();
        assert!(hilly_radius < flat_radius, "{hilly_radius} should be tighter than {flat_radius}");
    }

    /// A small fetched area clamps the ring, so no drone spawns off the edge
    /// of the height map.
    #[test]
    fn ring_stays_inside_a_small_terrain() {
        let size_km = 4.0;
        let ring = ring_formation(size_km, flat(0.0));
        let half = size_km * 0.5;
        for p in &ring {
            assert!(
                p.x.abs() <= half && p.z.abs() <= half,
                "{p:?} is outside a {size_km}km terrain"
            );
        }
        assert!(worst_neighbor_spacing(&ring) <= MAX_NEIGHBOR_SPACING_KM);
    }

    /// The formation is *connected on startup*: with the spawn aim, every
    /// drone's antenna #1/#3 close a link to its two ring neighbors on frame
    /// 0 — the same `rssi >= sensitivity` test
    /// `networking::detect_links_and_send_headers` applies.
    #[test]
    fn every_drone_links_both_neighbors_on_spawn() {
        let ring = ring_formation(20.0, ridge(0.4));
        let base_pos = Vec3::new(0.0, 0.0, -9.0);

        for i in 0..DRONE_COUNT {
            let antennas = formation_antennas(&ring, i, base_pos);
            for neighbor in [(i + 1) % DRONE_COUNT, (i + DRONE_COUNT - 1) % DRONE_COUNT] {
                let peer = ring[neighbor];
                let distance_km = (peer - ring[i]).length();
                // Heading is 0 at spawn, so the drone-relative azimuths the
                // antennas carry are already world-frame.
                let best = antennas
                    .iter()
                    .map(|a| {
                        a.rssi_dbm(a.off_boresight_deg(0.0, ring[i], peer), 0.0, distance_km)
                            - a.sensitivity_dbm
                    })
                    .fold(f32::NEG_INFINITY, f32::max);
                assert!(best >= 0.0, "drone {i} misses neighbor {neighbor} by {best} dB");
            }
        }
    }

    /// Only the ring neighbors are linked. Non-adjacent drones must stay
    /// invisible to each other so the mesh table's relayed, multi-hop rows are
    /// the only way they learn about one another.
    #[test]
    fn non_neighbors_are_not_linked_on_spawn() {
        let ring = ring_formation(20.0, flat(0.2));
        let base_pos = Vec3::new(0.0, 0.0, -9.0);

        for i in 0..DRONE_COUNT {
            let antennas = formation_antennas(&ring, i, base_pos);
            for (j, &peer) in ring.iter().enumerate() {
                let adjacent = j == (i + 1) % DRONE_COUNT || j == (i + DRONE_COUNT - 1) % DRONE_COUNT;
                if j == i || adjacent {
                    continue;
                }
                let distance_km = (peer - ring[i]).length();
                let linked = antennas.iter().any(|a| {
                    a.rssi_dbm(a.off_boresight_deg(0.0, ring[i], peer), 0.0, distance_km)
                        >= a.sensitivity_dbm
                });
                assert!(!linked, "drone {i} should not see non-neighbor {j}");
            }
        }
    }
}
