use bevy::{picking::prelude::*, prelude::*};

use crate::{
    base::{CommandQueue, base_position},
    camera::OrbitCamera,
    drone::{Drone, DroneType, SelectedDrone, drone_id, make_antenna},
    factories::{DroneAi, movement::DroneKinematics},
    navigation::{
        BOUNDARY_MARGIN_KM, DriftVector, MAX_LINK_SPACING_KM, MIN_SEPARATION_KM, PatrolVolume,
    },
    networking::{MeshRow, MeshTable, NetworkingBundle},
    radar::{RadarCone, cone_mesh_for, cone_transform_for},
    recovery::{ContactMemory, RecoveryState},
    seeking::SeekState,
    theme::{Theme, ThemeRole},
    ui::{
        InfoPopup, InfoPopupTable, InfoPopupTitle, NetworkTableButton, NetworkTablePanelText,
        NetworkTablePopup,
    },
};

pub const WORLD_SIZE: f32 = 20.0;
/// How many drones it takes to cover the patrol area.
///
/// Not a free choice — it falls out of the radio. The area is 14 x 14 km and
/// no two neighbours may sit further apart than
/// [`MAX_LINK_SPACING_KM`] (3.0 km) or they cannot link at all, so the grid
/// needs `ceil(14 / 3) = 6` drones per side. Fewer drones would leave the mesh
/// permanently partitioned no matter how well the antennas were aimed; this is
/// the count that gives the area continuous coverage.
pub const DRONE_COUNT: usize = 36;
/// Extra airframes above the minimum spacing-derived coverage count. This
/// leaves replacement paths when a node drops or has to return to base.
pub const COVERAGE_REDUNDANCY: f32 = 0.15;
/// Drawn radius of a drone, km. Still far larger than a real airframe — a
/// true-scale quad is sub-pixel on a 20 km map — but 4x smaller than the
/// original marker, so the 3 km separation ring reads as real distance between
/// drones rather than a couple of body widths.
pub const DRONE_RADIUS: f32 = 0.045;

/// Fixed seed for the spawn scatter and each drone's drift stream, so a run is
/// reproducible: same layout, same sequence of direction changes.
pub const SPAWN_SEED: u64 = 0x5EED_D40E_5EED_D40E;

/// Lay out `count` ground positions inside `volume`, keeping every adjacent
/// pair both far enough apart to be legal and close enough to talk.
///
/// A jittered grid rather than rejection sampling, because the guarantee is
/// structural instead of probabilistic. Points sit at the centers of a lattice
/// and are then pushed around inside their own cell. The jitter is bounded
/// from *both* sides, which is the whole trick:
///
///   - it can't exceed `(pitch − MIN_SEPARATION) / 2`, or two neighbours could
///     close past the separation floor;
///   - it can't exceed `(MAX_LINK_SPACING − pitch) / 2`, or two neighbours
///     could drift past the radio's reach and the mesh would come up with a
///     permanent hole in it.
///
/// So the layout is random but every neighbour pair lands inside
/// `[MIN_SEPARATION_KM, MAX_LINK_SPACING_KM]` by construction, with no retry
/// loop that might not terminate.
///
/// Returns horizontal positions only — altitude is measured from the terrain,
/// which this function has no view of (see [`setup`]).
pub fn scatter_spawn_points(count: usize, volume: &PatrolVolume, seed: u64) -> Vec<Vec2> {
    if count == 0 {
        return Vec::new();
    }
    let columns = (count as f32).sqrt().ceil() as usize;
    let rows = count.div_ceil(columns);
    let span = volume.span_km();

    // Cell pitch. Points sit at cell centers, so N cells across a span of S
    // have pitch S/N and the outermost centers sit half a pitch off the wall.
    let pitch = Vec2::new(span.x / columns as f32, span.y / rows as f32);
    // Bounded from both sides — see the doc comment. A negative bound (a grid
    // too coarse or too fine for the limits) clamps to zero: a plain lattice.
    let bounded_jitter = |pitch: f32| {
        ((pitch - MIN_SEPARATION_KM) / 2.0)
            .min((MAX_LINK_SPACING_KM - pitch) / 2.0)
            .max(0.0)
    };
    let jitter = Vec2::new(bounded_jitter(pitch.x), bounded_jitter(pitch.y));

    let mut rng = seed;
    let next = |state: &mut u64| {
        *state = state
            .wrapping_add(0x9e37_79b9_7f4a_7c15)
            .wrapping_mul(0xbf58_476d_1ce4_e5b9);
        ((*state >> 40) as f32 / (1u64 << 24) as f32) * 2.0 - 1.0 // [-1, 1)
    };

    (0..count)
        .map(|i| {
            let (column, row) = (i % columns, i / columns);
            Vec2::new(
                volume.min.x + (column as f32 + 0.5) * pitch.x + next(&mut rng) * jitter.x,
                volume.min.z + (row as f32 + 0.5) * pitch.y + next(&mut rng) * jitter.y,
            )
        })
        .collect()
}

/// Number of drones for continuous radio coverage of `volume`, including a
/// 15% redundancy reserve.
///
/// `ceil(width / MAX_LINK_SPACING_KM) * ceil(height / MAX_LINK_SPACING_KM)`
/// is the smallest rectangular lattice whose cell pitch stays within the
/// safe link spacing. The final `ceil(... * 1.15)` adds spare nodes without
/// weakening that guarantee; [`scatter_spawn_points`] recomputes the denser
/// lattice from this count.
pub fn drones_required_for_coverage(volume: &PatrolVolume) -> usize {
    let span = volume.span_km();
    let columns = (span.x / MAX_LINK_SPACING_KM).ceil().max(1.0) as usize;
    let rows = (span.y / MAX_LINK_SPACING_KM).ceil().max(1.0) as usize;
    ((columns * rows) as f32 * (1.0 + COVERAGE_REDUNDANCY)).ceil() as usize
}

pub fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    terrain: Res<crate::terrain::TerrainHeightMap>,
    theme: Res<Theme>,
    mut patrol: ResMut<PatrolVolume>,
    area: Res<crate::area::ScenarioArea>,
    selected_base: Res<crate::area::BasePosition>,
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

    // The target area: the world inset by BOUNDARY_MARGIN_KM on every side, so
    // there is always empty world between the formation and the map edge.
    let volume = PatrolVolume::inset(terrain.size_km(), BOUNDARY_MARGIN_KM);
    *patrol = volume;
    // Keep each hop within the radio budget for every valid target-area size.
    // At the 50 km UI limit this produces a 15×15 coverage grid, rather than
    // stretching the old 36 drones until the network partitions.
    let drone_count = drones_required_for_coverage(&volume);
    let slots_per_side = (drone_count as f32).sqrt().ceil() as usize;
    let mut positions = scatter_spawn_points(drone_count, &volume, SPAWN_SEED);
    for (row, strip) in positions.chunks_mut(slots_per_side).enumerate() {
        if row % 2 == 1 {
            strip.reverse();
        }
    }

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

    // ── Pass 1: settle identities and final positions ────────────────────────
    //
    // Both are needed to write the mission briefing below, and a drone's UUID
    // only exists once its `NetworkingBundle` has been built — so build them
    // all first, then spawn.
    let placed: Vec<(Vec3, NetworkingBundle)> = positions
        .iter()
        .enumerate()
        .map(|(i, &ground_xz)| {
            // Altitude is measured from the terrain, so each drone launches
            // mid-band over whatever is actually beneath it.
            let ground = terrain.height_at(ground_xz.x, ground_xz.y);
            let (floor, ceiling) = PatrolVolume::altitude_band(ground);
            let pos = Vec3::new(ground_xz.x, (floor + ceiling) / 2.0, ground_xz.y);
            (pos, NetworkingBundle::random(i))
        })
        .collect();

    // ── The mission briefing ─────────────────────────────────────────────────
    //
    // Every drone launches already knowing the formation: who else is up, where
    // they were placed, and who each of them is meant to be linked to. Without
    // this the mesh cannot cold-start — `maintain_mesh_antennas` has nothing to
    // aim at until a header arrives, but no header can arrive until an antenna
    // is aimed. Real formations brief the same way; the tests in `networking`
    // and `tracking` already seed rows for exactly this reason.
    //
    // Locations are base-relative, matching how every other mesh row is stored.
    let base_pos = match selected_base.0 {
        Some((lat, lon)) => {
            let x = ((lon - area.longitude) * 111.320 * area.latitude.to_radians().cos()) as f32;
            let z = ((lat - area.latitude) * 110.574) as f32;
            Vec3::new(x, terrain.height_at(x, z) + DRONE_RADIUS, z)
        }
        None => base_position(&terrain),
    };
    let count = placed.len();
    let briefing: Vec<MeshRow> = placed
        .iter()
        .enumerate()
        .map(|(i, (pos, networking))| MeshRow {
            id: networking.uuid.0.clone(),
            // Briefed, not observed — timestamp 0 makes it maximally stale, so
            // the first real header from that peer always supersedes it.
            timestamp: 0.0,
            location: *pos - base_pos,
            neighbour_distance: 1,
            // The serpentine ordering makes previous/next physical neighbours.
            connections: vec![
                placed[(i + count - 1) % count].1.uuid.0.clone(),
                placed[(i + 1) % count].1.uuid.0.clone(),
            ],
        })
        .collect();

    for (i, (drone_pos, networking)) in placed.into_iter().enumerate() {
        let drone_pos = drone_pos;
        let drone_type = if i % 3 == 0 { DroneType::Attack } else { DroneType::Node };

        let az0 = (i as f32 * 137.5) % 360.0;
        let el0 = ((i as f32 * 23.0) % 30.0) - 15.0;

        // Every drone carries exactly 3 antennas, 120° apart. Initial layout
        // only — `maintain_mesh_antennas` retargets every frame based on
        // each drone's ring-index neighbors.
        let antennas = vec![
            make_antenna(az0, el0, i),
            make_antenna((az0 + 120.0) % 360.0, el0, i + 100),
            make_antenna((az0 + 240.0) % 360.0, el0, i + 200),
        ];

        // The drift deadline is anchored to this drone's own clock. That clock
        // starts at a random offset (up to a day out), so anchoring to 0.0
        // would put the first deadline immediately in the past.
        let drift = DriftVector::seeded(SPAWN_SEED ^ i as u64, networking.clock.now);

        // This drone's copy of the briefing — everyone but itself.
        let mut networking = networking;
        networking.mesh_table = MeshTable(
            briefing
                .iter()
                .filter(|row| row.id != networking.uuid.0)
                .map(|row| (row.id.clone(), row.clone()))
                .collect(),
        );

        let drone_entity = commands
            .spawn((
                Mesh3d(drone_mesh.clone()),
                MeshMaterial3d(drone_mat.clone()),
                Transform::from_translation(drone_pos),
                Drone { id: drone_id(i), drone_type, antennas: antennas.clone() },
                DroneKinematics::default(),
                DroneAi::default(),
                CommandQueue::default(),
                networking,
                drift,
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

        for antenna in &antennas {
            commands.spawn((
                Mesh3d(cone_mesh_for(antenna, &mut meshes)),
                MeshMaterial3d(cone_mat.clone()),
                // heading 0.0 matches the DroneKinematics::default() this
                // drone spawns with; apply_velocity updates heading as it
                // moves, but nothing currently re-syncs the cone transform.
                cone_transform_for(antenna, 0.0, drone_pos),
                Visibility::Hidden,
                RadarCone { drone_entity },
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

/// Draw the patrol volume — the box the drones are actually allowed to fly in.
///
/// Without this the target area is invisible and the flight behavior reads as
/// arbitrary: drones stop at nothing, turn at nothing. Drawn as a wireframe
/// box in the accent color, with the floor ring emphasised (it's the edge that
/// reads against the terrain) and the vertical corner posts tying the two
/// altitude bounds together.
pub fn draw_patrol_volume(mut gizmos: Gizmos, theme: Res<Theme>, volume: Res<PatrolVolume>) {
    let (min, max) = (volume.min, volume.max);
    let color = theme.palette().accent;
    let floor = color.with_alpha(0.55);
    let ceiling = color.with_alpha(0.22);
    let post = color.with_alpha(0.30);

    // The four corners, in order, so consecutive pairs are box edges.
    let corners = |y: f32| {
        [
            Vec3::new(min.x, y, min.z),
            Vec3::new(max.x, y, min.z),
            Vec3::new(max.x, y, max.z),
            Vec3::new(min.x, y, max.z),
        ]
    };
    let low = corners(min.y);
    let high = corners(max.y);

    for i in 0..4 {
        let next = (i + 1) % 4;
        gizmos.line(low[i], low[next], floor);
        gizmos.line(high[i], high[next], ceiling);
        gizmos.line(low[i], high[i], post);
    }
}

pub fn draw_grid(
    mut gizmos: Gizmos,
    theme: Res<Theme>,
    terrain: Res<crate::terrain::TerrainHeightMap>,
) {
    // Scaled to the *actual* fetched terrain, not the fixed `WORLD_SIZE` the
    // hand-placed drone ring is designed for — a hardcoded 5km step only
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

    #[test]
    fn coverage_count_adds_fifteen_percent_redundancy() {
        // A 20 km target with 3 km margins has a 14 × 14 km patrol area.
        // Five safe 3 km cells fit per side: ceil(25 × 1.15) = 29 drones.
        let volume = PatrolVolume::inset(20.0, BOUNDARY_MARGIN_KM);
        assert_eq!(drones_required_for_coverage(&volume), 29);
    }

    #[test]
    fn redundant_formation_keeps_neighbour_spacing_safe() {
        let volume = PatrolVolume::inset(50.0, BOUNDARY_MARGIN_KM);
        let count = drones_required_for_coverage(&volume);
        let columns = (count as f32).sqrt().ceil() as usize;
        let rows = count.div_ceil(columns);
        let span = volume.span_km();
        assert!(span.x / columns as f32 <= MAX_LINK_SPACING_KM);
        assert!(span.y / rows as f32 <= MAX_LINK_SPACING_KM);
    }
}
