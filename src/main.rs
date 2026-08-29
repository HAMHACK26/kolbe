mod antenna;
mod area;
mod base;
mod camera;
mod drone;
mod factories;
mod navigation;
mod networking;
mod radar;
mod recovery;
mod seeking;
mod spherical;
mod terrain;
mod theme;
mod tracking;
mod ui;
mod world;

use bevy::{picking::prelude::MeshPickingPlugin, prelude::*};

use camera::OrbitCamera;
use drone::SelectedDrone;
use theme::Theme;

#[derive(States, Clone, Copy, Debug, Default, Eq, Hash, PartialEq)]
enum AppState {
    #[default]
    AreaSelection,
    LoadingTerrain,
    Simulation,
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Drone Map — 20km × 20km".into(),
                resolution: (1100u32, 800u32).into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(MeshPickingPlugin)
        .init_state::<AppState>()
        .init_resource::<area::ScenarioArea>()
        .insert_resource(SelectedDrone(None))
        .insert_resource(OrbitCamera::default())
        .insert_resource(Theme::default())
        // Seed the clear color from the palette so frame 0 matches; apply_theme
        // keeps it in sync afterwards.
        .insert_resource(ClearColor(Theme::default().palette().bg))
        .init_resource::<networking::Mailbox>()
        .init_resource::<networking::ReconnectBus>()
        .init_resource::<networking::ReconnectRequests>()
        .init_resource::<ui::NetworkTablePanelOpen>()
        .add_systems(Startup, (ui::setup_camera, theme::setup_moon))
        .add_systems(OnEnter(AppState::AreaSelection), area::setup)
        .add_systems(Update, area::interactions.run_if(in_state(AppState::AreaSelection)))
        .add_systems(OnExit(AppState::AreaSelection), area::cleanup)
        .add_systems(OnEnter(AppState::LoadingTerrain), terrain::start_loading)
        .add_systems(
            Update,
            (terrain::poll_loading, terrain::update_progress)
                .run_if(in_state(AppState::LoadingTerrain)),
        )
        .add_systems(OnExit(AppState::LoadingTerrain), terrain::cleanup_loading)
        .add_systems(
            OnEnter(AppState::Simulation),
            (
                terrain::spawn_mesh,
                world::setup,
                base::spawn_base,
                ui::make_camera_overlay,
            )
                .chain(),
        )
        .add_systems(Update, camera::orbit_camera.run_if(in_state(AppState::Simulation)))
        .add_systems(Update, radar::sync_radar_visibility.run_if(in_state(AppState::Simulation)))
        .add_systems(Update, ui::update_popup_position.run_if(in_state(AppState::Simulation)))
        .add_systems(Update, world::draw_grid.run_if(in_state(AppState::Simulation)))
        .add_systems(Update, terrain::draw_contours.run_if(in_state(AppState::Simulation)))
        .add_systems(Update, theme::moon_toggle)
        .add_systems(Update, theme::apply_theme)
        .add_systems(Update, factories::movement::apply_velocity.run_if(in_state(AppState::Simulation)))
        .add_systems(
            Update,
            networking::advance_clocks.run_if(in_state(AppState::Simulation)),
        )
        .add_systems(
            Update,
            (
                // Priority reconnection flood first, so a fresh slew-freeze is
                // visible to the aiming systems this same frame.
                networking::process_reconnect,
                // Antenna aiming (tracking::maintain_mesh_antennas,
                // seeking::seek_lost_links) is disabled for now — antennas and
                // radar cones stay at their spawn angles. Wiring live aiming
                // back in is a future PR.
                networking::detect_links_and_send_headers,
                networking::route_packets,
                // Partition detection + recovery run last — they need the
                // freshly (re)detected links and updated mesh table.
                recovery::detect_partitions,
                recovery::run_recovery,
            )
                .chain()
                .run_if(in_state(AppState::Simulation)),
        )
        .run();
}
