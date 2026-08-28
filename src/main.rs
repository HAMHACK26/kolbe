mod antenna;
mod base;
mod camera;
mod drone;
mod factories;
mod networking;
mod radar;
mod spherical;
mod theme;
mod tracking;
mod ui;
mod world;

use bevy::{picking::prelude::MeshPickingPlugin, prelude::*};

use camera::OrbitCamera;
use drone::SelectedDrone;
use theme::Theme;

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
        .insert_resource(SelectedDrone(None))
        .insert_resource(OrbitCamera::default())
        .insert_resource(Theme::default())
        .insert_resource(ClearColor(Color::BLACK))
        .init_resource::<networking::Mailbox>()
        .init_resource::<ui::NetworkTablePanelOpen>()
        .add_systems(Startup, world::setup)
        .add_systems(Startup, base::spawn_base)
        .add_systems(Startup, theme::setup_moon)
        .add_systems(Update, camera::orbit_camera)
        .add_systems(Update, radar::sync_radar_visibility)
        .add_systems(Update, ui::update_popup_position)
        .add_systems(Update, world::draw_grid)
        .add_systems(Update, theme::moon_toggle)
        .add_systems(Update, theme::apply_theme)
        .add_systems(Update, factories::movement::apply_velocity)
        .add_systems(Update, networking::advance_clocks)
        .add_systems(
            Update,
            (
                tracking::maintain_mesh_antennas,
                networking::detect_links_and_send_headers,
                networking::route_packets,
            )
                .chain(),
        )
        .run();
}
