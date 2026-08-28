mod antenna;
mod area;
mod base;
mod camera;
mod drone;
mod factories;
mod radar;
mod terrain;
mod theme;
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
        .insert_resource(ClearColor(Color::BLACK))
        .add_systems(Startup, (terrain::start_local_server, ui::setup_camera, theme::setup_moon))
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
        .run();
}
