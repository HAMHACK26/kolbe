//! Visual demo of the collision-avoidance ring, for seeing it work.
//!
//! Launch with:
//!
//! ```text
//! KOLBE_DEMO=1 cargo run
//! ```
//!
//! Opt-in only — a plain `cargo run` starts the normal simulator and never
//! touches this module. It builds its own `App` and shares no systems,
//! resources or state with the real one: no area selection, no Lantmateriet
//! fetch, no networking, no state machine.
//!
//! Keys: `1` crossing, `2` head-on, `R` reset, `Space` pause.
//!
//! ## Why this doesn't reuse `avoidance::avoid_collisions`
//!
//! That system hardcodes `world::DRONE_RADIUS`, which models a 180 m collision
//! body independently of the smaller visual marker. A 3 m sensor ring around
//! a 180 m body is invisible at any camera distance the real app allows.
//!
//! So this demo drops the render fiction and runs at *true* scale: 0.5 m
//! airframes, a real 3 m ring, 2 m/s cruise, camera 35 m back. Nothing here is
//! exaggerated — you are looking at three actual metres.
//!
//! It calls `navigate` and `avoidance_velocity` directly, in the same order
//! the real frame does, so the control law on screen is exactly the shipping
//! one. Only the ECS gathering wrapper is bypassed (that part is covered by
//! `avoidance::tests::system_deflects_in_a_real_app`).
//!
//! ## One world-unit is one metre here
//!
//! The real simulation scales its world in kilometres. This demo uses metres
//! instead, which matters for a mundane reason: at km scale the whole scenario
//! is ~0.03 units across, entirely inside Bevy's default 0.1 near clip plane,
//! and nothing renders at all. `navigate` and `avoidance_velocity` are
//! unit-agnostic — they only need their inputs to agree — so metres in means
//! metres out, and `FlightLimits` can be used as-is without `in_km()`.

use std::f32::consts::TAU;

use bevy::prelude::*;

use crate::avoidance::{avoidance_velocity, Detection, SENSOR_RANGE_M};
use crate::navigation::{navigate, DroneState, FlightLimits};

/// A realistic small quad, not the main app's 180 m collision body. Metres.
const BODY_RADIUS: f32 = 0.5;
/// Cruise speed, m/s. Well under the ~6.9 m/s closing rate the ring can arrest.
const CRUISE_MPS: f32 = 2.0;
/// How far out the drones start, metres.
const START_X: f32 = 15.0;
/// Lateral miss distance if nothing intervened. Less than 2x body radius, so
/// on these courses they collide unless the ring deflects them.
const OFFSET: f32 = 0.4;

/// The two geometries worth looking at, because the ring responds to them
/// completely differently.
#[derive(Clone, Copy, PartialEq)]
enum Scenario {
    /// Paths crossing at 90°. Each drone sees the other roughly *abeam*, so
    /// the escape bearing is perpendicular to its own travel — the response is
    /// almost pure course change, and the trails bend visibly.
    Crossing,
    /// Near head-on. Each drone sees the other dead ahead, so the escape
    /// bearing points backwards along its own track and the response is almost
    /// pure braking. They stop nose to nose and hold a standoff instead of
    /// steering around.
    HeadOn,
}

impl Scenario {
    fn name(self) -> &'static str {
        match self {
            Scenario::Crossing => "90 degree crossing  (escape is lateral -> course change)",
            Scenario::HeadOn => "near head-on  (escape is rearward -> braking + standoff)",
        }
    }

    /// `index` 0 is the red drone, 1 is the blue one.
    fn start(self, index: usize) -> Vec3 {
        match (self, index) {
            (Scenario::Crossing, 0) => Vec3::new(-START_X, 0.0, 0.0),
            (Scenario::Crossing, _) => Vec3::new(0.0, 0.0, -START_X),
            (Scenario::HeadOn, 0) => Vec3::new(-START_X, 0.0, OFFSET / 2.0),
            (Scenario::HeadOn, _) => Vec3::new(START_X, 0.0, -OFFSET / 2.0),
        }
    }

    fn waypoint(self, index: usize) -> Vec3 {
        match (self, index) {
            (Scenario::Crossing, 0) => Vec3::new(START_X * 2.0, 0.0, 0.0),
            (Scenario::Crossing, _) => Vec3::new(0.0, 0.0, START_X * 2.0),
            (Scenario::HeadOn, 0) => Vec3::new(START_X * 2.0, 0.0, OFFSET / 2.0),
            (Scenario::HeadOn, _) => Vec3::new(-START_X * 2.0, 0.0, -OFFSET / 2.0),
        }
    }
}

#[derive(Component)]
struct Flyer {
    index: usize,
    velocity: Vec3,
    heading_deg: f32,
    waypoint: Vec3,
    radius: f32,
    trail: Vec<Vec3>,
    engaged: bool,
}

#[derive(Component)]
struct Hud;

#[derive(Resource)]
struct Stats {
    min_gap: f32,
    limits: FlightLimits,
    paused: bool,
    scenario: Scenario,
}

/// Whether `KOLBE_DEMO` asks for the demo. Requires an actual truthy value, so
/// an empty or explicitly-off setting leaves the real simulator alone.
pub fn requested() -> bool {
    match std::env::var("KOLBE_DEMO") {
        Ok(value) => {
            let value = value.trim().to_ascii_lowercase();
            !matches!(value.as_str(), "" | "0" | "false" | "no" | "off")
        }
        Err(_) => false,
    }
}

pub fn run() {
    // Metres and m/s throughout — no km conversion, see the module docs.
    let mut limits = FlightLimits::default();
    limits.set_max_speed(CRUISE_MPS);

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Kolbe — collision avoidance demo (true scale)".into(),
                resolution: (900u32, 640u32).into(),
                ..default()
            }),
            ..default()
        }))
        .insert_resource(ClearColor(Color::srgb(0.04, 0.05, 0.07)))
        .insert_resource(Stats {
            min_gap: f32::MAX,
            limits,
            paused: false,
            scenario: Scenario::Crossing,
        })
        .add_systems(Startup, setup)
        .add_systems(Update, (controls, fly, draw, hud).chain())
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Looking down at a shallow angle so the X/Z plane — the only plane
    // avoidance acts in — reads clearly.
    commands.spawn((
        Camera3d::default(),
        // Both scenarios are arranged so the conflict happens at the origin,
        // so that is what to look at. Close enough in that three metres is a
        // legible distance on screen rather than a rounding error.
        Transform::from_xyz(0.0, 20.0, 13.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn(AmbientLight { brightness: 600.0, ..default() });
    commands.spawn((
        DirectionalLight { illuminance: 6000.0, shadow_maps_enabled: false, ..default() },
        Transform::from_xyz(5.0, 20.0, 8.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    let mesh = meshes.add(Sphere::new(BODY_RADIUS));
    let red = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.35, 0.3),
        emissive: LinearRgba::new(2.0, 0.2, 0.15, 1.0),
        ..default()
    });
    let blue = materials.add(StandardMaterial {
        base_color: Color::srgb(0.35, 0.7, 1.0),
        emissive: LinearRgba::new(0.15, 0.7, 2.0, 1.0),
        ..default()
    });

    let scenario = Scenario::Crossing;
    for (index, material) in [red, blue].into_iter().enumerate() {
        commands.spawn((
            Mesh3d(mesh.clone()),
            MeshMaterial3d(material),
            Transform::from_translation(scenario.start(index)),
            Flyer {
                index,
                velocity: Vec3::ZERO,
                heading_deg: 0.0,
                waypoint: scenario.waypoint(index),
                radius: BODY_RADIUS,
                trail: Vec::new(),
                engaged: false,
            },
        ));
    }

    commands.spawn((
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(14.0),
            top: Val::Px(12.0),
            ..default()
        },
        Text::new(""),
        TextFont { font_size: FontSize::Px(16.0), ..default() },
        TextColor(Color::srgb(0.85, 0.88, 0.92)),
        Hud,
    ));
}

fn controls(
    keys: Res<ButtonInput<KeyCode>>,
    mut stats: ResMut<Stats>,
    mut flyers: Query<(&mut Transform, &mut Flyer)>,
) {
    if keys.just_pressed(KeyCode::Space) {
        stats.paused = !stats.paused;
    }

    let switched = if keys.just_pressed(KeyCode::Digit1) {
        stats.scenario = Scenario::Crossing;
        true
    } else if keys.just_pressed(KeyCode::Digit2) {
        stats.scenario = Scenario::HeadOn;
        true
    } else {
        false
    };

    if !switched && !keys.just_pressed(KeyCode::KeyR) {
        return;
    }

    stats.min_gap = f32::MAX;
    let scenario = stats.scenario;
    for (mut transform, mut flyer) in &mut flyers {
        transform.translation = scenario.start(flyer.index);
        flyer.waypoint = scenario.waypoint(flyer.index);
        flyer.velocity = Vec3::ZERO;
        flyer.heading_deg = 0.0;
        flyer.trail.clear();
        flyer.engaged = false;
    }
}

/// One tick of the real pipeline: navigate, let the ring veto, integrate the
/// vetoed velocity. Same order as `run_recovery` -> `avoid_collisions` ->
/// `apply_velocity`.
fn fly(time: Res<Time>, mut stats: ResMut<Stats>, mut flyers: Query<(&mut Transform, &mut Flyer)>) {
    let dt = time.delta_secs();
    if dt <= 0.0 || stats.paused {
        return;
    }
    let limits = stats.limits;

    let others: Vec<(Vec3, f32)> = flyers.iter().map(|(t, f)| (t.translation, f.radius)).collect();

    for (index, (mut transform, mut flyer)) in flyers.iter_mut().enumerate() {
        let position = transform.translation;
        let flown = flyer.velocity;

        let mut state =
            DroneState { position, velocity: flyer.velocity, heading_deg: flyer.heading_deg };
        navigate(&mut state, flyer.waypoint, &limits, dt);

        let detections: Vec<Detection> = others
            .iter()
            .enumerate()
            .filter(|(other, _)| *other != index)
            .map(|(_, (pos, radius))| Detection { offset: *pos - position, radius_km: *radius })
            .collect();

        let closest = detections
            .iter()
            .map(|d| d.offset.length() - flyer.radius - d.radius_km)
            .fold(f32::MAX, f32::min);
        flyer.engaged = closest <= SENSOR_RANGE_M;

        flyer.velocity = avoidance_velocity(
            flown,
            state.velocity,
            flyer.radius,
            &detections,
            SENSOR_RANGE_M,
            &limits,
            dt,
        );
        flyer.heading_deg = state.heading_deg;
        transform.translation = position + flyer.velocity * dt;

        // Breadcrumbs, so the course change reads as a curve rather than
        // something you have to catch in the instant it happens.
        if flyer.trail.last().is_none_or(|last| last.distance(transform.translation) > 0.05) {
            let point = transform.translation;
            flyer.trail.push(point);
            if flyer.trail.len() > 4000 {
                flyer.trail.remove(0);
            }
        }
    }

    // Track the closest approach of this pass.
    let positions: Vec<(Vec3, f32)> =
        flyers.iter().map(|(t, f)| (t.translation, f.radius)).collect();
    if positions.len() == 2 {
        let gap = (positions[1].0 - positions[0].0).length() - positions[0].1 - positions[1].1;
        stats.min_gap = stats.min_gap.min(gap);
    }
}

fn draw(mut gizmos: Gizmos, flyers: Query<(&Transform, &Flyer)>) {
    // 1 m reference grid, so the 3 m ring has something to be measured against
    // by eye.
    let grid = Color::srgb(0.13, 0.15, 0.18);
    let extent = 20;
    for i in -extent..=extent {
        let d = i as f32;
        let e = extent as f32;
        gizmos.line(Vec3::new(d, 0.0, -e), Vec3::new(d, 0.0, e), grid);
        gizmos.line(Vec3::new(-e, 0.0, d), Vec3::new(e, 0.0, d), grid);
    }

    for (transform, flyer) in &flyers {
        let position = transform.translation;

        // The sensor ring: body radius + 3 m of reach. Lights up when
        // something is actually inside it.
        let ring_color = if flyer.engaged {
            Color::srgb(1.0, 0.75, 0.2)
        } else {
            Color::srgb(0.25, 0.3, 0.36)
        };
        circle_xz(&mut gizmos, position, flyer.radius + SENSOR_RANGE_M, ring_color);
        // The hull, so you can see the gap is measured between surfaces rather
        // than between centres.
        circle_xz(&mut gizmos, position, flyer.radius, Color::srgb(0.45, 0.5, 0.58));

        for pair in flyer.trail.windows(2) {
            gizmos.line(pair[0], pair[1], Color::srgb(0.35, 0.45, 0.55));
        }
    }

    // The gap under measurement.
    let all: Vec<&Transform> = flyers.iter().map(|(t, _)| t).collect();
    if all.len() == 2 {
        gizmos.line(all[0].translation, all[1].translation, Color::srgb(0.9, 0.55, 0.2));
    }
}

fn hud(
    stats: Res<Stats>,
    flyers: Query<(&Transform, &Flyer)>,
    mut text: Query<&mut Text, With<Hud>>,
) {
    let Ok(mut text) = text.single_mut() else {
        return;
    };
    let bodies: Vec<(&Transform, &Flyer)> = flyers.iter().collect();
    if bodies.len() != 2 {
        return;
    }
    let gap = (bodies[1].0.translation - bodies[0].0.translation).length()
        - bodies[0].1.radius
        - bodies[1].1.radius;
    let engaged = bodies.iter().any(|(_, f)| f.engaged);

    **text = format!(
        "{}\n\
         sensor ring: {:.2} m   |   body radius: {:.2} m   |   cruise: {:.1} m/s\n\
         gap: {:>7.3} m   {}\n\
         speeds: {:>5.2} m/s  /  {:>5.2} m/s\n\
         closest approach this pass: {:.3} m\n\
         [1] crossing   [2] head-on   [R] reset   [Space] {}",
        stats.scenario.name(),
        SENSOR_RANGE_M,
        BODY_RADIUS,
        CRUISE_MPS,
        gap,
        if gap <= 0.0 {
            "*** CONTACT ***"
        } else if engaged {
            "RING ENGAGED"
        } else {
            "clear"
        },
        bodies[0].1.velocity.length(),
        bodies[1].1.velocity.length(),
        if stats.min_gap == f32::MAX { 0.0 } else { stats.min_gap },
        if stats.paused { "resume" } else { "pause" },
    );
}

/// Bevy's circle gizmo API moves around between releases; a hand-rolled ring
/// out of line segments is stable and costs nothing at this scale.
fn circle_xz(gizmos: &mut Gizmos, center: Vec3, radius: f32, color: Color) {
    const SEGMENTS: usize = 64;
    for k in 0..SEGMENTS {
        let a = k as f32 / SEGMENTS as f32 * TAU;
        let b = (k + 1) as f32 / SEGMENTS as f32 * TAU;
        gizmos.line(
            center + Vec3::new(a.cos() * radius, 0.0, a.sin() * radius),
            center + Vec3::new(b.cos() * radius, 0.0, b.sin() * radius),
            color,
        );
    }
}
