use std::f32::consts::PI;

use bevy::{
    input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll},
    prelude::*,
};

/// Closest the camera may orbit to its target — 1 meter (world space is km).
const MIN_RADIUS_KM: f32 = 0.001;
/// Farthest the camera may orbit from its target.
const MAX_RADIUS_KM: f32 = 60.0;

#[derive(Resource)]
pub struct OrbitCamera {
    pub yaw: f32,
    pub pitch: f32,
    pub radius: f32,
    pub drag_total: f32,
    /// World-space point the camera orbits around. Right-drag pans this
    /// across the ground plane — without it, being zoomed in to `MIN_RADIUS_KM`
    /// would only ever let you inspect the one fixed point at the world
    /// origin instead of anywhere on the map.
    pub target: Vec3,
}

impl Default for OrbitCamera {
    fn default() -> Self {
        Self { yaw: PI / 4.0, pitch: PI / 4.0, radius: 25.0, drag_total: 0.0, target: Vec3::ZERO }
    }
}

pub fn orbit_camera(
    mut orbit: ResMut<OrbitCamera>,
    mut camera_q: Query<&mut Transform, With<Camera3d>>,
    mouse_button: Res<ButtonInput<MouseButton>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
) {
    if mouse_button.just_pressed(MouseButton::Left) {
        orbit.drag_total = 0.0;
    }
    if mouse_button.pressed(MouseButton::Left) && motion.delta != Vec2::ZERO {
        orbit.drag_total += motion.delta.length();
        orbit.yaw -= motion.delta.x * 0.006;
        orbit.pitch = (orbit.pitch - motion.delta.y * 0.006).clamp(0.08, PI / 2.0 - 0.05);
    }

    // Right-drag pans the orbit target across the ground plane, so you can
    // actually explore the map once zoomed in close instead of being stuck
    // orbiting the one fixed point at the world origin.
    if mouse_button.pressed(MouseButton::Right) && motion.delta != Vec2::ZERO {
        let (sin_yaw, cos_yaw) = orbit.yaw.sin_cos();
        // Ground-plane right/forward for the camera's current yaw (see
        // `orbit_camera`'s position formula below: at yaw = 0 the camera
        // sits on +Z looking toward -Z, so right = +X, forward = -Z).
        let right = Vec3::new(cos_yaw, 0.0, -sin_yaw);
        let forward = Vec3::new(-sin_yaw, 0.0, -cos_yaw);
        // Pan speed scales with the current distance so a drag feels the
        // same on screen whether zoomed out to 60 km or in to 1 m.
        let pan_scale = orbit.radius * 0.0015;
        orbit.target -= right * motion.delta.x * pan_scale;
        orbit.target += forward * motion.delta.y * pan_scale;
    }

    if scroll.delta.y != 0.0 {
        // Multiplicative zoom: a fixed km-per-notch step would be useless at
        // either end of a 1 m .. 60 km range, so each notch instead covers a
        // fixed fraction of the *current* distance.
        let factor = (1.0 - scroll.delta.y * 0.12).max(0.1);
        orbit.radius = (orbit.radius * factor).clamp(MIN_RADIUS_KM, MAX_RADIUS_KM);
    }

    let x = orbit.radius * orbit.pitch.cos() * orbit.yaw.sin();
    let y = orbit.radius * orbit.pitch.sin();
    let z = orbit.radius * orbit.pitch.cos() * orbit.yaw.cos();

    if let Ok(mut t) = camera_q.single_mut() {
        t.translation = orbit.target + Vec3::new(x, y, z);
        *t = t.looking_at(orbit.target, Vec3::Y);
    }
}
