use std::f32::consts::PI;

use bevy::{
    input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll},
    prelude::*,
};

#[derive(Resource)]
pub struct OrbitCamera {
    pub yaw: f32,
    pub pitch: f32,
    pub radius: f32,
    pub drag_total: f32,
}

impl Default for OrbitCamera {
    fn default() -> Self {
        Self { yaw: PI / 4.0, pitch: PI / 4.0, radius: 25.0, drag_total: 0.0 }
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
    if scroll.delta.y != 0.0 {
        orbit.radius = (orbit.radius - scroll.delta.y * 1.2).clamp(5.0, 60.0);
    }

    let x = orbit.radius * orbit.pitch.cos() * orbit.yaw.sin();
    let y = orbit.radius * orbit.pitch.sin();
    let z = orbit.radius * orbit.pitch.cos() * orbit.yaw.cos();

    if let Ok(mut t) = camera_q.single_mut() {
        t.translation = Vec3::new(x, y, z);
        *t = t.looking_at(Vec3::ZERO, Vec3::Y);
    }
}
