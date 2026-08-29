//! Manual drone control using the first connected game controller.

use bevy::{
    input::gamepad::{Gamepad, GamepadButton},
    prelude::*,
};

use crate::{
    drone::{Drone, SelectedDrone},
    factories::movement::DroneKinematics,
};

const STICK_DEAD_ZONE: f32 = 0.15;
const MAX_HORIZONTAL_SPEED_KM_S: f32 = 0.015;
const MAX_CLIMB_SPEED_KM_S: f32 = 0.005;
const MAX_DESCENT_SPEED_KM_S: f32 = 0.003;

/// Give the selected drone camera-relative-independent world-space controls.
/// Left stick is X/Z movement; right/left trigger is climb/descent.
pub fn control_selected_drone(
    selected: Res<SelectedDrone>,
    gamepads: Query<&Gamepad>,
    mut drones: Query<&mut DroneKinematics, With<Drone>>,
) {
    let Some(selected) = selected.0 else {
        return;
    };
    let Some(gamepad) = gamepads.iter().next() else {
        return;
    };
    let Ok(mut kinematics) = drones.get_mut(selected) else {
        return;
    };

    let left_stick = gamepad.left_stick();
    let climb = gamepad.get(GamepadButton::RightTrigger2).unwrap_or(0.0);
    let descend = gamepad.get(GamepadButton::LeftTrigger2).unwrap_or(0.0);
    kinematics.velocity = velocity_from_input(left_stick, climb, descend);
}

fn velocity_from_input(left_stick: Vec2, climb: f32, descend: f32) -> Vec3 {
    let horizontal = apply_radial_dead_zone(left_stick, STICK_DEAD_ZONE);
    let vertical_input = (climb - descend).clamp(-1.0, 1.0);
    let vertical_speed = if vertical_input >= 0.0 {
        vertical_input * MAX_CLIMB_SPEED_KM_S
    } else {
        vertical_input * MAX_DESCENT_SPEED_KM_S
    };

    Vec3::new(
        horizontal.x * MAX_HORIZONTAL_SPEED_KM_S,
        vertical_speed,
        -horizontal.y * MAX_HORIZONTAL_SPEED_KM_S,
    )
}

fn apply_radial_dead_zone(input: Vec2, dead_zone: f32) -> Vec2 {
    let magnitude = input.length();
    if magnitude <= dead_zone {
        return Vec2::ZERO;
    }

    let scaled_magnitude = ((magnitude.min(1.0) - dead_zone) / (1.0 - dead_zone)).min(1.0);
    input.normalize() * scaled_magnitude
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn centered_controls_hover() {
        assert_eq!(velocity_from_input(Vec2::ZERO, 0.0, 0.0), Vec3::ZERO);
    }

    #[test]
    fn stick_dead_zone_filters_small_drift() {
        assert_eq!(
            velocity_from_input(Vec2::new(0.1, -0.1), 0.0, 0.0),
            Vec3::ZERO
        );
    }

    #[test]
    fn full_stick_maps_to_horizontal_speed_limit() {
        let velocity = velocity_from_input(Vec2::new(1.0, 0.0), 0.0, 0.0);
        assert!((velocity.length() - MAX_HORIZONTAL_SPEED_KM_S).abs() < f32::EPSILON);
    }

    #[test]
    fn forward_stick_moves_toward_negative_z() {
        let velocity = velocity_from_input(Vec2::Y, 0.0, 0.0);
        assert_eq!(velocity, Vec3::new(0.0, 0.0, -MAX_HORIZONTAL_SPEED_KM_S));
    }

    #[test]
    fn triggers_use_asymmetric_vertical_limits() {
        assert_eq!(
            velocity_from_input(Vec2::ZERO, 1.0, 0.0).y,
            MAX_CLIMB_SPEED_KM_S
        );
        assert_eq!(
            velocity_from_input(Vec2::ZERO, 0.0, 1.0).y,
            -MAX_DESCENT_SPEED_KM_S
        );
    }
}
