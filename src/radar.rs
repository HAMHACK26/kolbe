use bevy::{mesh::primitives::ConeAnchor, prelude::*};

use crate::{
    antenna::{Antenna, radar_direction},
    drone::SelectedDrone,
};

/// Fixed visual beam length (km). The drone knows only its pointing angles —
/// not how far the beam reaches — so the cone length is a constant, not derived
/// from link physics.
pub const BEAM_KM: f32 = 3.0;

#[derive(Component)]
pub struct RadarCone {
    pub drone_entity: Entity,
}

/// Build a cone mesh: fixed `BEAM_KM` length, half-angle = θ₃dB / 2.
/// Length is a constant — the drone only knows its pointing angles.
pub fn cone_mesh_for(antenna: &Antenna, meshes: &mut Assets<Mesh>) -> Handle<Mesh> {
    let half_angle = (antenna.theta_3db_deg / 2.0).to_radians();
    meshes.add(
        Cone { radius: BEAM_KM * half_angle.tan(), height: BEAM_KM }
            .mesh()
            .anchor(ConeAnchor::Tip)
            .build(),
    )
}

/// Place cone tip at drone_pos, base extending along the antenna boresight.
/// ConeAnchor::Tip puts origin at tip; -Y extends to base, so rotate -Y → dir.
pub fn cone_transform_for(antenna: &Antenna, drone_pos: Vec3) -> Transform {
    let dir = radar_direction(antenna.azimuth_deg, antenna.elevation_deg);
    Transform {
        translation: drone_pos,
        rotation: Quat::from_rotation_arc(Vec3::NEG_Y, dir),
        ..default()
    }
}

pub fn sync_radar_visibility(
    selected: Res<SelectedDrone>,
    mut cones: Query<(&RadarCone, &mut Visibility)>,
) {
    if !selected.is_changed() {
        return;
    }
    for (cone, mut vis) in &mut cones {
        *vis = if selected.0 == Some(cone.drone_entity) {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };
    }
}
