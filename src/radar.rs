use bevy::{mesh::primitives::ConeAnchor, prelude::*};

use crate::{
    antenna::{Antenna, radar_direction},
    drone::SelectedDrone,
    world::WORLD_SIZE,
};

#[derive(Component)]
pub struct RadarCone {
    pub drone_entity: Entity,
}

/// Build a cone mesh whose geometry derives from antenna physics:
/// length = max_range_km, half-angle = θ₃dB / 2.
pub fn cone_mesh_for(antenna: &Antenna, meshes: &mut Assets<Mesh>) -> Handle<Mesh> {
    let range = antenna.max_range_km().min(WORLD_SIZE * 1.5);
    let half_angle = (antenna.theta_3db_deg / 2.0).to_radians();
    meshes.add(
        Cone { radius: range * half_angle.tan(), height: range }
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
