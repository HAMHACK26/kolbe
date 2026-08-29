use bevy::{mesh::primitives::ConeAnchor, prelude::*};

use crate::{
    antenna::{Antenna, radar_direction},
    base::Base,
    drone::Drone,
    factories::movement::DroneKinematics,
};

/// Fixed visual beam length (km). The drone knows only its pointing angles —
/// not how far the beam reaches — so the cone length is a constant, not derived
/// from link physics.
pub const BEAM_KM: f32 = 3.0;

#[derive(Component)]
pub struct RadarCone {
    pub drone_entity: Entity,
    /// Slot in the owning drone's antenna array.
    pub antenna_index: usize,
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
/// `heading_deg` is the owner's heading (0.0 for a base) — antenna.azimuth_deg
/// is drone-relative, so it must be turned into a world direction here.
/// ConeAnchor::Tip puts origin at tip; -Y extends to base, so rotate -Y → dir.
pub fn cone_transform_for(antenna: &Antenna, heading_deg: f32, drone_pos: Vec3) -> Transform {
    let dir = radar_direction(antenna.world_azimuth_deg(heading_deg), antenna.elevation_deg);
    Transform {
        translation: drone_pos,
        rotation: Quat::from_rotation_arc(Vec3::NEG_Y, dir),
        ..default()
    }
}

/// Keep every antenna beam drawn, on every node, all the time.
///
/// The beams *are* the mesh topology — a live link is two of them meeting — so
/// hiding them until their owner is selected hid the one thing worth watching.
/// Selection now only moves the camera and the info popup.
pub fn sync_radar_visibility(
    // Radar cones are separate entities, so this filter proves they cannot
    // overlap the mutable cone-transform query below.
    drones: Query<(&Transform, &Drone, &DroneKinematics), Without<RadarCone>>,
    // The base owns cones too, and its five antennas are re-aimed every frame
    // by `detect_base_links_and_send_headers`. Without this query the cones it
    // owns never matched the query above (a base has no `Drone`), so they sat
    // frozen at their spawn-time sector bearings instead of showing where the
    // antennas are actually pointing.
    bases: Query<(&Transform, &Base), Without<RadarCone>>,
    mut cones: Query<(&RadarCone, &mut Transform, &mut Visibility)>,
) {
    for (cone, mut transform, mut vis) in &mut cones {
        *vis = Visibility::Visible;

        // The cone is a separate render entity, so it must be refreshed from
        // its owner every frame rather than remaining at the launch point.
        if let Ok((drone_transform, drone, kin)) = drones.get(cone.drone_entity) {
            if let Some(antenna) = drone.antennas.get(cone.antenna_index) {
                *transform =
                    cone_transform_for(antenna, kin.heading_deg, drone_transform.translation);
            }
        } else if let Ok((base_transform, base)) = bases.get(cone.drone_entity)
            && let Some(antenna) = base.antennas.get(cone.antenna_index)
        {
            // A base has no heading — its antenna azimuths are world-frame.
            *transform = cone_transform_for(antenna, 0.0, base_transform.translation);
        }
    }
}
