use bevy::{mesh::primitives::ConeAnchor, prelude::*};

use crate::{
    antenna::{Antenna, Antennas, radar_direction},
    drone::SelectedDrone,
    factories::movement::DroneKinematics,
    networking::LinkSet,
};

/// Fixed visual beam length (km). The drone knows only its pointing angles —
/// not how far the beam reaches — so the cone length is a constant, not derived
/// from link physics.
pub const BEAM_KM: f32 = 3.0;

#[derive(Component)]
pub struct RadarCone {
    pub drone_entity: Entity,
    /// Which of the owner's antennas this cone draws. Needed because the
    /// antennas re-aim every frame (`crate::tracking`, `crate::seeking`), so
    /// the cone has to be re-derived from its own antenna rather than left at
    /// the angle it spawned with.
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

/// Re-place every drone cone from its antenna's *current* aim and its owner's
/// current position and heading.
///
/// Without this the cones freeze at their spawn angles and quietly stop
/// describing where the drone is actually listening — which is exactly the
/// thing they exist to show, now that the aiming systems slew them every
/// frame. Base cones are left alone: a base neither moves nor re-aims.
pub fn sync_radar_transforms(
    mut cones: Query<(&RadarCone, &mut Transform), Without<Antennas>>,
    owners: Query<(&Transform, &Antennas, Option<&DroneKinematics>)>,
) {
    for (cone, mut cone_transform) in &mut cones {
        let Ok((owner_transform, antennas, kin)) = owners.get(cone.drone_entity) else {
            continue; // owner despawned.
        };
        let Some(antenna) = antennas.0.get(cone.antenna_index) else {
            continue;
        };
        // A base has no airframe and so no heading — its azimuths are already
        // world-frame.
        let heading_deg = kin.map(|k| k.heading_deg).unwrap_or(0.0);
        *cone_transform = cone_transform_for(antenna, heading_deg, owner_transform.translation);
    }
}

/// Draw a line between every pair of drones that has a *live, two-way* link.
///
/// A connection counts only when each drone independently detected the other
/// and is therefore sending it headers — mutual membership in both
/// [`LinkSet`]s. A one-sided detection is not a connection: a drone that only
/// hears its peer, or one still spiral-searching for a lock
/// (`crate::seeking`), has no entry on the far side and gets no line. The
/// picture is therefore exactly the mesh that is actually carrying data.
pub fn draw_mesh_links(
    mut gizmos: Gizmos,
    theme: Res<crate::theme::Theme>,
    nodes: Query<(Entity, &Transform, &LinkSet), With<Antennas>>,
) {
    // One color for every hop, base or drone — the mesh is one system, and the
    // cones the links come out of are drawn in the same yellow.
    let color = theme.palette().base;
    let nodes: Vec<RadioNode> = nodes
        .iter()
        .map(|(entity, transform, links)| RadioNode {
            entity,
            position: transform.translation,
            links,
        })
        .collect();
    for (from, to) in mutual_link_segments(&nodes) {
        gizmos.line(from, to, color);
    }
}

/// One node in the link picture: anything carrying [`Antennas`], drone or base.
struct RadioNode<'a> {
    entity: Entity,
    position: Vec3,
    links: &'a LinkSet,
}

/// The undirected segments of [`draw_mesh_links`]: one per pair that appears
/// in *both* nodes' link sets, each pair yielded exactly once.
fn mutual_link_segments(nodes: &[RadioNode]) -> Vec<(Vec3, Vec3)> {
    let mut segments = Vec::new();
    for (i, node) in nodes.iter().enumerate() {
        for peer in &nodes[i + 1..] {
            let mutual = node.links.connected.contains_key(&peer.entity)
                && peer.links.connected.contains_key(&node.entity);
            if mutual {
                segments.push((node.position, peer.position));
            }
        }
    }
    segments
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
