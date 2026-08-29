use bevy::{mesh::primitives::ConeAnchor, prelude::*};

use crate::{
    antenna::{Antenna, radar_direction},
    drone::{Drone, SelectedDrone},
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
    mut cones: Query<(&RadarCone, &mut Transform), Without<Drone>>,
    owners: Query<(&Transform, &Drone, &DroneKinematics)>,
) {
    for (cone, mut cone_transform) in &mut cones {
        let Ok((owner_transform, drone, kin)) = owners.get(cone.drone_entity) else {
            continue; // a base, or an owner that has been despawned.
        };
        let Some(antenna) = drone.antennas.get(cone.antenna_index) else {
            continue;
        };
        *cone_transform =
            cone_transform_for(antenna, kin.heading_deg, owner_transform.translation);
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
    drones: Query<(Entity, &Transform, &LinkSet), With<Drone>>,
) {
    let color = theme.palette().drone_cone;
    let nodes: Vec<(Entity, Vec3, &LinkSet)> =
        drones.iter().map(|(e, t, links)| (e, t.translation, links)).collect();
    for (from, to) in mutual_link_segments(&nodes) {
        gizmos.line(from, to, color);
    }
}

/// The undirected segments of [`draw_mesh_links`]: one per pair that appears
/// in *both* drones' link sets, each pair yielded exactly once.
fn mutual_link_segments(nodes: &[(Entity, Vec3, &LinkSet)]) -> Vec<(Vec3, Vec3)> {
    let mut segments = Vec::new();
    for (i, (self_entity, self_pos, links)) in nodes.iter().enumerate() {
        for (peer_entity, peer_pos, peer_links) in &nodes[i + 1..] {
            let mutual = links.connected.contains_key(peer_entity)
                && peer_links.connected.contains_key(self_entity);
            if mutual {
                segments.push((*self_pos, *peer_pos));
            }
        }
    }
    segments
}

#[cfg(test)]
mod tests {
    use super::*;

    fn links_to(peers: &[Entity]) -> LinkSet {
        let mut set = LinkSet::default();
        for &peer in peers {
            set.connected.insert(peer, 0.0);
        }
        set
    }

    /// A line is drawn only for a two-way link. One-sided detection — which is
    /// what a drone still searching for a lock looks like from the other side
    /// — draws nothing.
    #[test]
    fn only_mutual_links_get_a_segment() {
        let (a, b, c) = (Entity::from_raw_u32(1).unwrap(), Entity::from_raw_u32(2).unwrap(), Entity::from_raw_u32(3).unwrap());
        let (a_pos, b_pos, c_pos) =
            (Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0));

        // A↔B is mutual. A→C is one-sided: C is still searching and has not
        // detected A back.
        let a_links = links_to(&[b, c]);
        let b_links = links_to(&[a]);
        let c_links = links_to(&[]);
        let nodes = vec![(a, a_pos, &a_links), (b, b_pos, &b_links), (c, c_pos, &c_links)];

        let segments = mutual_link_segments(&nodes);
        assert_eq!(segments, vec![(a_pos, b_pos)], "only the two-way A↔B link should draw");
    }

    /// A pair is one segment, not two — both drones list each other, but the
    /// line between them is drawn once.
    #[test]
    fn a_mutual_pair_draws_exactly_one_segment() {
        let (a, b) = (Entity::from_raw_u32(1).unwrap(), Entity::from_raw_u32(2).unwrap());
        let (a_links, b_links) = (links_to(&[b]), links_to(&[a]));
        let nodes = vec![
            (a, Vec3::ZERO, &a_links),
            (b, Vec3::new(2.0, 0.0, 0.0), &b_links),
        ];
        assert_eq!(mutual_link_segments(&nodes).len(), 1);
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
