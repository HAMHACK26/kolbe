use bevy::{prelude::*, window::PrimaryWindow};

use crate::drone::{Drone, DroneType, SelectedDrone};

#[derive(Component)]
pub struct InfoPopup;

#[derive(Component)]
pub struct InfoPopupText;

pub fn update_popup_position(
    selected: Res<SelectedDrone>,
    drones: Query<(&GlobalTransform, &Drone)>,
    camera_q: Query<(&Camera, &GlobalTransform), With<Camera3d>>,
    _windows: Query<&Window, With<PrimaryWindow>>,
    mut popup_q: Query<(&mut Node, &mut Visibility), With<InfoPopup>>,
    mut text_q: Query<&mut Text, With<InfoPopupText>>,
) {
    let Ok((mut node, mut vis)) = popup_q.single_mut() else { return };
    let Ok(mut text) = text_q.single_mut() else { return };

    let Some(entity) = selected.0 else {
        *vis = Visibility::Hidden;
        return;
    };
    let Ok((gt, drone)) = drones.get(entity) else {
        *vis = Visibility::Hidden;
        return;
    };
    let Ok((camera, cam_gt)) = camera_q.single() else { return };

    match camera.world_to_viewport(cam_gt, gt.translation()) {
        Ok(screen_pos) => {
            *vis = Visibility::Visible;
            node.left = Val::Px(screen_pos.x + 14.0);
            node.top = Val::Px(screen_pos.y - 10.0);
            **text = build_popup_text(drone);
        }
        Err(_) => {
            *vis = Visibility::Hidden;
        }
    }
}

fn build_popup_text(drone: &Drone) -> String {
    let type_str = match drone.drone_type {
        DroneType::Attack => "Attack",
        DroneType::Node => "Node",
    };
    let mut s = format!("{}  [{}]", drone.id, type_str);
    for (i, ant) in drone.antennas.iter().enumerate() {
        let range = ant.max_range_km();
        s.push_str(&format!(
            "\nAnt {}: Az {:.0}°  El {:.0}°\n  G {:.1} dBi  θ₃dB {:.1}°  P_tx {} dBm\n  f {} MHz  Range {:.2} km  RSSI@edge {:.1} dBm",
            i + 1,
            ant.azimuth_deg,
            ant.elevation_deg,
            ant.g_peak_dbi,
            ant.theta_3db_deg,
            ant.p_tx_dbm as i32,
            ant.frequency_mhz as i32,
            range,
            ant.rssi_dbm(0.0, 0.0, range),
        ));
    }
    s
}
