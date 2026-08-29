use bevy::prelude::*;

use crate::AppState;

pub const AREA_SIZE_KM: f32 = 20.0;

#[derive(Resource, Clone, Debug)]
pub struct ScenarioArea {
    pub name: &'static str,
    pub latitude: f64,
    pub longitude: f64,
    pub size_km: f32,
}

impl Default for ScenarioArea {
    fn default() -> Self {
        STOCKHOLM.into()
    }
}

impl ScenarioArea {
    /// STAC searches use WGS84 longitude/latitude bounds.
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn wgs84_bbox(&self) -> [f64; 4] {
        let half_km = self.size_km as f64 * 0.5;
        let lat_delta = half_km / 110.574;
        let lon_delta = half_km / (111.320 * self.latitude.to_radians().cos());
        [
            self.longitude - lon_delta,
            self.latitude - lat_delta,
            self.longitude + lon_delta,
            self.latitude + lat_delta,
        ]
    }
}

#[derive(Clone, Copy)]
struct AreaPreset {
    name: &'static str,
    latitude: f64,
    longitude: f64,
}

impl From<AreaPreset> for ScenarioArea {
    fn from(value: AreaPreset) -> Self {
        Self {
            name: value.name,
            latitude: value.latitude,
            longitude: value.longitude,
            size_km: AREA_SIZE_KM,
        }
    }
}

const KIRUNA: AreaPreset = AreaPreset {
    name: "Kiruna",
    latitude: 67.8558,
    longitude: 20.2253,
};
const UMEA: AreaPreset = AreaPreset {
    name: "Umea",
    latitude: 63.8258,
    longitude: 20.2630,
};
const OSTERSUND: AreaPreset = AreaPreset {
    name: "Ostersund",
    latitude: 63.1792,
    longitude: 14.6357,
};
const STOCKHOLM: AreaPreset = AreaPreset {
    name: "Stockholm",
    latitude: 59.3293,
    longitude: 18.0686,
};
const GOTEBORG: AreaPreset = AreaPreset {
    name: "Goteborg",
    latitude: 57.7089,
    longitude: 11.9746,
};
const MALMO: AreaPreset = AreaPreset {
    name: "Malmo",
    latitude: 55.6050,
    longitude: 13.0038,
};

#[derive(Component)]
pub(crate) struct AreaSelectionRoot;

#[derive(Component)]
pub(crate) struct AreaChoice(AreaPreset);

#[derive(Component)]
pub(crate) struct GenerateTerrain;

#[derive(Component)]
pub(crate) struct SelectionLabel;

pub fn setup(
    mut commands: Commands,
    area: Res<ScenarioArea>,
    theme: Res<crate::theme::Theme>,
    load_error: Option<Res<crate::terrain::TerrainLoadError>>,
) {
    let p = theme.palette();
    commands
        .spawn((
            Node {
                width: Val::Percent(100.0),
                height: Val::Percent(100.0),
                justify_content: JustifyContent::Center,
                align_items: AlignItems::Center,
                column_gap: Val::Px(48.0),
                ..default()
            },
            BackgroundColor(p.bg),
            AreaSelectionRoot,
        ))
        .with_children(|root| {
            root.spawn((
                Node {
                    width: Val::Px(280.0),
                    height: Val::Px(560.0),
                    position_type: PositionType::Relative,
                    border: UiRect::all(Val::Px(2.0)),
                    border_radius: BorderRadius::all(Val::Px(80.0)),
                    ..default()
                },
                BackgroundColor(p.surface),
                BorderColor::all(p.accent),
            ))
            .with_children(|map| {
                spawn_area_button(map, KIRUNA, 70.0, 28.0, p.bg, p.text);
                spawn_area_button(map, UMEA, 145.0, 190.0, p.bg, p.text);
                spawn_area_button(map, OSTERSUND, 35.0, 250.0, p.bg, p.text);
                spawn_area_button(map, STOCKHOLM, 160.0, 345.0, p.bg, p.text);
                spawn_area_button(map, GOTEBORG, 35.0, 405.0, p.bg, p.text);
                spawn_area_button(map, MALMO, 90.0, 495.0, p.bg, p.text);
            });

            root.spawn(Node {
                width: Val::Px(430.0),
                flex_direction: FlexDirection::Column,
                row_gap: Val::Px(18.0),
                ..default()
            })
            .with_children(|panel| {
                panel.spawn((
                    Text::new("Select a simulation area"),
                    TextFont { font_size: FontSize::Px(34.0), ..default() },
                    TextColor(p.text),
                ));
                panel.spawn((
                    Text::new("Choose a point in Sweden. A 20 x 20 km terrain will be generated around it."),
                    TextFont { font_size: FontSize::Px(17.0), ..default() },
                    TextColor(p.subtext),
                ));
                panel.spawn((
                    Text::new(selection_text(&area)),
                    TextFont { font_size: FontSize::Px(20.0), ..default() },
                    TextColor(p.accent),
                    SelectionLabel,
                ));
                panel
                    .spawn((
                        Button,
                        Node {
                            width: Val::Px(230.0),
                            height: Val::Px(48.0),
                            justify_content: JustifyContent::Center,
                            align_items: AlignItems::Center,
                            border_radius: BorderRadius::all(Val::Px(7.0)),
                            ..default()
                        },
                        BackgroundColor(p.accent),
                        GenerateTerrain,
                    ))
                    .with_child((
                        Text::new("Generate terrain"),
                        TextFont { font_size: FontSize::Px(18.0), ..default() },
                        TextColor(p.bg),
                    ));
                panel.spawn((
                    Text::new("Terrain source: Lantmateriet (local)"),
                    TextFont { font_size: FontSize::Px(14.0), ..default() },
                    TextColor(p.subtext),
                ));
                if let Some(error) = load_error.as_ref() {
                    panel.spawn((
                        Text::new(format!("Last attempt failed: {}", error.0)),
                        TextFont { font_size: FontSize::Px(14.0), ..default() },
                        TextColor(p.danger),
                    ));
                }
            });
        });
}

fn spawn_area_button(
    parent: &mut ChildSpawnerCommands,
    preset: AreaPreset,
    left: f32,
    top: f32,
    fill: Color,
    text: Color,
) {
    parent
        .spawn((
            Button,
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(left),
                top: Val::Px(top),
                padding: UiRect::axes(Val::Px(9.0), Val::Px(6.0)),
                border_radius: BorderRadius::all(Val::Px(12.0)),
                ..default()
            },
            BackgroundColor(fill),
            AreaChoice(preset),
        ))
        .with_child((
            Text::new(preset.name),
            TextFont {
                font_size: FontSize::Px(14.0),
                ..default()
            },
            TextColor(text),
        ));
}

pub fn interactions(
    mut commands: Commands,
    choices: Query<(&Interaction, &AreaChoice), Changed<Interaction>>,
    generate: Query<&Interaction, (Changed<Interaction>, With<GenerateTerrain>)>,
    mut area: ResMut<ScenarioArea>,
    mut labels: Query<&mut Text, With<SelectionLabel>>,
    mut next_state: ResMut<NextState<AppState>>,
) {
    for (interaction, choice) in &choices {
        if *interaction == Interaction::Pressed {
            *area = choice.0.into();
            for mut label in &mut labels {
                **label = selection_text(&area);
            }
        }
    }

    if generate
        .iter()
        .any(|interaction| *interaction == Interaction::Pressed)
    {
        commands.remove_resource::<crate::terrain::TerrainLoadError>();
        next_state.set(AppState::LoadingTerrain);
    }
}

pub fn cleanup(mut commands: Commands, roots: Query<Entity, With<AreaSelectionRoot>>) {
    for entity in &roots {
        commands.entity(entity).despawn();
    }
}

fn selection_text(area: &ScenarioArea) -> String {
    format!(
        "{}  |  {:.4} N, {:.4} E  |  {:.0} km square",
        area.name, area.latitude, area.longitude, area.size_km
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bbox_contains_selected_center() {
        let area = ScenarioArea::default();
        let [west, south, east, north] = area.wgs84_bbox();
        assert!(west < area.longitude && area.longitude < east);
        assert!(south < area.latitude && area.latitude < north);
    }

    #[test]
    fn bbox_represents_requested_square_size() {
        let area = ScenarioArea::default();
        let [west, south, east, north] = area.wgs84_bbox();
        let north_south_km = (north - south) * 110.574;
        let east_west_km =
            (east - west) * 111.320 * area.latitude.to_radians().cos();

        assert!((north_south_km - area.size_km as f64).abs() < 0.001);
        assert!((east_west_km - area.size_km as f64).abs() < 0.001);
    }
}
