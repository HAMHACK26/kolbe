//! Catppuccin theming — Mocha (dark) / Latte (light) with a runtime toggle.
//!
//! Entities carry a `ThemeRole` marker; `apply_theme` recolors their shared
//! material whenever the `Theme` resource changes. UI colors (popup, moon)
//! are driven the same way.

use bevy::prelude::*;

use crate::{
    area::{AreaBg, BodyText, HeadingText, SourceText},
    terrain::{LoadingBarFill, LoadingHeading, LoadingRoot, LoadingStatus, LoadingTrack},
    ui::{InfoPopup, InfoPopupTitle},
};

/// Which conceptual color an entity's material should take.
#[derive(Component, Clone, Copy)]
pub enum ThemeRole {
    Ground,
    Water,
    Drone,
    DroneCone,
    BaseMarker,
    BaseCone,
}

/// Moon toggle button + its crescent cut-out.
#[derive(Component)]
pub struct MoonButton;
#[derive(Component)]
pub struct MoonCrescent;
/// Container for the sun rays (shown in light mode only).
#[derive(Component)]
pub struct SunRays;

#[derive(Resource)]
pub struct Theme {
    pub dark: bool,
}

impl Default for Theme {
    fn default() -> Self {
        Self { dark: false }
    }
}

/// Named Catppuccin colors used across the app.
pub struct Palette {
    pub bg: Color,       // window clear color (base)
    pub ground: Color,   // green
    pub water: Color,    // sea-level reference plane
    pub surface: Color,  // popup background (surface0)
    pub text: Color,     // text
    pub subtext: Color,  // dimmed body text (subtext0)
    pub accent: Color,   // blue — table header / highlights
    pub drone: Color,    // red
    pub drone_cone: Color, // sapphire
    pub base: Color,     // yellow
    pub grid: Color,     // overlay0
    pub danger: Color,   // red — errors
    pub moon: Color,     // yellow
}

impl Theme {
    pub fn palette(&self) -> Palette {
        if self.dark {
            Palette {
                bg: Color::srgb_u8(0x1e, 0x1e, 0x2e),
                ground: Color::srgb_u8(0x45, 0x47, 0x5a), // surface1 — gray
                water: Color::srgb_u8(0x74, 0xc7, 0xec),  // sapphire
                surface: Color::srgb_u8(0x58, 0x5b, 0x70), // surface2 — popup, distinct
                text: Color::srgb_u8(0xcd, 0xd6, 0xf4),
                subtext: Color::srgb_u8(0xa6, 0xad, 0xc8),
                accent: Color::srgb_u8(0x89, 0xb4, 0xfa),
                drone: Color::srgb_u8(0xf3, 0x8b, 0xa8),
                drone_cone: Color::srgb_u8(0x74, 0xc7, 0xec),
                base: Color::srgb_u8(0xf9, 0xe2, 0xaf),
                grid: Color::srgb_u8(0x6c, 0x70, 0x86),
                danger: Color::srgb_u8(0xf3, 0x8b, 0xa8),
                moon: Color::srgb_u8(0xf9, 0xe2, 0xaf),
            }
        } else {
            Palette {
                bg: Color::srgb_u8(0xef, 0xf1, 0xf5),
                ground: Color::srgb_u8(0xbc, 0xc0, 0xcc), // surface1 — gray
                water: Color::srgb_u8(0x20, 0x9f, 0xb5),  // sapphire
                surface: Color::srgb_u8(0xac, 0xb0, 0xbe), // surface2 — popup, distinct
                text: Color::srgb_u8(0x4c, 0x4f, 0x69),
                subtext: Color::srgb_u8(0x6c, 0x6f, 0x85),
                accent: Color::srgb_u8(0x1e, 0x66, 0xf5),
                drone: Color::srgb_u8(0xd2, 0x0f, 0x39),
                drone_cone: Color::srgb_u8(0x20, 0x9f, 0xb5),
                base: Color::srgb_u8(0xdf, 0x8e, 0x1d),
                grid: Color::srgb_u8(0x9c, 0xa0, 0xb0),
                danger: Color::srgb_u8(0xd2, 0x0f, 0x39),
                moon: Color::srgb_u8(0xdf, 0x8e, 0x1d),
            }
        }
    }
}

/// Emissive glow = linear color scaled by `k`, alpha 0.
fn glow(c: Color, k: f32) -> LinearRgba {
    let l = c.to_linear();
    LinearRgba::new(l.red * k, l.green * k, l.blue * k, 0.0)
}

/// Recolor everything whenever `Theme` changes (also runs on first frame).
pub fn apply_theme(
    theme: Res<Theme>,
    mut clear: ResMut<ClearColor>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    roles: Query<(&MeshMaterial3d<StandardMaterial>, &ThemeRole)>,
    mut popup_bg: Query<&mut BackgroundColor, (With<InfoPopup>, Without<MoonButton>, Without<MoonCrescent>, Without<AreaBg>)>,
    mut title: Query<&mut TextColor, (With<InfoPopupTitle>, Without<HeadingText>, Without<BodyText>, Without<SourceText>)>,
    mut moon_btn: Query<&mut BackgroundColor, (With<MoonButton>, Without<InfoPopup>, Without<MoonCrescent>, Without<AreaBg>)>,
    mut crescent: Query<&mut Visibility, (With<MoonCrescent>, Without<SunRays>)>,
    mut rays: Query<&mut Visibility, (With<SunRays>, Without<MoonCrescent>)>,
    mut crescent_bg: Query<&mut BackgroundColor, (With<MoonCrescent>, Without<MoonButton>, Without<InfoPopup>, Without<AreaBg>)>,
    area_ui: (
        Query<&mut BackgroundColor, (With<AreaBg>, Without<InfoPopup>, Without<MoonButton>, Without<MoonCrescent>)>,
        Query<&mut TextColor, (With<HeadingText>, Without<InfoPopupTitle>, Without<BodyText>, Without<SourceText>)>,
        Query<&mut TextColor, (With<BodyText>, Without<HeadingText>, Without<InfoPopupTitle>, Without<SourceText>)>,
        Query<&mut TextColor, (With<SourceText>, Without<HeadingText>, Without<BodyText>, Without<InfoPopupTitle>)>,
    ),
) {
    let (mut area_bg, mut headings, mut bodies, mut sources) = area_ui;

    if !theme.is_changed() {
        return;
    }
    let p = theme.palette();

    clear.0 = p.bg;

    for mut bg in &mut area_bg {
        bg.0 = p.bg;
    }
    for mut tc in &mut headings {
        tc.0 = p.text;
    }
    for mut tc in &mut bodies {
        tc.0 = p.text.with_alpha(0.8);
    }
    for mut tc in &mut sources {
        tc.0 = p.text.with_alpha(0.6);
    }

    for (mat_handle, role) in &roles {
        let Some(mut mat) = materials.get_mut(&mat_handle.0) else { continue };
        match role {
            ThemeRole::Ground => {
                mat.base_color = p.ground;
                mat.emissive = LinearRgba::BLACK;
            }
            ThemeRole::Water => {
                mat.base_color = p.water.with_alpha(0.55);
                mat.emissive = glow(p.water, 0.15);
            }
            ThemeRole::Drone => {
                mat.base_color = p.drone;
                mat.emissive = glow(p.drone, 2.0);
            }
            ThemeRole::DroneCone => {
                mat.base_color = p.drone_cone.with_alpha(0.30);
                mat.emissive = glow(p.drone_cone, 0.6);
            }
            ThemeRole::BaseMarker => {
                mat.base_color = p.base;
                mat.emissive = glow(p.base, 1.5);
            }
            ThemeRole::BaseCone => {
                mat.base_color = p.base.with_alpha(0.20);
                mat.emissive = glow(p.base, 0.5);
            }
        }
    }

    if let Ok(mut bg) = popup_bg.single_mut() {
        bg.0 = p.surface.with_alpha(0.92);
    }
    if let Ok(mut tc) = title.single_mut() {
        tc.0 = p.text;
    }
    if let Ok(mut bg) = moon_btn.single_mut() {
        bg.0 = p.moon;
    }
    if let Ok(mut bg) = crescent_bg.single_mut() {
        bg.0 = p.bg;
    }
    // Dark → crescent (moon). Light → rays (sun).
    if let Ok(mut vis) = crescent.single_mut() {
        *vis = if theme.dark { Visibility::Visible } else { Visibility::Hidden };
    }
    if let Ok(mut vis) = rays.single_mut() {
        *vis = if theme.dark { Visibility::Hidden } else { Visibility::Visible };
    }
}

/// Recolors the loading screen. Kept separate from `apply_theme` so its
/// queries never have to prove disjointness against that system's — the
/// two screens' entities never coexist anyway.
pub fn apply_loading_theme(
    theme: Res<Theme>,
    mut root_bg: Query<&mut BackgroundColor, (With<LoadingRoot>, Without<LoadingTrack>, Without<LoadingBarFill>)>,
    mut track_bg: Query<&mut BackgroundColor, (With<LoadingTrack>, Without<LoadingRoot>, Without<LoadingBarFill>)>,
    mut fill_bg: Query<&mut BackgroundColor, (With<LoadingBarFill>, Without<LoadingTrack>, Without<LoadingRoot>)>,
    mut heading: Query<&mut TextColor, (With<LoadingHeading>, Without<LoadingStatus>)>,
    mut status: Query<&mut TextColor, (With<LoadingStatus>, Without<LoadingHeading>)>,
) {
    if !theme.is_changed() {
        return;
    }
    let p = theme.palette();
    for mut bg in &mut root_bg {
        bg.0 = p.bg;
    }
    for mut bg in &mut track_bg {
        bg.0 = p.surface;
    }
    for mut bg in &mut fill_bg {
        bg.0 = p.accent;
    }
    for mut tc in &mut heading {
        tc.0 = p.text;
    }
    for mut tc in &mut status {
        tc.0 = p.accent;
    }
}

/// Toggle the theme when the moon button is pressed.
pub fn moon_toggle(
    interactions: Query<&Interaction, (Changed<Interaction>, With<MoonButton>)>,
    mut theme: ResMut<Theme>,
) {
    for interaction in &interactions {
        if *interaction == Interaction::Pressed {
            theme.dark = !theme.dark;
        }
    }
}

/// Startup system: place the moon toggle.
pub fn setup_moon(mut commands: Commands, theme: Res<Theme>) {
    spawn_moon(&mut commands, &theme);
}

/// Spawn the round day/night toggle (top-right): a small disc that reads as a
/// crescent moon in dark mode and a rayed sun in light mode.
pub fn spawn_moon(commands: &mut Commands, theme: &Theme) {
    let p = theme.palette();

    // Compact geometry. The disc is the sun/moon body; a slightly offset disc of
    // the background color carves the crescent; thin rays ring it for the sun.
    const EDGE: f32 = 14.0; // margin from the top-right corner
    const DISC: f32 = 18.0; // sun/moon body diameter
    const CRESCENT: f32 = 15.0; // carving disc (dark mode)
    const RAY_LEN: f32 = 4.0; // sun ray length
    const RAY_W: f32 = 2.0; // sun ray thickness
    const RAY_GAP: f32 = 2.5; // gap between disc edge and ray

    // Rays live in a box centered on the disc; sized to fit disc + rays + gap.
    let box_size = DISC + 2.0 * (RAY_GAP + RAY_LEN);
    let disc_center_from_edge = EDGE + DISC / 2.0;
    let box_offset = disc_center_from_edge - box_size / 2.0;

    commands
        .spawn((
            Button,
            Node {
                position_type: PositionType::Absolute,
                right: Val::Px(EDGE),
                top: Val::Px(EDGE),
                width: Val::Px(DISC),
                height: Val::Px(DISC),
                border_radius: BorderRadius::all(Val::Percent(50.0)),
                overflow: Overflow::clip(),
                ..default()
            },
            BackgroundColor(p.moon),
            MoonButton,
        ))
        .with_children(|b| {
            // Offset disc carves the crescent (bg color in dark, hidden in light).
            b.spawn((
                Node {
                    position_type: PositionType::Absolute,
                    right: Val::Px(-4.0),
                    top: Val::Px(-2.5),
                    width: Val::Px(CRESCENT),
                    height: Val::Px(CRESCENT),
                    border_radius: BorderRadius::all(Val::Percent(50.0)),
                    ..default()
                },
                BackgroundColor(if theme.dark { p.bg } else { p.moon }),
                Pickable::IGNORE,
                MoonCrescent,
            ));
        });

    // Sun rays — short capsules around the disc, as a sibling so they aren't
    // clipped by the disc's `overflow: clip`.
    let center = box_size / 2.0;
    let radius = DISC / 2.0 + RAY_GAP + RAY_LEN / 2.0;
    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                right: Val::Px(box_offset),
                top: Val::Px(box_offset),
                width: Val::Px(box_size),
                height: Val::Px(box_size),
                ..default()
            },
            Pickable::IGNORE,
            Visibility::Visible,
            SunRays,
        ))
        .with_children(|r| {
            for k in 0..8 {
                let a = k as f32 * std::f32::consts::TAU / 8.0;
                let cx = center + radius * a.cos();
                let cy = center + radius * a.sin();
                r.spawn((
                    Node {
                        position_type: PositionType::Absolute,
                        left: Val::Px(cx - RAY_W / 2.0),
                        top: Val::Px(cy - RAY_LEN / 2.0),
                        width: Val::Px(RAY_W),
                        height: Val::Px(RAY_LEN),
                        border_radius: BorderRadius::all(Val::Percent(50.0)),
                        ..default()
                    },
                    BackgroundColor(p.moon),
                    Pickable::IGNORE,
                ));
            }
        });
}
