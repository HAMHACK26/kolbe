//! Catppuccin theming — Mocha (dark) / Latte (light) with a runtime toggle.
//!
//! Entities carry a `ThemeRole` marker; `apply_theme` recolors their shared
//! material whenever the `Theme` resource changes. UI colors (popup, moon)
//! are driven the same way.

use bevy::prelude::*;

use crate::{
    ui::{InfoPopup, InfoPopupTitle},
};

/// Which conceptual color an entity's material should take.
#[derive(Component, Clone, Copy)]
pub enum ThemeRole {
    Ground,
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
    pub surface: Color,  // popup background (surface0)
    pub text: Color,     // text
    pub accent: Color,   // blue — table header
    pub drone: Color,    // red
    pub drone_cone: Color, // sapphire
    pub base: Color,     // yellow
    pub grid: Color,     // overlay0
    pub moon: Color,     // yellow
}

impl Theme {
    pub fn palette(&self) -> Palette {
        if self.dark {
            Palette {
                bg: Color::srgb_u8(0x1e, 0x1e, 0x2e),
                ground: Color::srgb_u8(0x45, 0x47, 0x5a), // surface1 — gray
                surface: Color::srgb_u8(0x58, 0x5b, 0x70), // surface2 — popup, distinct
                text: Color::srgb_u8(0xcd, 0xd6, 0xf4),
                accent: Color::srgb_u8(0x89, 0xb4, 0xfa),
                drone: Color::srgb_u8(0xf3, 0x8b, 0xa8),
                drone_cone: Color::srgb_u8(0x74, 0xc7, 0xec),
                base: Color::srgb_u8(0xf9, 0xe2, 0xaf),
                grid: Color::srgb_u8(0x6c, 0x70, 0x86),
                moon: Color::srgb_u8(0xf9, 0xe2, 0xaf),
            }
        } else {
            Palette {
                bg: Color::srgb_u8(0xef, 0xf1, 0xf5),
                ground: Color::srgb_u8(0xbc, 0xc0, 0xcc), // surface1 — gray
                surface: Color::srgb_u8(0xac, 0xb0, 0xbe), // surface2 — popup, distinct
                text: Color::srgb_u8(0x4c, 0x4f, 0x69),
                accent: Color::srgb_u8(0x1e, 0x66, 0xf5),
                drone: Color::srgb_u8(0xd2, 0x0f, 0x39),
                drone_cone: Color::srgb_u8(0x20, 0x9f, 0xb5),
                base: Color::srgb_u8(0xdf, 0x8e, 0x1d),
                grid: Color::srgb_u8(0x9c, 0xa0, 0xb0),
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
    mut popup_bg: Query<&mut BackgroundColor, (With<InfoPopup>, Without<MoonButton>)>,
    mut title: Query<&mut TextColor, With<InfoPopupTitle>>,
    mut moon_btn: Query<&mut BackgroundColor, (With<MoonButton>, Without<InfoPopup>)>,
    mut crescent: Query<&mut Visibility, (With<MoonCrescent>, Without<SunRays>)>,
    mut rays: Query<&mut Visibility, (With<SunRays>, Without<MoonCrescent>)>,
    mut crescent_bg: Query<&mut BackgroundColor, (With<MoonCrescent>, Without<MoonButton>, Without<InfoPopup>)>,
) {
    if !theme.is_changed() {
        return;
    }
    let p = theme.palette();

    clear.0 = p.bg;

    for (mat_handle, role) in &roles {
        let Some(mut mat) = materials.get_mut(&mat_handle.0) else { continue };
        match role {
            ThemeRole::Ground => {
                mat.base_color = p.ground;
                mat.emissive = LinearRgba::BLACK;
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

/// Spawn the round moon button (top-right) with a crescent cut-out.
pub fn spawn_moon(commands: &mut Commands, theme: &Theme) {
    let p = theme.palette();
    commands
        .spawn((
            Button,
            Node {
                position_type: PositionType::Absolute,
                right: Val::Px(16.0),
                top: Val::Px(16.0),
                width: Val::Px(34.0),
                height: Val::Px(34.0),
                border_radius: BorderRadius::all(Val::Percent(50.0)),
                overflow: Overflow::clip(),
                ..default()
            },
            BackgroundColor(p.moon),
            MoonButton,
        ))
        .with_children(|b| {
            // Offset circle carves the crescent (matches bg in dark, moon in light).
            b.spawn((
                Node {
                    position_type: PositionType::Absolute,
                    right: Val::Px(-8.0),
                    top: Val::Px(-4.0),
                    width: Val::Px(30.0),
                    height: Val::Px(30.0),
                    border_radius: BorderRadius::all(Val::Percent(50.0)),
                    ..default()
                },
                BackgroundColor(if theme.dark { p.bg } else { p.moon }),
                Pickable::IGNORE,
                MoonCrescent,
            ));
        });

    // Sun rays — a ring of dots around the disc, sibling so they aren't clipped.
    // Button center sits 33px from the top/right edges; container is centered on it.
    let ray = 25.0_f32;
    let dot = 6.0_f32;
    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                right: Val::Px(3.0),
                top: Val::Px(3.0),
                width: Val::Px(60.0),
                height: Val::Px(60.0),
                ..default()
            },
            Pickable::IGNORE,
            Visibility::Visible,
            SunRays,
        ))
        .with_children(|r| {
            for k in 0..8 {
                let a = k as f32 * std::f32::consts::TAU / 8.0;
                let cx = 30.0 + ray * a.cos() - dot / 2.0;
                let cy = 30.0 + ray * a.sin() - dot / 2.0;
                r.spawn((
                    Node {
                        position_type: PositionType::Absolute,
                        left: Val::Px(cx),
                        top: Val::Px(cy),
                        width: Val::Px(dot),
                        height: Val::Px(dot),
                        border_radius: BorderRadius::all(Val::Percent(50.0)),
                        ..default()
                    },
                    BackgroundColor(p.moon),
                    Pickable::IGNORE,
                ));
            }
        });
}
