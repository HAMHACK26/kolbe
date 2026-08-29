//! Catppuccin theming — Mocha (dark) / Latte (light) with a runtime toggle.
//!
//! Two mechanisms consume the palette. 3D entities carry a [`ThemeRole`] and
//! get their shared material recolored by [`apply_theme`]. UI nodes carry
//! [`UiFill`] / [`UiStroke`] / [`UiInk`] naming a [`Slot`], and
//! [`apply_ui_slots`] repaints all of them in one pass — so a screen that
//! spawns its own chrome only has to name the slot each node belongs to
//! instead of growing another query in `apply_theme`.

use bevy::prelude::*;

use crate::{
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

#[derive(Resource, Default)]
pub struct Theme {
    pub dark: bool,
}

/// Named Catppuccin colors used across the app.
///
/// Grouped by what a color *means* rather than what it looks like, so the
/// Mocha/Latte swap below stays one decision per role instead of one per
/// widget.
pub struct Palette {
    // ── Chassis ────────────────────────────────────────────────────────────
    pub bg: Color,       // window clear color (base)
    pub panel: Color,    // an instrument panel sitting on `bg` (surface0)
    pub raised: Color,   // a control sitting on a panel (surface1)
    pub line: Color,     // hairline rules and control borders (overlay0)
    pub surface: Color,  // popup background (surface2)

    // ── Type ───────────────────────────────────────────────────────────────
    pub text: Color,     // text
    pub subtext: Color,  // dimmed body text (subtext0)

    // ── Meaning ────────────────────────────────────────────────────────────
    pub accent: Color,   // blue — table header / highlights / live data
    /// Yellow. The operator's own marks: the selected area, the primary
    /// action, the ground station. Shares its value with `base` on purpose —
    /// the square on the map and the station inside it are one idea.
    pub signal: Color,
    pub danger: Color,   // red — errors

    // ── Scene ──────────────────────────────────────────────────────────────
    pub ground: Color,   // green
    pub water: Color,    // sea-level reference plane
    pub drone: Color,    // red
    pub drone_cone: Color, // sapphire
    pub base: Color,     // yellow
    pub grid: Color,     // overlay0
    pub moon: Color,     // yellow
}

/// A palette entry, addressable at runtime so a UI node can name its color
/// instead of carrying one. See [`UiFill`], [`UiStroke`], [`UiInk`].
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Slot {
    Bg,
    Panel,
    Raised,
    Line,
    Text,
    Subtext,
    Accent,
    Signal,
    Danger,
}

impl Palette {
    pub fn slot(&self, slot: Slot) -> Color {
        match slot {
            Slot::Bg => self.bg,
            Slot::Panel => self.panel,
            Slot::Raised => self.raised,
            Slot::Line => self.line,
            Slot::Text => self.text,
            Slot::Subtext => self.subtext,
            Slot::Accent => self.accent,
            Slot::Signal => self.signal,
            Slot::Danger => self.danger,
        }
    }
}

impl Theme {
    pub fn palette(&self) -> Palette {
        if self.dark {
            Palette {
                bg: Color::srgb_u8(0x1e, 0x1e, 0x2e),
                panel: Color::srgb_u8(0x31, 0x32, 0x44), // surface0
                raised: Color::srgb_u8(0x45, 0x47, 0x5a), // surface1
                line: Color::srgb_u8(0x6c, 0x70, 0x86),  // overlay0
                surface: Color::srgb_u8(0x58, 0x5b, 0x70), // surface2 — popup, distinct
                text: Color::srgb_u8(0xcd, 0xd6, 0xf4),
                subtext: Color::srgb_u8(0xa6, 0xad, 0xc8),
                accent: Color::srgb_u8(0x89, 0xb4, 0xfa),
                signal: Color::srgb_u8(0xf9, 0xe2, 0xaf),
                danger: Color::srgb_u8(0xf3, 0x8b, 0xa8),
                ground: Color::srgb_u8(0x45, 0x47, 0x5a), // surface1 — gray
                water: Color::srgb_u8(0x74, 0xc7, 0xec),  // sapphire
                drone: Color::srgb_u8(0xf3, 0x8b, 0xa8),
                drone_cone: Color::srgb_u8(0x74, 0xc7, 0xec),
                base: Color::srgb_u8(0xf9, 0xe2, 0xaf),
                grid: Color::srgb_u8(0x6c, 0x70, 0x86),
                moon: Color::srgb_u8(0xf9, 0xe2, 0xaf),
            }
        } else {
            Palette {
                bg: Color::srgb_u8(0xef, 0xf1, 0xf5),
                panel: Color::srgb_u8(0xcc, 0xd0, 0xda), // surface0
                raised: Color::srgb_u8(0xbc, 0xc0, 0xcc), // surface1
                line: Color::srgb_u8(0x9c, 0xa0, 0xb0),  // overlay0
                surface: Color::srgb_u8(0xac, 0xb0, 0xbe), // surface2 — popup, distinct
                text: Color::srgb_u8(0x4c, 0x4f, 0x69),
                subtext: Color::srgb_u8(0x6c, 0x6f, 0x85),
                accent: Color::srgb_u8(0x1e, 0x66, 0xf5),
                signal: Color::srgb_u8(0xdf, 0x8e, 0x1d),
                danger: Color::srgb_u8(0xd2, 0x0f, 0x39),
                ground: Color::srgb_u8(0xbc, 0xc0, 0xcc), // surface1 — gray
                water: Color::srgb_u8(0x20, 0x9f, 0xb5),  // sapphire
                drone: Color::srgb_u8(0xd2, 0x0f, 0x39),
                drone_cone: Color::srgb_u8(0x20, 0x9f, 0xb5),
                base: Color::srgb_u8(0xdf, 0x8e, 0x1d),
                grid: Color::srgb_u8(0x9c, 0xa0, 0xb0),
                moon: Color::srgb_u8(0xdf, 0x8e, 0x1d),
            }
        }
    }
}

/// Paints a node's background from `slot`, at `alpha` of its full opacity.
#[derive(Component, Clone, Copy)]
pub struct UiFill(pub Slot, pub f32);

/// Paints a node's border from `slot`. All four sides are colored; which of
/// them actually draws is decided by `Node::border`, so a bottom-only rule is
/// a one-sided `border` width, not a one-sided color.
#[derive(Component, Clone, Copy)]
pub struct UiStroke(pub Slot, pub f32);

/// Paints text from `slot`.
#[derive(Component, Clone, Copy)]
pub struct UiInk(pub Slot, pub f32);

impl UiFill {
    pub fn new(slot: Slot) -> Self {
        Self(slot, 1.0)
    }
}
impl UiStroke {
    pub fn new(slot: Slot) -> Self {
        Self(slot, 1.0)
    }
}
impl UiInk {
    pub fn new(slot: Slot) -> Self {
        Self(slot, 1.0)
    }
}

/// Repaint every slot-tagged UI node on a theme change.
///
/// Deliberately one system over the whole tagged set rather than a query per
/// widget: screens that spawn their own chrome (the area picker, chiefly)
/// would otherwise have to add a parameter here for every new control, and
/// `apply_theme` already shows where that ends up. Entities spawned *after* a
/// switch are colored correctly at spawn by whoever spawns them, so this only
/// has to handle the switch itself.
pub fn apply_ui_slots(
    theme: Res<Theme>,
    mut fills: Query<(&UiFill, &mut BackgroundColor)>,
    mut strokes: Query<(&UiStroke, &mut BorderColor)>,
    mut inks: Query<(&UiInk, &mut TextColor)>,
) {
    if !theme.is_changed() {
        return;
    }
    let p = theme.palette();

    for (fill, mut bg) in &mut fills {
        bg.0 = p.slot(fill.0).with_alpha(fill.1);
    }
    for (stroke, mut border) in &mut strokes {
        border.set_all(p.slot(stroke.0).with_alpha(stroke.1));
    }
    for (ink, mut color) in &mut inks {
        color.0 = p.slot(ink.0).with_alpha(ink.1);
    }
}

/// Emissive glow = linear color scaled by `k`, alpha 0.
fn glow(c: Color, k: f32) -> LinearRgba {
    let l = c.to_linear();
    LinearRgba::new(l.red * k, l.green * k, l.blue * k, 0.0)
}

/// Recolor the 3D scene and the sim-overlay chrome whenever `Theme` changes.
///
/// UI nodes that carry a [`Slot`] tag are *not* handled here — see
/// [`apply_ui_slots`]. What is left is the material-driven scene plus the two
/// overlay widgets that predate the slot system and need special handling
/// (the popup wants a translucent fill; the moon swaps visibility, not color).
#[allow(clippy::type_complexity)] // Queries are distinct Bevy system inputs.
pub fn apply_theme(
    theme: Res<Theme>,
    mut clear: ResMut<ClearColor>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    roles: Query<(&MeshMaterial3d<StandardMaterial>, &ThemeRole)>,
    mut popup_bg: Query<&mut BackgroundColor, (With<InfoPopup>, Without<MoonButton>, Without<MoonCrescent>)>,
    mut title: Query<&mut TextColor, With<InfoPopupTitle>>,
    mut moon_btn: Query<&mut BackgroundColor, (With<MoonButton>, Without<InfoPopup>, Without<MoonCrescent>)>,
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
