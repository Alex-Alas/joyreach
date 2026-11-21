//! M0 HUD (DESIGN §10): cream plates with rounded corners, ink text in Spanish,
//! left-hand info bottom-left, right-hand info bottom-right, connection state as a
//! minimal capsule silhouette (filled when connected, outline when not).
//! Typography: M PLUS Rounded 1c (OFL, see assets/fonts/OFL.txt).

use bevy::prelude::*;

use crate::arms::HandSide;
use crate::joycon::Hands;
use crate::theme;

pub struct HudPlugin;

impl Plugin for HudPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<HudState>()
            .add_systems(Startup, setup_hud)
            .add_systems(Update, update_hud.after(crate::player::InputSet));
    }
}

#[derive(Resource, Default)]
struct HudState {
    fps: f32,
}

#[derive(Component)]
struct HudText(HudTextKind);

#[derive(Clone, Copy)]
enum HudTextKind {
    Fps,
    Hz(HandSide),
}

#[derive(Component)]
struct HudSilhouette(HandSide);

fn setup_hud(mut commands: Commands, asset_server: Res<AssetServer>) {
    let medium: Handle<Font> = asset_server.load(theme::FONT_MEDIUM);
    let bold: Handle<Font> = asset_server.load(theme::FONT_BOLD);

    let body_font = TextFont {
        font: medium.clone().into(),
        font_size: FontSize::Px(14.0),
        ..default()
    };
    let state_font = TextFont {
        font: bold.clone().into(),
        font_size: FontSize::Px(14.0),
        ..default()
    };

    // --- Frames per second, quiet, top-left ---------------------------------
    commands.spawn((
        Node {
            position_type: PositionType::Absolute,
            top: px(18),
            left: px(18),
            padding: UiRect::axes(px(10), px(5)),
            border_radius: BorderRadius::all(px(6)),
            ..default()
        },
        BackgroundColor(theme::plate()),
    ))
    .with_child((
        Text::new("60 FPS"),
        body_font.clone(),
        TextColor(theme::INK),
        HudText(HudTextKind::Fps),
    ));

    // --- Hand panels --------------------------------------------------------
    commands.spawn((
        Node {
            position_type: PositionType::Absolute,
            left: px(18),
            bottom: px(18),
            padding: UiRect::all(px(12)),
            flex_direction: FlexDirection::Column,
            row_gap: px(7),
            align_items: AlignItems::FlexStart,
            border_radius: BorderRadius::all(px(6)),
            ..default()
        },
        BackgroundColor(theme::plate()),
    ))
    .with_children(|panel| {
        panel.spawn((
            Text::new("Joy-Con L"),
            state_font.clone(),
            TextColor(theme::INK),
        ));
        panel.spawn((
            Node {
                width: px(46),
                height: px(16),
                border: UiRect::all(px(1)),
                border_radius: BorderRadius::all(px(8)),
                ..default()
            },
            BackgroundColor(Color::NONE),
            BorderColor::all(Color::NONE),
            HudSilhouette(HandSide::Left),
        ));
        panel.spawn((
            Text::new("—"),
            body_font.clone(),
            TextColor(theme::INK),
            HudText(HudTextKind::Hz(HandSide::Left)),
        ));
    });

    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                right: px(18),
                bottom: px(18),
                padding: UiRect::all(px(12)),
                flex_direction: FlexDirection::Column,
                row_gap: px(7),
                align_items: AlignItems::FlexEnd,
                border_radius: BorderRadius::all(px(6)),
                ..default()
            },
            BackgroundColor(theme::plate()),
        ))
        .with_children(|panel| {
            panel.spawn((
                Text::new("Joy-Con R"),
                TextFont {
                    font: bold.into(),
                    font_size: FontSize::Px(14.0),
                    ..default()
                },
                TextColor(theme::INK),
                TextLayout::justify(Justify::Right),
            ));
            panel.spawn((
                Node {
                    width: px(46),
                    height: px(16),
                    border: UiRect::all(px(1)),
                    border_radius: BorderRadius::all(px(8)),
                    ..default()
                },
                BackgroundColor(Color::NONE),
                BorderColor::all(Color::NONE),
                HudSilhouette(HandSide::Right),
            ));
            panel.spawn((
                Text::new("—"),
                body_font,
                TextColor(theme::INK),
                TextLayout::justify(Justify::Right),
                HudText(HudTextKind::Hz(HandSide::Right)),
            ));
        });

    // --- Bottom-centre hint --------------------------------------------------
    commands
        .spawn(Node {
            position_type: PositionType::Absolute,
            bottom: px(22),
            left: px(0),
            right: px(0),
            justify_content: JustifyContent::Center,
            ..default()
        })
        .with_child((
            Node {
                padding: UiRect::axes(px(14), px(7)),
                border_radius: BorderRadius::all(px(6)),
                ..default()
            },
            BackgroundColor(theme::plate()),
            children![(
                Text::new("Stick izq: moverse · Stick der: mirar · B: saltar · clic stick izq: agacharse · −/+: recentrar"),
                TextFont {
                    font: asset_server.load(theme::FONT_MEDIUM).into(),
                    font_size: FontSize::Px(14.0),
                    ..default()
                },
                TextColor(theme::INK),
            )],
        ));
}

fn update_hud(
    hands: Res<Hands>,
    time: Res<Time>,
    mut state: ResMut<HudState>,
    mut texts: Query<(&HudText, &mut Text)>,
    mut silhouettes: Query<(&HudSilhouette, &mut BackgroundColor, &mut BorderColor)>,
) {
    let instant = 1.0 / time.delta_secs().max(1e-4);
    state.fps = if state.fps <= 0.0 {
        instant
    } else {
        state.fps * 0.92 + instant * 0.08
    };

    for (kind, mut text) in &mut texts {
        match kind.0 {
            HudTextKind::Fps => text.0 = format!("{:.0} FPS", state.fps),
            HudTextKind::Hz(side) => {
                let input = match side {
                    HandSide::Left => &hands.left,
                    HandSide::Right => &hands.right,
                };
                text.0 = if input.connected && input.imu_hz > 0.5 {
                    format!("{:.0} Hz", input.imu_hz)
                } else {
                    "—".to_string()
                };
            }
        }
    }

    for (silhouette, mut background, mut border) in &mut silhouettes {
        let input = match silhouette.0 {
            HandSide::Left => &hands.left,
            HandSide::Right => &hands.right,
        };
        let color = silhouette.0.color();
        if input.connected {
            background.0 = color;
            *border = BorderColor::all(color);
        } else {
            background.0 = Color::NONE;
            *border = BorderColor::all(color);
        }
    }
}
