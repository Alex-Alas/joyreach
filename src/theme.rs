//! Single source of truth for the visual tokens in `DESIGN.md` §3 and the HUD fonts.
//!
//! Rules that matter here:
//! - Only the sleeves are saturated. `JOY_LEFT` / `JOY_RIGHT` must never appear on world props.
//! - Props use mustard / sage / plum / stone / charcoal and nothing else.

use bevy::prelude::*;

// --- Hand colours (the only saturated things in the world) -------------------
pub const JOY_LEFT: Color = Color::srgb_u8(0xFF, 0x3C, 0x28);
pub const JOY_RIGHT: Color = Color::srgb_u8(0x0A, 0xB9, 0xE6);

// --- World tokens ------------------------------------------------------------
/// Wooden deck floor (L ≈ 40 %).
pub const DECK: Color = Color::srgb_u8(0x7A, 0x5A, 0x3E);
/// Slightly lighter / darker planks, same wood family.
pub const DECK_LIGHT: Color = Color::srgb_u8(0x88, 0x66, 0x48);
pub const DECK_DARK: Color = Color::srgb_u8(0x6C, 0x50, 0x37);
/// Working table: dark wood, the backdrop that makes the gloves readable.
pub const WALNUT: Color = Color::srgb_u8(0x4E, 0x36, 0x27);
/// Low parapets and planters (mint wall of the reference video).
pub const CELADON: Color = Color::srgb_u8(0xC6, 0xD7, 0xCB);
/// Door / window frames, HUD plates.
pub const CREAM: Color = Color::srgb_u8(0xE9, 0xE1, 0xCF);
/// HUD text and panel bezels. Warm, not pure black.
pub const INK: Color = Color::srgb_u8(0x2B, 0x2A, 0x28);
/// Matte off-white glove.
pub const GLOVE: Color = Color::srgb_u8(0xEE, 0xE9, 0xDF);

// --- Grabbable prop colours --------------------------------------------------
pub const MUSTARD: Color = Color::srgb_u8(0xD9, 0xA1, 0x3B);
pub const SAGE: Color = Color::srgb_u8(0x7E, 0x9F, 0x78);
pub const PLUM: Color = Color::srgb_u8(0x7A, 0x5C, 0x85);
/// Mid warm stone. Replaced "chalk" (#EDE8DD), which had the glove's value and read as a hand.
pub const STONE: Color = Color::srgb_u8(0xA8, 0x9F, 0x8E);
pub const CHARCOAL: Color = Color::srgb_u8(0x3B, 0x3C, 0x3D);

// --- HUD ---------------------------------------------------------------------
pub const FONT_MEDIUM: &str = "fonts/MPLUSRounded1c-Medium.ttf";
pub const FONT_BOLD: &str = "fonts/MPLUSRounded1c-Bold.ttf";

/// Cream at 85 % for the HUD plates (DESIGN §10).
pub fn plate() -> Color {
    Color::from(Srgba::from(CREAM).with_alpha(0.85))
}

/// Lower a colour's saturation, used for the sleeve of a disconnected hand.
pub fn desaturate(color: Color, amount: f32) -> Color {
    let mut hsla = Hsla::from(Srgba::from(color));
    hsla.saturation = (hsla.saturation * (1.0 - amount)).clamp(0.0, 1.0);
    Color::from(hsla)
}
