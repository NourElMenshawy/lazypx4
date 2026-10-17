//! Kitty graphics protocol for the satellite map background. Terminals that
//! speak it (kitty, Ghostty, WezTerm) get the snapshot at full resolution,
//! scaled by the terminal into the grid cells, instead of two half-block
//! pixels per cell.
//!
//! The image is transmitted once per snapshot; each frame only (re)places
//! it. It sits below cells with a non-default background, so the map grid
//! leaves its empty cells at the default background and keeps the darkened
//! box behind every glyph. Screens declare what they want with [`want`]
//! while drawing; [`flush`] after `terminal.draw` applies the difference.

use std::io::Write;
use std::sync::{Arc, Mutex, OnceLock};

use crate::satellite::SatImage;

/// Below cells with a non-default background (kitty: < INT32_MIN / 2).
const Z_UNDER_BG: i64 = -1_073_741_825;
const PLACEMENT_ID: u32 = 1;

/// The image drawn over `cols` x `rows` cells from (`col`, `row`), showing
/// the `src` = [x, y, w, h] pixel rectangle of the snapshot.
#[derive(Clone)]
pub struct Placement {
    pub image: Arc<SatImage>,
    pub col: u16,
    pub row: u16,
    pub cols: u16,
    pub rows: u16,
    pub src: [u32; 4],
}

impl Placement {
    fn key(&self) -> (u32, u16, u16, u16, u16, [u32; 4]) {
        (self.image.id(), self.col, self.row, self.cols, self.rows, self.src)
    }
}

#[derive(Default)]
struct Graphics {
    wanted: Option<Placement>,
    /// Image id the terminal holds.
    sent: Option<u32>,
    placed: Option<(u32, u16, u16, u16, u16, [u32; 4])>,
}

static STATE: Mutex<Graphics> = Mutex::new(Graphics { wanted: None, sent: None, placed: None });

/// kitty / Ghostty / WezTerm, outside tmux (which doesn't pass the protocol
/// through). LAZYPX4_GRAPHICS=off forces the half-block fallback,
/// =kitty forces the protocol on.
pub fn supported() -> bool {
    static ON: OnceLock<bool> = OnceLock::new();
    *ON.get_or_init(|| {
        let env = |k: &str| std::env::var(k).unwrap_or_default();
        match env("LAZYPX4_GRAPHICS").as_str() {
            "off" | "0" | "none" => return false,
            "kitty" | "on" | "1" => return true,
            _ => {}
        }
        if !env("TMUX").is_empty() {
            return false;
        }
        !env("KITTY_WINDOW_ID").is_empty()
            || env("TERM") == "xterm-kitty"
            || env("TERM") == "xterm-ghostty"
            || matches!(env("TERM_PROGRAM").as_str(), "ghostty" | "WezTerm")
    })
}

fn lock() -> std::sync::MutexGuard<'static, Graphics> {
    STATE.lock().unwrap_or_else(|e| e.into_inner())
}

/// What this frame should show (`None`: no image). Call while drawing.
pub fn want(p: Option<Placement>) {
    lock().wanted = p;
}

/// Bring the terminal in line with the last [`want`]; after `terminal.draw`.
pub fn flush() {
    if !supported() {
        return;
    }
    let mut g = lock();
    let wanted = g.wanted.take();
    if wanted.as_ref().map(Placement::key) == g.placed {
        return;
    }
    let mut out = Vec::new();
    match &wanted {
        None => {
            if let Some((id, ..)) = g.placed {
                out.extend(format!("\x1b_Ga=d,d=i,i={id},p={PLACEMENT_ID},q=2\x1b\\").bytes());
            }
        }
        Some(p) => {
            let id = p.image.id();
            if g.sent != Some(id) {
                if let Some(old) = g.sent {
                    out.extend(format!("\x1b_Ga=d,d=I,i={old},q=2\x1b\\").bytes());
                }
                transmit(&mut out, id, &p.image);
                g.sent = Some(id);
            }
            let [x, y, w, h] = p.src;
            // Save cursor, go to the anchor cell, place without moving it.
            out.extend(
                format!(
                    "\x1b7\x1b[{};{}H\x1b_Ga=p,i={id},p={PLACEMENT_ID},x={x},y={y},w={w},h={h},c={},r={},z={Z_UNDER_BG},C=1,q=2\x1b\\\x1b8",
                    p.row + 1,
                    p.col + 1,
                    p.cols,
                    p.rows
                )
                .bytes(),
            );
        }
    }
    g.placed = wanted.as_ref().map(Placement::key);
    let mut stdout = std::io::stdout();
    let _ = stdout.write_all(&out);
    let _ = stdout.flush();
}

/// Forget everything on the terminal (before leaving the alternate screen:
/// a child process or exit). The next frame re-sends what it needs.
pub fn clear() {
    if !supported() {
        return;
    }
    let mut g = lock();
    if g.sent.is_some() {
        let mut stdout = std::io::stdout();
        let _ = stdout.write_all(b"\x1b_Ga=d,d=A,q=2\x1b\\");
        let _ = stdout.flush();
    }
    *g = Graphics::default();
}

/// PNG, base64, 4096-byte chunks.
fn transmit(out: &mut Vec<u8>, id: u32, image: &SatImage) {
    let mut png = std::io::Cursor::new(Vec::new());
    if image.rgb().write_to(&mut png, image::ImageFormat::Png).is_err() {
        return;
    }
    let data = base64(png.get_ref());
    let chunks: Vec<&[u8]> = data.chunks(4096).collect();
    for (i, chunk) in chunks.iter().enumerate() {
        let more = (i + 1 < chunks.len()) as u8;
        if i == 0 {
            out.extend(format!("\x1b_Ga=t,f=100,t=d,i={id},q=2,m={more};").bytes());
        } else {
            out.extend(format!("\x1b_Gm={more};").bytes());
        }
        out.extend_from_slice(chunk);
        out.extend_from_slice(b"\x1b\\");
    }
}

fn base64(data: &[u8]) -> Vec<u8> {
    const ABC: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = Vec::with_capacity(data.len().div_ceil(3) * 4);
    for c in data.chunks(3) {
        let n = (c[0] as u32) << 16 | (*c.get(1).unwrap_or(&0) as u32) << 8 | *c.get(2).unwrap_or(&0) as u32;
        out.push(ABC[(n >> 18) as usize & 63]);
        out.push(ABC[(n >> 12) as usize & 63]);
        out.push(if c.len() > 1 { ABC[(n >> 6) as usize & 63] } else { b'=' });
        out.push(if c.len() > 2 { ABC[n as usize & 63] } else { b'=' });
    }
    out
}

#[cfg(test)]
mod tests {
    use super::base64;

    #[test]
    fn base64_padding() {
        assert_eq!(base64(b"Man"), b"TWFu");
        assert_eq!(base64(b"Ma"), b"TWE=");
        assert_eq!(base64(b"M"), b"TQ==");
    }
}
