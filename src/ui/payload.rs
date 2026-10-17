//! The payload screen ([o]): gimbal attitude and aiming, camera capture,
//! gripper / winch, and the "Peripheral via Actuator Set" outputs.

use ratatui::style::{Modifier, Style};
use ratatui::text::Line;

use super::other::{age, section_with};
use super::*;
use crate::link::{GCS_COMPONENT_ID, GCS_SYSTEM_ID};
use crate::mav::payload::{self, ACTUATOR_COUNT};

const AXIS_WIDTH: usize = 37;

/// A min..max scale with the measured (●) and commanded (◆) angle on it.
fn axis(min: f64, max: f64, actual: Option<f64>, target: Option<f64>) -> Ln {
    let col = |v: f64| (((v - min) / (max - min)).clamp(0.0, 1.0) * (AXIS_WIDTH - 1) as f64).round() as usize;
    let dim = Style::new().add_modifier(Modifier::DIM);
    let mut cells = vec![('·', dim); AXIS_WIDTH];
    if min < 0.0 && max > 0.0 {
        cells[col(0.0)] = ('┼', dim);
    }
    if let Some(t) = target {
        cells[col(t)] = ('◆', Style::new().fg(MAUVE));
    }
    if let Some(a) = actual {
        cells[col(a)] = ('●', Style::new().fg(GREEN).add_modifier(Modifier::BOLD));
    }
    let l = cells.into_iter().fold(Ln::new().dim(format!("{min:>5.0} ")), |l, (c, s)| l.st(c.to_string(), s));
    l.dim(format!(" {max:<5.0}"))
}

fn degrees(v: Option<f64>) -> String {
    v.map(|v| format!("{v:+6.1}°")).unwrap_or_else(|| "    --".into())
}

fn since(t: f64) -> String {
    format!("{:.0}s ago", crate::state::now() - t)
}

pub fn draw(ctx: &Ctx) -> Vec<Line<'static>> {
    let st = ctx.st;
    let p = &st.payload;
    let mut lines = vec![Ln::new().raw(" MODE: ").bold(st.mode.clone()).raw("    ").spans(vec![armed_span(st.armed)]).line()];

    // --- gimbal ----------------------------------------------------------
    lines.push(blank());
    lines.push(section_with("GIMBAL", age(p.last_gimbal_attitude)));
    let fresh = p.gimbal_fresh();
    let lock = p.gimbal_yaw_lock;
    let (cmd_pitch, cmd_yaw) = match p.gimbal_cmd {
        Some((a, b)) => (Some(a), Some(b)),
        None => (None, None),
    };
    let (pmin, pmax) = p.gimbal_pitch_range.unwrap_or((-90.0, 30.0));
    let (ymin, ymax) = match p.gimbal_yaw_range {
        Some(r) if !lock => r,
        _ => (-180.0, 180.0),
    };
    let pitch = fresh.then_some(p.gimbal_pitch);
    let yaw = fresh.then_some(if lock { p.gimbal_yaw_north } else { p.gimbal_yaw });
    lines.push(
        Ln::new()
            .raw("   pitch ")
            .spans(axis(pmin, pmax, pitch, cmd_pitch).0)
            .raw(format!(" {}", degrees(pitch)))
            .fg(format!("  → {}", degrees(cmd_pitch)), MAUVE)
            .line(),
    );
    lines.push(
        Ln::new()
            .raw("   yaw   ")
            .spans(axis(ymin, ymax, yaw, cmd_yaw).0)
            .raw(format!(" {}", degrees(yaw)))
            .fg(format!("  → {}", degrees(cmd_yaw)), MAUVE)
            .dim(if lock { "  from North" } else { "  from the nose" })
            .line(),
    );
    if fresh {
        lines.push(Line::from(format!(
            "   roll {:+.1}°   yaw from nose {:+.1}°   from North {:+.1}°",
            p.gimbal_roll, p.gimbal_yaw, p.gimbal_yaw_north
        )));
        if p.gimbal_failure_flags != 0 {
            lines.push(Ln::new().raw("   ").fg(format!("device failure flags 0x{:04x}", p.gimbal_failure_flags), RED).line());
        }
    } else {
        lines.push(dim_line("no GIMBAL_DEVICE_ATTITUDE_STATUS - is a gimbal set up (MNT_MODE_IN = MAVLink v2, MNT_MODE_OUT)?"));
    }
    let control = if p.last_gimbal_status == 0.0 {
        Ln::new().dim("unknown")
    } else {
        match p.gimbal_primary {
            (0, 0) => Ln::new().fg("nobody - [c] takes it", YELLOW),
            id if id == (GCS_SYSTEM_ID, GCS_COMPONENT_ID) => Ln::new().fg("lazypx4", GREEN),
            (s, c) => Ln::new().fg(format!("sys {s} comp {c} - [c] takes it"), YELLOW),
        }
    };
    let yaw_mode = if lock { Ln::new().fg("LOCKED to North", CYAN) } else { Ln::new().raw("follows vehicle") };
    lines.push(
        Ln::new()
            .raw("   control: ")
            .spans(control.0)
            .raw("   yaw: ")
            .spans(yaw_mode.0)
            .raw(format!("   step {:.0}°", ctx.app.session.gimbal_step))
            .line(),
    );
    match p.gimbal_roi {
        Some((lat, lon, alt)) => lines.push(
            Ln::new().raw("   ROI: ").fg(format!("{lat:.7}, {lon:.7} @ {alt:.1} m MSL"), MAUVE).dim("   (0 / N / arrows end it)").line(),
        ),
        None => lines.push(dim_line("ROI: none")),
    }

    // --- camera ----------------------------------------------------------
    lines.push(blank());
    let camera_note = match p.camera_component {
        Some(c) => Ln::new().raw(format!("MAVLink camera, component {c} · heartbeat ")).spans(age(p.last_camera_heartbeat).0),
        None => Ln::new().dim("no MAVLink camera - stills go through PX4's camera trigger"),
    };
    lines.push(section_with("CAMERA", camera_note));
    let busy = Style::new().fg(GREEN).add_modifier(Modifier::BOLD);
    let interval = if p.interval_running() {
        let every = if p.capture_status_fresh() && p.image_interval > 0.0 { format!(" every {:.1} s", p.image_interval) } else { String::new() };
        Ln::new().st(format!("INTERVAL{every}"), busy)
    } else {
        Ln::new().raw("idle")
    };
    let video = if p.video_running() {
        let t = p.recording_time_s as u64;
        let clock = if p.capture_status_fresh() { format!(" {:02}:{:02}", t / 60, t % 60) } else { String::new() };
        Ln::new().st(format!("● REC{clock}"), Style::new().fg(RED).add_modifier(Modifier::BOLD))
    } else {
        Ln::new().raw("idle")
    };
    lines.push(Ln::new().raw("   photo: ").spans(interval.0).raw("   video: ").spans(video.0).line());
    if p.capture_status_fresh() {
        let storage = if p.available_mib >= 0.0 { format!("   storage free: {:.0} MiB", p.available_mib) } else { String::new() };
        lines.push(Line::from(format!("   images on camera: {}{storage}", p.image_count)));
    }
    if p.last_image_at > 0.0 {
        let (text, color) = if p.last_image_ok { ("OK", GREEN) } else { ("FAILED", RED) };
        lines.push(
            Ln::new()
                .raw(format!("   last image #{} ", p.last_image_index))
                .fg(text, color)
                .raw(format!(" {}   this session: {} ok, {} failed", since(p.last_image_at), p.images_captured, p.images_failed))
                .line(),
        );
    }
    if p.last_trigger > 0.0 {
        lines.push(Line::from(format!("   PX4 camera trigger: #{} {}", p.trigger_seq, since(p.last_trigger))));
    }

    // --- gripper / winch -------------------------------------------------
    lines.push(blank());
    lines.push(section_with("GRIPPER / WINCH", age(p.last_winch_status)));
    match &p.gripper_last {
        Some((what, t)) => lines.push(
            Ln::new().raw("   gripper: last ").bold(what.clone()).raw(format!(" sent {}", since(*t))).dim("   (PX4 reports no gripper state)").line(),
        ),
        None => lines.push(dim_line("gripper: not commanded this session")),
    }
    if p.last_winch_status > 0.0 {
        lines.push(Line::from(format!(
            "   winch: line {:.2} m   speed {:+.2} m/s   tension {:.2} kg   {}",
            p.winch_line_m,
            p.winch_speed,
            p.winch_tension_kg,
            payload::winch_flag_names(p.winch_flags).join(" ")
        )));
    } else if let Some(c) = p.winch_component {
        lines.push(dim_line(format!("winch: component {c}, no WINCH_STATUS yet")));
    } else {
        lines.push(dim_line("winch: no MAVLink winch seen (PX4 itself has no winch driver)"));
    }
    if let Some((what, t)) = &p.winch_last {
        lines.push(Ln::new().raw("   winch: last ").bold(what.clone()).raw(format!(" sent {}", since(*t))).line());
    }

    // --- actuators -------------------------------------------------------
    lines.push(blank());
    lines.push(section("ACTUATORS", "Peripheral via Actuator Set 1-6 · last value sent"));
    let cells = (0..ACTUATOR_COUNT).fold(Ln::new().raw("  "), |l, i| {
        let l = l.raw(format!(" {}: ", i + 1));
        match p.actuators[i] {
            Some(v) => l.fg(format!("{v:+.2} "), CYAN),
            None => l.dim("  --  "),
        }
    });
    lines.push(cells.line());
    for (i, timer) in p.actuator_timers.iter().enumerate() {
        let Some(t) = timer else { continue };
        let text = if t.tries == 0 {
            format!("   {}: back to {:+.2} in {:.0} s", i + 1, t.then, (t.due - crate::state::now()).max(0.0).ceil())
        } else {
            format!("   {}: returning to {:+.2} - waiting for PX4 to confirm ({}/{ACTUATOR_REVERT_TRIES})", i + 1, t.then, t.tries)
        };
        lines.push(Ln::new().fg(text, YELLOW).line());
    }
    if st.last_servo_output > 0.0 {
        let outs: Vec<String> = st.servo_outputs.iter().enumerate().map(|(i, v)| format!("S{} {v}", i + 1)).collect();
        lines.push(dim_line(format!("outputs (SERVO_OUTPUT_RAW): {}", outs.join("  "))));
    }
    lines.push(dim_line("assign the functions in PX4's Actuators setup (QGC) before using them"));

    // --- keys ------------------------------------------------------------
    lines.push(blank());
    lines.push(section("KEYS", "gimbal and camera act at once; gripper, winch and actuators ask YES"));
    lines.push(Ln::new().raw("   ").key("←↑↓→", "aim").bold("[ ]").raw("step  ").key("0", "center").key("N", "nadir").key("Y", "yaw lock").key("I", "ROI").key("c", "take control").line());
    lines.push(Ln::new().raw("   ").key("p", "photo").key("i", "interval").key("v", "video").key("g", "gripper").key("w", "winch").key("s", "actuator").key("r", "re-request").line());
    lines
}
