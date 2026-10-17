//! Payloads: the gimbal (Gimbal Protocol v2, through PX4's gimbal manager),
//! the camera (MAVLink Camera Protocol v2 when a camera component sends a
//! HEARTBEAT, else PX4's camera_trigger driver), the gripper (PX4's
//! payload_deliverer), the winch (a MAVLink winch component - PX4 itself
//! has no winch driver) and generic outputs (DO_SET_ACTUATOR, PX4's
//! "Peripheral via Actuator Set N" functions).
//!
//! Gimbal and camera commands go out at once, like a jog step - aiming a
//! camera can't drop anything. The gripper, winch and actuator senders run
//! behind a "type YES" (see `Action` in app.rs).

use mavlink::dialects::development::{self as mav, MavCmd};

use crate::config::*;
use crate::link::{GCS_COMPONENT_ID, GCS_SYSTEM_ID, Link};
use crate::mav::commands::{command_int, vehicle_ready};
use crate::mav::handlers::quaternion_to_euler_deg;
use crate::state::{self, Shared, State, now};

/// DO_SET_ACTUATOR carries six values (param7 = 0 addresses 1-6).
pub const ACTUATOR_COUNT: usize = 6;

const NAN: f32 = f32::NAN;

const MSG_CAMERA_CAPTURE_STATUS: f32 = 262.0;
const MSG_GIMBAL_MANAGER_INFORMATION: f32 = 280.0;

// GIMBAL_MANAGER_FLAGS / GIMBAL_DEVICE_FLAGS (the low 16 bits match).
const GIMBAL_FLAGS_ROLL_LOCK: u32 = 4;
const GIMBAL_FLAGS_PITCH_LOCK: u32 = 8;
const GIMBAL_FLAGS_YAW_LOCK: u32 = 16;
const GIMBAL_FLAGS_YAW_IN_VEHICLE_FRAME: u32 = 32;
const GIMBAL_FLAGS_YAW_IN_EARTH_FRAME: u32 = 64;

/// What the payload screen knows. Gripper and actuator state is only what
/// was last commanded from here - PX4 reports neither.
#[derive(Debug, Default)]
pub struct Payload {
    /// Components of the locked system announcing themselves by HEARTBEAT.
    pub camera_component: Option<u8>,
    pub last_camera_heartbeat: f64,
    pub gimbal_component: Option<u8>,
    pub last_gimbal_heartbeat: f64,
    pub winch_component: Option<u8>,
    pub last_winch_heartbeat: f64,

    // GIMBAL_DEVICE_ATTITUDE_STATUS, degrees. Yaw both ways round.
    pub last_gimbal_attitude: f64,
    pub gimbal_roll: f64,
    pub gimbal_pitch: f64,
    /// Relative to the vehicle's nose.
    pub gimbal_yaw: f64,
    /// Relative to North.
    pub gimbal_yaw_north: f64,
    pub gimbal_failure_flags: u32,

    // GIMBAL_MANAGER_INFORMATION.
    pub last_gimbal_info: f64,
    pub gimbal_pitch_range: Option<(f64, f64)>,
    pub gimbal_yaw_range: Option<(f64, f64)>,

    // GIMBAL_MANAGER_STATUS: who is in control (0, 0 = nobody).
    pub last_gimbal_status: f64,
    pub gimbal_primary: (u8, u8),

    /// Last pitch / yaw sent; yaw is North-relative while `gimbal_yaw_lock`.
    pub gimbal_cmd: Option<(f64, f64)>,
    pub gimbal_yaw_lock: bool,
    /// lat, lon, AMSL of the active region of interest.
    pub gimbal_roi: Option<(f64, f64, f64)>,
    pub gimbal_control_sent_at: f64,

    // CAMERA_CAPTURE_STATUS (from a MAVLink camera).
    pub last_capture_status: f64,
    pub last_capture_status_request: f64,
    pub image_status: u8,
    pub video_status: u8,
    pub image_interval: f64,
    pub recording_time_s: f64,
    pub available_mib: f64,
    pub image_count: i32,

    // CAMERA_IMAGE_CAPTURED.
    pub images_captured: u32,
    pub images_failed: u32,
    pub last_image_index: i32,
    pub last_image_ok: bool,
    pub last_image_at: f64,

    /// CAMERA_TRIGGER, from PX4's own camera_trigger driver.
    pub trigger_seq: u32,
    pub last_trigger: f64,

    /// Sequence number for single-shot IMAGE_START_CAPTURE.
    pub capture_sequence: u32,
    /// The camera_trigger fallback has been explained in the event log.
    pub camera_fallback_noted: bool,
    /// Commanded from here; the capture status overrides when it's fresh.
    pub interval_active: bool,
    pub video_active: bool,

    /// (what, when) last commanded.
    pub gripper_last: Option<(String, f64)>,
    pub winch_last: Option<(String, f64)>,

    // WINCH_STATUS.
    pub last_winch_status: f64,
    pub winch_line_m: f64,
    pub winch_speed: f64,
    pub winch_tension_kg: f64,
    pub winch_flags: u32,

    /// Last DO_SET_ACTUATOR value sent per output, -1..1.
    pub actuators: [Option<f32>; ACTUATOR_COUNT],
    /// "Hold for N seconds, then go to X", per output.
    pub actuator_timers: [Option<ActuatorTimer>; ACTUATOR_COUNT],
}

/// PX4 holds an Actuator Set value until told otherwise, so a timed pulse
/// is two commands and the second one is ours to send - and to repeat until
/// PX4 acknowledges it, since a lost revert would leave the output on.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ActuatorTimer {
    pub then: f32,
    pub due: f64,
    /// Revert commands sent so far (0 = still counting down).
    pub tries: u32,
    pub last_sent: f64,
}

impl Payload {
    pub fn gimbal_fresh(&self) -> bool {
        fresh(self.last_gimbal_attitude)
    }

    pub fn capture_status_fresh(&self) -> bool {
        fresh(self.last_capture_status)
    }

    /// Interval capture running: the camera's word when it has one.
    pub fn interval_running(&self) -> bool {
        if self.capture_status_fresh() { matches!(self.image_status, 2 | 3) } else { self.interval_active }
    }

    pub fn video_running(&self) -> bool {
        if self.capture_status_fresh() { self.video_status == 1 } else { self.video_active }
    }

    /// The pitch / yaw an arrow-key step starts from: the last command,
    /// else where the gimbal points now, else straight ahead.
    pub fn gimbal_base(&self) -> (f64, f64) {
        if let Some(cmd) = self.gimbal_cmd {
            return cmd;
        }
        if self.gimbal_fresh() {
            let yaw = if self.gimbal_yaw_lock { self.gimbal_yaw_north } else { self.gimbal_yaw };
            return (self.gimbal_pitch, yaw);
        }
        (0.0, 0.0)
    }
}

fn fresh(last: f64) -> bool {
    last > 0.0 && now() - last < TELEMETRY_TIMEOUT
}

pub fn wrap180(deg: f64) -> f64 {
    (deg + 180.0).rem_euclid(360.0) - 180.0
}

// ---------------------------------------------------------------------------
// Telemetry
// ---------------------------------------------------------------------------

/// A camera / gimbal / winch component of the locked system. True when the
/// HEARTBEAT is one of those (so it says nothing about the vehicle).
pub fn handle_heartbeat(st: &mut State, src_sys: u8, src_comp: u8, mavtype: mav::MavType) -> bool {
    use mav::MavType as T;
    if !matches!(mavtype, T::MAV_TYPE_CAMERA | T::MAV_TYPE_GIMBAL | T::MAV_TYPE_WINCH) {
        return false;
    }
    if !st.vehicle_locked || src_sys != st.target_system {
        return true;
    }
    let t = now();
    let p = &mut st.payload;
    let (slot, last, what) = match mavtype {
        T::MAV_TYPE_CAMERA => (&mut p.camera_component, &mut p.last_camera_heartbeat, "camera"),
        T::MAV_TYPE_GIMBAL => (&mut p.gimbal_component, &mut p.last_gimbal_heartbeat, "gimbal"),
        _ => (&mut p.winch_component, &mut p.last_winch_heartbeat, "winch"),
    };
    *last = t;
    if *slot != Some(src_comp) {
        *slot = Some(src_comp);
        st.info(format!("MAVLink {what} found: component {src_comp}"));
    }
    true
}

pub fn handle_gimbal_attitude(st: &mut State, d: &mav::GIMBAL_DEVICE_ATTITUDE_STATUS_DATA) {
    let (roll, pitch, yaw) = quaternion_to_euler_deg(d.q);
    let flags = d.flags.bits() as u32;
    // Without either frame flag, YAW_LOCK alone says "earth frame".
    let earth = flags & GIMBAL_FLAGS_YAW_IN_EARTH_FRAME != 0
        || (flags & GIMBAL_FLAGS_YAW_IN_VEHICLE_FRAME == 0 && flags & GIMBAL_FLAGS_YAW_LOCK != 0);
    let vehicle_yaw = st.yaw;
    let p = &mut st.payload;
    p.last_gimbal_attitude = now();
    p.gimbal_roll = roll;
    p.gimbal_pitch = pitch;
    if earth {
        p.gimbal_yaw_north = wrap180(yaw);
        p.gimbal_yaw = wrap180(yaw - vehicle_yaw);
    } else {
        p.gimbal_yaw = wrap180(yaw);
        p.gimbal_yaw_north = wrap180(yaw + vehicle_yaw);
    }
    p.gimbal_failure_flags = d.failure_flags.bits();
}

pub fn handle_gimbal_info(st: &mut State, d: &mav::GIMBAL_MANAGER_INFORMATION_DATA) {
    let range = |lo: f32, hi: f32| (lo.is_finite() && hi.is_finite() && hi > lo).then(|| (lo.to_degrees() as f64, hi.to_degrees() as f64));
    let p = &mut st.payload;
    p.last_gimbal_info = now();
    p.gimbal_pitch_range = range(d.pitch_min, d.pitch_max);
    p.gimbal_yaw_range = range(d.yaw_min, d.yaw_max);
}

pub fn handle_gimbal_status(st: &mut State, d: &mav::GIMBAL_MANAGER_STATUS_DATA) {
    let p = &mut st.payload;
    p.last_gimbal_status = now();
    p.gimbal_primary = (d.primary_control_sysid, d.primary_control_compid);
}

pub fn handle_capture_status(st: &mut State, d: &mav::CAMERA_CAPTURE_STATUS_DATA) {
    let p = &mut st.payload;
    p.last_capture_status = now();
    p.image_status = d.image_status;
    p.video_status = d.video_status;
    p.image_interval = if d.image_interval.is_finite() { d.image_interval as f64 } else { 0.0 };
    p.recording_time_s = d.recording_time_ms as f64 / 1000.0;
    p.available_mib = if d.available_capacity.is_finite() { d.available_capacity as f64 } else { -1.0 };
    p.image_count = d.image_count;
}

pub fn handle_image_captured(st: &mut State, d: &mav::CAMERA_IMAGE_CAPTURED_DATA) {
    let ok = d.capture_result.contains(mav::MavBool::MAV_BOOL_TRUE);
    let p = &mut st.payload;
    p.last_image_at = now();
    p.last_image_index = d.image_index;
    p.last_image_ok = ok;
    if ok {
        p.images_captured += 1;
    } else {
        p.images_failed += 1;
        st.warn(format!("Camera: image #{} FAILED", d.image_index));
    }
}

pub fn handle_camera_trigger(st: &mut State, d: &mav::CAMERA_TRIGGER_DATA) {
    st.payload.trigger_seq = d.seq;
    st.payload.last_trigger = now();
}

pub fn handle_winch_status(st: &mut State, d: &mav::WINCH_STATUS_DATA) {
    let p = &mut st.payload;
    p.last_winch_status = now();
    p.winch_line_m = d.line_length as f64;
    p.winch_speed = d.speed as f64;
    p.winch_tension_kg = d.tension as f64;
    p.winch_flags = d.status.bits();
}

pub fn winch_flag_names(flags: u32) -> Vec<&'static str> {
    const NAMES: [&str; 14] = [
        "HEALTHY", "RETRACTED", "MOVING", "CLUTCH", "LOCKED", "DROPPING", "ARRESTING",
        "GROUND_SENSE", "RETRACTING", "REDELIVER", "ABANDON", "LOCKING", "LOAD_LINE", "LOAD_PAYLOAD",
    ];
    NAMES.iter().enumerate().filter(|(i, _)| flags & (1 << i) != 0).map(|(_, n)| *n).collect()
}

// ---------------------------------------------------------------------------
// Requests
// ---------------------------------------------------------------------------

/// Gimbal / winch streams and the one-shot gimbal limits. Paced like
/// `configure_streams`, so it runs on a helper thread.
pub fn request_payload_streams(link: &Link, camera: Option<u8>) {
    let pace = || std::thread::sleep(CONFIGURE_STREAMS_PACING);
    for &(msg_id, interval_us) in PAYLOAD_MESSAGE_INTERVALS {
        link.command_long(MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL, [msg_id as f32, interval_us as f32, 0., 0., 0., 0., 0.]);
        pace();
    }
    link.command_long(MavCmd::MAV_CMD_REQUEST_MESSAGE, [MSG_GIMBAL_MANAGER_INFORMATION, 0., 0., 0., 0., 0., 0.]);
    if let Some(comp) = camera {
        pace();
        link.command_long_to(comp, MavCmd::MAV_CMD_REQUEST_MESSAGE, [MSG_CAMERA_CAPTURE_STATUS, 0., 0., 0., 0., 0., 0.]);
    }
}

/// While the payload screen is up: a camera only sends its capture status
/// when asked.
pub fn poll_camera(link: &Link, st: &mut State) {
    let Some(comp) = st.payload.camera_component else { return };
    let t = now();
    if t - st.payload.last_capture_status_request < CAMERA_STATUS_POLL_S {
        return;
    }
    st.payload.last_capture_status_request = t;
    link.command_long_to(comp, MavCmd::MAV_CMD_REQUEST_MESSAGE, [MSG_CAMERA_CAPTURE_STATUS, 0., 0., 0., 0., 0., 0.]);
}

// ---------------------------------------------------------------------------
// Gimbal
// ---------------------------------------------------------------------------

/// PX4's gimbal manager ignores a GCS that hasn't claimed primary control.
fn ensure_gimbal_control(link: &Link, st: &mut State) {
    let ours = (GCS_SYSTEM_ID, GCS_COMPONENT_ID);
    let primary = st.payload.gimbal_primary;
    if primary == ours || now() - st.payload.gimbal_control_sent_at < GIMBAL_CONTROL_RETRY_S {
        return;
    }
    take_gimbal_control(link, st);
    if primary != (0, 0) {
        st.warn(format!("Gimbal: taking control from sys {} comp {}", primary.0, primary.1));
    }
}

pub fn take_gimbal_control(link: &Link, st: &mut State) -> bool {
    st.payload.gimbal_control_sent_at = now();
    link.command_long(
        MavCmd::MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
        [GCS_SYSTEM_ID as f32, GCS_COMPONENT_ID as f32, -1., -1., NAN, NAN, 0.],
    )
}

/// Point the gimbal: pitch up-positive, yaw right-positive, degrees. Yaw is
/// from the nose, or from North while the yaw lock is on. `quiet` keeps
/// arrow-key steps out of the event log.
pub fn gimbal_pitch_yaw(link: &Link, st: &mut State, pitch: f64, yaw: f64, quiet: bool) -> bool {
    if !vehicle_ready(st, link) {
        return false;
    }
    let lock = st.payload.gimbal_yaw_lock;
    let (pmin, pmax) = st.payload.gimbal_pitch_range.unwrap_or((-90.0, 90.0));
    let pitch = pitch.clamp(pmin, pmax);
    let yaw = match st.payload.gimbal_yaw_range {
        Some((lo, hi)) if !lock && hi - lo < 359.0 => yaw.clamp(lo, hi),
        _ => wrap180(yaw),
    };
    ensure_gimbal_control(link, st);
    let frame = if lock {
        GIMBAL_FLAGS_YAW_LOCK | GIMBAL_FLAGS_YAW_IN_EARTH_FRAME
    } else {
        GIMBAL_FLAGS_YAW_IN_VEHICLE_FRAME
    };
    let flags = GIMBAL_FLAGS_ROLL_LOCK | GIMBAL_FLAGS_PITCH_LOCK | frame;
    if !link.command_long(
        MavCmd::MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
        [pitch as f32, yaw as f32, NAN, NAN, flags as f32, 0., 0.],
    ) {
        st.error("Failed to send gimbal command");
        return false;
    }
    st.payload.gimbal_cmd = Some((pitch, yaw));
    st.payload.gimbal_roi = None;
    if !quiet {
        let from = if lock { "North" } else { "nose" };
        st.command(format!("GIMBAL: pitch {pitch:+.1}°  yaw {yaw:+.1}° from {from}"));
    }
    true
}

/// Switch between yaw-follows-the-vehicle and yaw-locked-to-North, keeping
/// the gimbal pointing where it is.
pub fn toggle_yaw_lock(link: &Link, st: &mut State) {
    let lock = !st.payload.gimbal_yaw_lock;
    let (pitch, yaw) = st.payload.gimbal_base();
    st.payload.gimbal_yaw_lock = lock;
    let yaw = if lock { yaw + st.yaw } else { yaw - st.yaw };
    st.info(if lock { "Gimbal yaw LOCKED to North" } else { "Gimbal yaw FOLLOWS the vehicle" });
    if st.payload.gimbal_cmd.is_some() || st.payload.gimbal_fresh() {
        gimbal_pitch_yaw(link, st, pitch, yaw, true);
    }
}

/// Region of interest at lat / lon / AMSL; the gimbal tracks it.
pub fn gimbal_roi(link: &Link, st: &mut State, lat: f64, lon: f64, amsl: f64) -> bool {
    if !vehicle_ready(st, link) {
        return false;
    }
    ensure_gimbal_control(link, st);
    let sent = command_int(
        link,
        mav::MavFrame::MAV_FRAME_GLOBAL,
        MavCmd::MAV_CMD_DO_SET_ROI_LOCATION,
        [0., 0., 0., 0.],
        (lat * 1e7).round() as i32,
        (lon * 1e7).round() as i32,
        amsl as f32,
    );
    if !sent {
        st.error("Failed to send gimbal ROI");
        return false;
    }
    st.payload.gimbal_roi = Some((lat, lon, amsl));
    st.payload.gimbal_cmd = None;
    st.command(format!("GIMBAL ROI: {lat:.7}, {lon:.7} @ {amsl:.1} m MSL"));
    true
}

pub fn gimbal_roi_none(link: &Link, st: &mut State) -> bool {
    if !vehicle_ready(st, link) {
        return false;
    }
    if !link.command_long(MavCmd::MAV_CMD_DO_SET_ROI_NONE, [0., NAN, NAN, NAN, NAN, NAN, NAN]) {
        st.error("Failed to clear gimbal ROI");
        return false;
    }
    st.payload.gimbal_roi = None;
    st.command("GIMBAL ROI cleared");
    true
}

// ---------------------------------------------------------------------------
// Camera
// ---------------------------------------------------------------------------

fn camera_fallback_note(st: &mut State) {
    if std::mem::replace(&mut st.payload.camera_fallback_noted, true) {
        return;
    }
    st.info("No MAVLink camera seen - triggering through PX4's camera_trigger (TRIG_MODE / TRIG_INTERFACE)");
}

/// One photo: IMAGE_START_CAPTURE to a MAVLink camera, else
/// DO_DIGICAM_CONTROL "shoot" to PX4.
pub fn camera_photo(link: &Link, st: &mut State) -> bool {
    if !vehicle_ready(st, link) {
        return false;
    }
    let sent = match st.payload.camera_component {
        Some(comp) => {
            st.payload.capture_sequence += 1;
            let seq = st.payload.capture_sequence as f32;
            link.command_long_to(comp, MavCmd::MAV_CMD_IMAGE_START_CAPTURE, [0., 0., 1., seq, NAN, NAN, NAN])
        }
        None => {
            camera_fallback_note(st);
            link.command_long(MavCmd::MAV_CMD_DO_DIGICAM_CONTROL, [0., 0., 0., 0., 1., 0., 0.])
        }
    };
    if !sent {
        st.error("Failed to send photo command");
        return false;
    }
    st.payload.last_capture_status_request = 0.0;
    st.command("CAMERA: photo");
    true
}

/// Interval capture every `interval_s` (`count` 0 = until stopped), or
/// stop it with `None`.
pub fn camera_interval(link: &Link, st: &mut State, interval: Option<(f64, u32)>) -> bool {
    if !vehicle_ready(st, link) {
        return false;
    }
    let sent = match (st.payload.camera_component, interval) {
        (Some(comp), Some((s, n))) => {
            link.command_long_to(comp, MavCmd::MAV_CMD_IMAGE_START_CAPTURE, [0., s as f32, n as f32, 0., NAN, NAN, NAN])
        }
        (Some(comp), None) => link.command_long_to(comp, MavCmd::MAV_CMD_IMAGE_STOP_CAPTURE, [0., NAN, NAN, NAN, NAN, NAN, NAN]),
        (None, Some((s, n))) => {
            camera_fallback_note(st);
            if n > 0 {
                st.warn("PX4's camera_trigger has no image count - it runs until stopped");
            }
            link.command_long(MavCmd::MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL, [(s * 1000.0) as f32, -1., NAN, NAN, NAN, NAN, NAN])
                && link.command_long(MavCmd::MAV_CMD_DO_TRIGGER_CONTROL, [1., -1., -1., NAN, NAN, NAN, NAN])
        }
        (None, None) => link.command_long(MavCmd::MAV_CMD_DO_TRIGGER_CONTROL, [0., -1., -1., NAN, NAN, NAN, NAN]),
    };
    if !sent {
        st.error("Failed to send interval-capture command");
        return false;
    }
    st.payload.interval_active = interval.is_some();
    st.payload.last_capture_status_request = 0.0;
    st.command(match interval {
        Some((s, 0)) => format!("CAMERA: interval capture every {s:.1} s"),
        Some((s, n)) => format!("CAMERA: {n} images, every {s:.1} s"),
        None => "CAMERA: interval capture stopped".into(),
    });
    true
}

/// Video needs a MAVLink camera - PX4's trigger driver only takes stills.
pub fn camera_video(link: &Link, st: &mut State, start: bool) -> bool {
    if !vehicle_ready(st, link) {
        return false;
    }
    let Some(comp) = st.payload.camera_component else {
        st.error("Video: no MAVLink camera found (PX4's camera trigger takes stills only)");
        return false;
    };
    let sent = if start {
        // param2: CAMERA_CAPTURE_STATUS at 1 Hz while recording.
        link.command_long_to(comp, MavCmd::MAV_CMD_VIDEO_START_CAPTURE, [0., 1., NAN, NAN, NAN, NAN, NAN])
    } else {
        link.command_long_to(comp, MavCmd::MAV_CMD_VIDEO_STOP_CAPTURE, [0., NAN, NAN, NAN, NAN, NAN, NAN])
    };
    if !sent {
        st.error("Failed to send video command");
        return false;
    }
    st.payload.video_active = start;
    st.payload.last_capture_status_request = 0.0;
    st.command(if start { "CAMERA: video START" } else { "CAMERA: video STOP" });
    true
}

// ---------------------------------------------------------------------------
// Gripper / winch / actuators (each behind a YES)
// ---------------------------------------------------------------------------

pub fn send_gripper(link: &Link, shared: &Shared, grab: bool) -> bool {
    let mut st = state::lock(shared);
    if !vehicle_ready(&mut st, link) {
        return false;
    }
    let what = if grab { "GRAB" } else { "RELEASE" };
    if !link.command_long(MavCmd::MAV_CMD_DO_GRIPPER, [1., if grab { 1. } else { 0. }, 0., 0., 0., 0., 0.]) {
        st.error(format!("Failed to send gripper {what}"));
        return false;
    }
    st.payload.gripper_last = Some((what.into(), now()));
    st.command(format!("GRIPPER {what} SENT"));
    true
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WinchAction {
    Relax,
    Lock,
    Deliver,
    Hold,
    Retract,
    /// Metres of line to let out (negative winds in).
    Length(f64),
    /// m/s to let line out at (negative winds in).
    Rate(f64),
}

impl WinchAction {
    pub fn label(self) -> String {
        match self {
            WinchAction::Relax => "RELAX".into(),
            WinchAction::Lock => "LOCK".into(),
            WinchAction::Deliver => "DELIVER".into(),
            WinchAction::Hold => "HOLD".into(),
            WinchAction::Retract => "RETRACT".into(),
            WinchAction::Length(m) => format!("LINE {m:+.2} m"),
            WinchAction::Rate(r) => format!("RATE {r:+.2} m/s"),
        }
    }

    /// (WINCH_ACTIONS value, length, rate) for MAV_CMD_DO_WINCH.
    fn params(self) -> (f32, f32, f32) {
        match self {
            WinchAction::Relax => (mav::WinchActions::WINCH_RELAXED as u32 as f32, 0., 0.),
            WinchAction::Length(m) => (mav::WinchActions::WINCH_RELATIVE_LENGTH_CONTROL as u32 as f32, m as f32, 0.),
            WinchAction::Rate(r) => (mav::WinchActions::WINCH_RATE_CONTROL as u32 as f32, 0., r as f32),
            WinchAction::Lock => (mav::WinchActions::WINCH_LOCK as u32 as f32, 0., 0.),
            WinchAction::Deliver => (mav::WinchActions::WINCH_DELIVER as u32 as f32, 0., 0.),
            WinchAction::Hold => (mav::WinchActions::WINCH_HOLD as u32 as f32, 0., 0.),
            WinchAction::Retract => (mav::WinchActions::WINCH_RETRACT as u32 as f32, 0., 0.),
        }
    }
}

pub fn send_winch(link: &Link, shared: &Shared, action: WinchAction) -> bool {
    let mut st = state::lock(shared);
    if !vehicle_ready(&mut st, link) {
        return false;
    }
    let (value, length, rate) = action.params();
    let label = action.label();
    let params = [1., value, length, rate, 0., 0., 0.];
    let sent = match st.payload.winch_component {
        Some(comp) => link.command_long_to(comp, MavCmd::MAV_CMD_DO_WINCH, params),
        None => {
            st.warn("No MAVLink winch seen - sending to the autopilot, but PX4 has no winch driver of its own");
            link.command_long(MavCmd::MAV_CMD_DO_WINCH, params)
        }
    };
    if !sent {
        st.error(format!("Failed to send winch {label}"));
        return false;
    }
    st.payload.winch_last = Some((label.clone(), now()));
    st.command(format!("WINCH {label} SENT"));
    true
}

/// DO_SET_ACTUATOR for output `index` (0-based); NaN leaves the other
/// five untouched.
fn actuator_command(link: &Link, index: usize, value: f32) -> bool {
    let mut params = [NAN; 7];
    params[index] = value;
    params[6] = 0.0;
    link.command_long(MavCmd::MAV_CMD_DO_SET_ACTUATOR, params)
}

/// Output `index` of "Peripheral via Actuator Set" to `value`, and with
/// `timer` = (seconds, then) back to `then` afterwards. Any new value for
/// the output cancels a running timer.
pub fn send_actuator(link: &Link, shared: &Shared, index: usize, value: f32, timer: Option<(f64, f32)>) -> bool {
    let mut st = state::lock(shared);
    if !vehicle_ready(&mut st, link) {
        return false;
    }
    if !actuator_command(link, index, value) {
        st.error(format!("Failed to set actuator {}", index + 1));
        return false;
    }
    st.payload.actuators[index] = Some(value);
    st.payload.actuator_timers[index] =
        timer.map(|(secs, then)| ActuatorTimer { then, due: now() + secs, tries: 0, last_sent: 0.0 });
    st.command(match timer {
        Some((secs, then)) => format!("ACTUATOR {} -> {value:+.2} SENT, back to {then:+.2} in {secs:.1} s", index + 1),
        None => format!("ACTUATOR {} -> {value:+.2} SENT", index + 1),
    });
    true
}

/// Send the reverts that are due, and repeat them until acknowledged.
/// Every tick, like the other request timeouts.
pub fn check_actuator_timers(link: &Link, st: &mut State) {
    let t = now();
    for i in 0..ACTUATOR_COUNT {
        let Some(mut timer) = st.payload.actuator_timers[i] else { continue };
        if t < timer.due || (timer.tries > 0 && t - timer.last_sent < ACTUATOR_REVERT_RETRY_S) {
            continue;
        }
        if timer.tries >= ACTUATOR_REVERT_TRIES {
            st.payload.actuator_timers[i] = None;
            st.alert(format!("ACTUATOR {} timer: return to {:+.2} NOT confirmed by PX4 - check the output", i + 1, timer.then));
            continue;
        }
        actuator_command(link, i, timer.then);
        if timer.tries == 0 {
            st.command(format!("ACTUATOR {} timer done -> {:+.2}", i + 1, timer.then));
        }
        timer.tries += 1;
        timer.last_sent = t;
        st.payload.actuators[i] = Some(timer.then);
        st.payload.actuator_timers[i] = Some(timer);
    }
}

/// DO_SET_ACTUATOR ACCEPTED. The ACK doesn't say which output, so it
/// settles every revert in flight (a manual set acknowledged at the same
/// moment would too - an accepted cost for a once-a-second retry).
pub fn handle_actuator_ack(st: &mut State) {
    for slot in st.payload.actuator_timers.iter_mut() {
        if slot.is_some_and(|t| t.tries > 0) {
            *slot = None;
        }
    }
}

// ---------------------------------------------------------------------------
// Parsing (shared by the prompts and the `:` commands)
// ---------------------------------------------------------------------------

fn numbers(text: &str) -> Result<Vec<f64>, String> {
    text.replace(',', " ")
        .split_whitespace()
        .map(|t| t.parse::<f64>().ok().filter(|v| v.is_finite()).ok_or_else(|| format!("'{t}' is not a number")))
        .collect()
}

/// `PITCH [YAW]` in degrees.
pub fn parse_angles(text: &str) -> Result<(f64, Option<f64>), String> {
    match numbers(text)?.as_slice() {
        [p] => Ok((*p, None)),
        [p, y] => Ok((*p, Some(*y))),
        _ => Err("gimbal: PITCH [YAW] in degrees".into()),
    }
}

/// `LAT LON [ALT]`, or `none` / `off` / `clear` (-> None).
pub fn parse_roi(text: &str) -> Result<Option<(f64, f64, Option<f64>)>, String> {
    if matches!(text.trim().to_lowercase().as_str(), "none" | "off" | "clear") {
        return Ok(None);
    }
    let n = numbers(text).map_err(|e| format!("ROI: {e}"))?;
    let (lat, lon, alt) = match n.as_slice() {
        [la, lo] => (*la, *lo, None),
        [la, lo, a] => (*la, *lo, Some(*a)),
        _ => return Err("ROI: LAT LON [ALT m MSL], or none".into()),
    };
    if !(-90.0..=90.0).contains(&lat) || !(-180.0..=180.0).contains(&lon) {
        return Err("ROI: latitude / longitude out of range".into());
    }
    Ok(Some((lat, lon, alt)))
}

/// `SECONDS [COUNT]`; blank, `0` or `stop` -> None.
pub fn parse_interval(text: &str) -> Result<Option<(f64, u32)>, String> {
    let text = text.trim();
    if text.is_empty() || text.eq_ignore_ascii_case("stop") || text.eq_ignore_ascii_case("off") {
        return Ok(None);
    }
    let n = numbers(text).map_err(|e| format!("Interval: {e}"))?;
    let (s, count) = match n.as_slice() {
        [s] => (*s, 0.0),
        [s, c] => (*s, *c),
        _ => return Err("Interval: SECONDS [COUNT]".into()),
    };
    if s == 0.0 {
        return Ok(None);
    }
    if s < 0.0 || count < 0.0 || count.fract() != 0.0 {
        return Err("Interval: seconds must be positive, count a whole number".into());
    }
    Ok(Some((s, count as u32)))
}

/// true = grab.
pub fn parse_gripper(text: &str) -> Result<bool, String> {
    match text.trim().to_lowercase().as_str() {
        "g" | "grab" | "close" => Ok(true),
        "r" | "release" | "open" | "drop" => Ok(false),
        _ => Err("Gripper: grab or release".into()),
    }
}

pub fn parse_winch(text: &str) -> Result<WinchAction, String> {
    let lower = text.trim().to_lowercase();
    let words: Vec<&str> = lower.split_whitespace().collect();
    let number = |w: &str| w.parse::<f64>().ok().filter(|v| v.is_finite());
    Ok(match words.as_slice() {
        ["relax" | "relaxed"] => WinchAction::Relax,
        ["lock"] => WinchAction::Lock,
        ["deliver"] => WinchAction::Deliver,
        ["hold"] => WinchAction::Hold,
        ["retract"] => WinchAction::Retract,
        ["len" | "length", v] if number(v).is_some() => WinchAction::Length(number(v).unwrap()),
        ["rate", v] if number(v).is_some() => WinchAction::Rate(number(v).unwrap()),
        _ => return Err("Winch: relax · lock · deliver · hold · retract · len METRES · rate M/S".into()),
    })
}

/// A parsed `N VALUE [SECONDS [THEN]]`.
#[derive(Debug, PartialEq)]
pub struct ActuatorRequest {
    /// 0-based.
    pub index: usize,
    pub value: f32,
    /// Seconds to hold `value`, and what to go to afterwards (None = the
    /// caller picks).
    pub timer: Option<(f64, Option<f32>)>,
}

/// `N VALUE [SECONDS [THEN]]`: output 1-6 to -1..1, optionally only for
/// SECONDS.
pub fn parse_actuator(text: &str) -> Result<ActuatorRequest, String> {
    let n = numbers(text).map_err(|e| format!("Actuator: {e}"))?;
    let (i, v, secs, then) = match n.as_slice() {
        [i, v] => (*i, *v, None, None),
        [i, v, s] => (*i, *v, Some(*s), None),
        [i, v, s, t] => (*i, *v, Some(*s), Some(*t)),
        _ => return Err("Actuator: N VALUE [SECONDS [THEN]]  (N 1-6, values -1..1)".into()),
    };
    if i.fract() != 0.0 || !(1.0..=ACTUATOR_COUNT as f64).contains(&i) {
        return Err(format!("Actuator: N must be 1-{ACTUATOR_COUNT}"));
    }
    let in_range = |x: f64| (-1.0..=1.0).contains(&x);
    if !in_range(v) || then.is_some_and(|t| !in_range(t)) {
        return Err("Actuator: values must be within -1..1".into());
    }
    if secs.is_some_and(|s| s <= 0.0 || s > ACTUATOR_TIMER_MAX_S) {
        return Err(format!("Actuator: SECONDS must be within 0..{ACTUATOR_TIMER_MAX_S:.0}"));
    }
    Ok(ActuatorRequest {
        index: i as usize - 1,
        value: v as f32,
        timer: secs.map(|s| (s, then.map(|t| t as f32))),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_roi() {
        assert_eq!(parse_roi(" none "), Ok(None));
        assert_eq!(parse_roi("52.1, 6.9"), Ok(Some((52.1, 6.9, None))));
        assert_eq!(parse_roi("52.1 6.9 40"), Ok(Some((52.1, 6.9, Some(40.0)))));
        assert!(parse_roi("95 6").is_err());
        assert!(parse_roi("52").is_err());
    }

    #[test]
    fn parses_interval() {
        assert_eq!(parse_interval(""), Ok(None));
        assert_eq!(parse_interval("0"), Ok(None));
        assert_eq!(parse_interval("stop"), Ok(None));
        assert_eq!(parse_interval("2.5"), Ok(Some((2.5, 0))));
        assert_eq!(parse_interval("2 10"), Ok(Some((2.0, 10))));
        assert!(parse_interval("-1").is_err());
        assert!(parse_interval("2 1.5").is_err());
    }

    #[test]
    fn parses_gripper_winch_actuator() {
        assert_eq!(parse_gripper("G"), Ok(true));
        assert_eq!(parse_gripper("release"), Ok(false));
        assert!(parse_gripper("maybe").is_err());

        assert_eq!(parse_winch("Deliver"), Ok(WinchAction::Deliver));
        assert_eq!(parse_winch("len -1.5"), Ok(WinchAction::Length(-1.5)));
        assert_eq!(parse_winch("rate 0.3"), Ok(WinchAction::Rate(0.3)));
        assert!(parse_winch("len").is_err());

        let req = |index, value, timer| Ok(ActuatorRequest { index, value, timer });
        assert_eq!(parse_actuator("2 0.5"), req(1, 0.5, None));
        assert_eq!(parse_actuator("2 1 10"), req(1, 1.0, Some((10.0, None))));
        assert_eq!(parse_actuator("2 1 10 -1"), req(1, 1.0, Some((10.0, Some(-1.0)))));
        assert!(parse_actuator("7 0").is_err());
        assert!(parse_actuator("1 1.5").is_err());
        assert!(parse_actuator("1.5 0").is_err());
        assert!(parse_actuator("1 1 0").is_err());
        assert!(parse_actuator("1 1 10 2").is_err());
    }

    #[test]
    fn wraps_and_flags() {
        assert_eq!(wrap180(190.0), -170.0);
        assert_eq!(wrap180(-180.0), -180.0);
        assert_eq!(winch_flag_names(1 | 4), vec!["HEALTHY", "MOVING"]);
    }
}
