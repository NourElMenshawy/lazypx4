//! Key handling and prompts for the payload screen ([o]): gimbal aiming,
//! camera, gripper, winch and actuators. The `:gimbal` / `:photo` / ...
//! commands land here too.

use crate::app::{Action, App, InputKind, Screen, normalize_vim_key};
use crate::config::*;
use crate::mav::payload::{self, WinchAction};
use crate::state;

impl App {
    pub(crate) fn request_payload_streams(&self) {
        let link = self.link.clone();
        let camera = state::lock(&self.shared).payload.camera_component;
        std::thread::spawn(move || payload::request_payload_streams(&link, camera));
    }

    pub(crate) fn handle_payload_key(&mut self, key: String) {
        match key.as_str() {
            "o" => {
                self.session.screen = Screen::Dashboard;
                return;
            }
            "ESC" => {
                self.focus_sidebar();
                return;
            }
            // Arrows aim; j/k / PgUp / PgDn still scroll the page.
            "UP" => return self.gimbal_step(1.0, 0.0),
            "DOWN" => return self.gimbal_step(-1.0, 0.0),
            "LEFT" => return self.gimbal_step(0.0, -1.0),
            "RIGHT" => return self.gimbal_step(0.0, 1.0),
            _ => {}
        }
        let key = normalize_vim_key(key);
        if self.scroll_main(&key) {
            return;
        }
        let step = self.session.gimbal_step;
        match key.as_str() {
            "[" => self.session.gimbal_step = (step / 2.0).max(GIMBAL_STEP_MIN_DEG),
            "]" => self.session.gimbal_step = (step * 2.0).min(GIMBAL_STEP_MAX_DEG),
            "0" => self.gimbal_point(0.0, Some(0.0)),
            "N" => self.gimbal_point(-90.0, Some(0.0)),
            "Y" => payload::toggle_yaw_lock(&self.link, &mut state::lock(&self.shared)),
            "c" => {
                let mut st = state::lock(&self.shared);
                if payload::take_gimbal_control(&self.link, &mut st) {
                    st.command("GIMBAL: requested primary control (DO_GIMBAL_MANAGER_CONFIGURE)");
                }
            }
            "I" => self.request_input(
                "Gimbal region of interest: LAT LON [ALT m MSL, blank = home altitude]  ·  none clears it:",
                InputKind::GimbalRoi,
            ),
            "p" => {
                payload::camera_photo(&self.link, &mut state::lock(&self.shared));
            }
            "i" => self.request_input(
                "Interval capture: SECONDS [COUNT]  (COUNT 0 = until stopped; blank or 0 stops it):",
                InputKind::CaptureInterval,
            ),
            "v" => self.cmd_video(None),
            "g" => self.request_input("Gripper: g = grab, r = release:", InputKind::Gripper),
            "w" => self.request_input(
                "Winch: relax · lock · deliver · hold · retract · len METRES (+ out, - in) · rate M/S:",
                InputKind::Winch,
            ),
            "s" => self.request_input(
                "Actuator: N VALUE [SECONDS [THEN]]  (Actuator Set N = 1-6, values -1..1; e.g. 2 1 10 -1 = on for 10 s, then -1):",
                InputKind::Actuator,
            ),
            "r" => {
                self.request_payload_streams();
                state::lock(&self.shared).info("Re-requested gimbal / camera / winch status");
            }
            _ => {}
        }
    }

    /// One arrow-key step from the last commanded (or measured) angle.
    fn gimbal_step(&mut self, pitch: f64, yaw: f64) {
        let step = self.session.gimbal_step;
        let mut st = state::lock(&self.shared);
        let (p, y) = st.payload.gimbal_base();
        payload::gimbal_pitch_yaw(&self.link, &mut st, p + pitch * step, y + yaw * step, true);
    }

    /// Absolute aim; `None` keeps the current yaw. Clears a ROI first so
    /// the angles take over.
    fn gimbal_point(&mut self, pitch: f64, yaw: Option<f64>) {
        let mut st = state::lock(&self.shared);
        if st.payload.gimbal_roi.is_some() && !payload::gimbal_roi_none(&self.link, &mut st) {
            return;
        }
        let yaw = yaw.unwrap_or_else(|| st.payload.gimbal_base().1);
        payload::gimbal_pitch_yaw(&self.link, &mut st, pitch, yaw, false);
    }

    pub(crate) fn gimbal_roi_submit(&mut self, text: &str) {
        let mut st = state::lock(&self.shared);
        match payload::parse_roi(text) {
            Ok(None) => {
                payload::gimbal_roi_none(&self.link, &mut st);
            }
            Ok(Some((lat, lon, alt))) => {
                let alt = match alt {
                    Some(a) => a,
                    None if st.home_set => st.home_alt,
                    None => return st.error("ROI: no home position yet - give the altitude (m MSL)"),
                };
                payload::gimbal_roi(&self.link, &mut st, lat, lon, alt);
            }
            Err(e) => st.error(e),
        }
    }

    pub(crate) fn capture_interval_submit(&mut self, text: &str) {
        let mut st = state::lock(&self.shared);
        match payload::parse_interval(text) {
            Ok(interval) => {
                payload::camera_interval(&self.link, &mut st, interval);
            }
            Err(e) => st.error(e),
        }
    }

    pub(crate) fn gripper_submit(&mut self, text: &str) {
        match payload::parse_gripper(text) {
            Ok(true) => self.request_confirmation("GRIPPER GRAB? Type YES", Action::Gripper(true)),
            Ok(false) => self.request_confirmation(
                "GRIPPER RELEASE - drops whatever it holds? Type YES",
                Action::Gripper(false),
            ),
            Err(e) => state::lock(&self.shared).error(e),
        }
    }

    pub(crate) fn winch_submit(&mut self, text: &str) {
        match payload::parse_winch(text) {
            Ok(action) => {
                let note = match action {
                    WinchAction::Relax => " - the line runs free",
                    WinchAction::Deliver => " - lowers and releases the payload",
                    _ => "",
                };
                self.request_confirmation(format!("WINCH {}{note}? Type YES", action.label()), Action::Winch(action));
            }
            Err(e) => state::lock(&self.shared).error(e),
        }
    }

    pub(crate) fn actuator_submit(&mut self, text: &str) {
        let req = match payload::parse_actuator(text) {
            Ok(r) => r,
            Err(e) => return state::lock(&self.shared).error(e),
        };
        let (n, value) = (req.index + 1, req.value);
        // Without THEN, go back to what lazypx4 last set, else -1 (the
        // bottom of the range - usually "off"). Shown in the prompt.
        let timer = req.timer.map(|(secs, then)| {
            let previous = state::lock(&self.shared).payload.actuators[req.index];
            (secs, then.or(previous).unwrap_or(-1.0))
        });
        let what = match timer {
            Some((secs, then)) => format!("to {value:+.2} for {secs:.1} s, then {then:+.2}"),
            None => format!("to {value:+.2}"),
        };
        self.request_confirmation(
            format!("Set actuator {n} (Peripheral via Actuator Set {n}) {what}? Type YES"),
            Action::Actuator(req.index, value, timer),
        );
    }

    // -----------------------------------------------------------------
    // `:` commands
    // -----------------------------------------------------------------

    /// `:gimbal PITCH [YAW] | center | nadir | roi ... | lock on|off | control`
    pub(crate) fn cmd_gimbal(&mut self, args: &str) {
        let words: Vec<String> = args.split_whitespace().map(str::to_lowercase).collect();
        let words: Vec<&str> = words.iter().map(String::as_str).collect();
        match words.as_slice() {
            ["center" | "centre" | "home"] => self.gimbal_point(0.0, Some(0.0)),
            ["nadir" | "down"] => self.gimbal_point(-90.0, Some(0.0)),
            ["control"] => self.handle_payload_key("c".into()),
            ["roi", ..] => self.gimbal_roi_submit(args.split_once(char::is_whitespace).map(|x| x.1).unwrap_or("")),
            ["lock", on @ ("on" | "off")] => {
                let mut st = state::lock(&self.shared);
                if st.payload.gimbal_yaw_lock != (*on == "on") {
                    payload::toggle_yaw_lock(&self.link, &mut st);
                }
            }
            _ => match payload::parse_angles(args) {
                Ok((pitch, yaw)) => self.gimbal_point(pitch, yaw),
                Err(_) => state::lock(&self.shared).error("usage: :gimbal PITCH [YAW] | center | nadir | roi LAT LON [ALT] | roi none | lock on|off | control"),
            },
        }
    }

    /// `:photo` one shot · `:photo SECONDS [COUNT]` interval · `:photo stop`
    pub(crate) fn cmd_photo(&mut self, args: &str) {
        if args.trim().is_empty() {
            payload::camera_photo(&self.link, &mut state::lock(&self.shared));
        } else {
            self.capture_interval_submit(args);
        }
    }

    /// `:video [start|stop]`; no argument toggles.
    pub(crate) fn cmd_video(&mut self, start: Option<bool>) {
        let mut st = state::lock(&self.shared);
        let start = start.unwrap_or_else(|| !st.payload.video_running());
        payload::camera_video(&self.link, &mut st, start);
    }
}
