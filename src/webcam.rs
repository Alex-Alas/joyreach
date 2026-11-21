//! Camera feed over the game plus session recording, through the `ffmpeg` / `ffplay` CLIs
//! (no extra crates).
//!
//! * The camera (DirectShow; a phone through Windows "Enlace Móvil" shows up as e.g.
//!   "Redmi Note 13 Pro (Windows Virtual Camera)") is opened once by `ffmpeg`, scaled to 320×180
//!   and piped into a borderless, always-on-top `ffplay` window pinned to the game's top-left
//!   corner. `F8` toggles it. It is a separate window on purpose: in Bevy 0.20 updating an
//!   `Image` asset every frame (or rendering a second camera to one) stopped the 3D scene from
//!   drawing, while a separate window cannot disturb the renderer.
//! * `F9` or Capture (Joy-Con L) records the session to `recordings/`: the same `ffmpeg` also
//!   writes the camera at full resolution (`*_camara.mp4`), and a second one records the desktop
//!   (`*_pantalla.mp4`, which includes the game and the camera window), so the real hands and the
//!   virtual ones can be compared side by side.
//! * `JOYREACH_CAM="<device name>"` picks a camera; otherwise the first DirectShow video device
//!   is used. `JOYREACH_CAM=0` disables the feed.
//!
//! Processes are stopped with `q` so the MP4 files are finalised, also when the game closes.

use std::io::Write;
use std::path::PathBuf;
use std::process::{Child, ChildStdout, Command, Stdio};
use std::thread::JoinHandle;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use bevy::prelude::*;
use bevy::window::{PrimaryWindow, WindowPosition};

use crate::joycon::Hands;
use crate::theme;

pub const FEED_WIDTH: u32 = 320;
pub const FEED_HEIGHT: u32 = 180;
const RECORDINGS_DIR: &str = "recordings";
/// Offset of the camera window from the game window's outer top-left corner, in logical px:
/// below the title bar, under the FPS badge.
const FEED_OFFSET: Vec2 = Vec2::new(26.0, 96.0);

#[cfg(windows)]
fn hide_console(command: &mut Command) {
    use std::os::windows::process::CommandExt;
    const CREATE_NO_WINDOW: u32 = 0x0800_0000;
    command.creation_flags(CREATE_NO_WINDOW);
}

#[cfg(not(windows))]
fn hide_console(_command: &mut Command) {}

/// A running ffmpeg. Dropping it asks it to finish (`q`) so its files are valid.
struct Ffmpeg {
    child: Child,
}

impl Ffmpeg {
    fn spawn(args: &[String], pipe_out: bool) -> std::io::Result<(Self, Option<ChildStdout>)> {
        let mut command = Command::new("ffmpeg");
        command
            .args(args)
            .stdin(Stdio::piped())
            .stdout(if pipe_out { Stdio::piped() } else { Stdio::null() })
            .stderr(Stdio::null());
        hide_console(&mut command);
        let mut child = command.spawn()?;
        let stdout = child.stdout.take();
        Ok((Self { child }, stdout))
    }

    fn running(&mut self) -> bool {
        matches!(self.child.try_wait(), Ok(None))
    }
}

impl Drop for Ffmpeg {
    fn drop(&mut self) {
        if let Some(mut stdin) = self.child.stdin.take() {
            let _ = stdin.write_all(b"q");
        }
        let deadline = Instant::now() + Duration::from_secs(4);
        while Instant::now() < deadline {
            if !matches!(self.child.try_wait(), Ok(None)) {
                return;
            }
            std::thread::sleep(Duration::from_millis(30));
        }
        let _ = self.child.kill();
    }
}

/// The camera pipeline: ffmpeg (device → preview pipe [+ file]) → ffplay window.
struct Feed {
    /// Declared first so it is dropped first: ffmpeg finalises the file, then ffplay closes.
    ffmpeg: Ffmpeg,
    ffplay: Option<Child>,
}

impl Drop for Feed {
    fn drop(&mut self) {
        if let Some(mut ffplay) = self.ffplay.take() {
            let _ = ffplay.kill();
        }
    }
}

struct Recording {
    camera_path: Option<PathBuf>,
    screen: Option<Ffmpeg>,
    screen_path: PathBuf,
    started: Instant,
}

#[derive(Resource)]
pub struct Webcam {
    device: Option<String>,
    detecting: Option<JoinHandle<Option<String>>>,
    feed: Option<Feed>,
    enabled: bool,
    recording: Option<Recording>,
    prev_capture: bool,
    status: String,
    /// Where the camera window was placed (logical px), to follow the game window.
    placed_at: Option<IVec2>,
}

impl Webcam {
    fn start_feed(&mut self, record_to: Option<PathBuf>, at: IVec2) {
        self.feed = None;
        let Some(device) = self.device.clone() else {
            self.status = "Sin cámara (JOYREACH_CAM)".into();
            return;
        };
        let (ffmpeg, stdout) = match Ffmpeg::spawn(&camera_args(&device, record_to.as_ref()), true)
        {
            Ok(spawned) => spawned,
            Err(error) => {
                self.status = format!("ffmpeg no disponible: {error}");
                return;
            }
        };
        let ffplay = stdout.and_then(|stdout| {
            let mut command = Command::new("ffplay");
            command
                .args(preview_args(at))
                .stdin(Stdio::from(stdout))
                .stdout(Stdio::null())
                .stderr(Stdio::null());
            hide_console(&mut command);
            command.spawn().ok()
        });
        self.status = format!("Cámara: {device}");
        self.placed_at = Some(at);
        self.feed = Some(Feed { ffmpeg, ffplay });
    }

    fn start_recording(&mut self, at: IVec2) {
        let stamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map_or(0, |d| d.as_secs());
        let dir = PathBuf::from(RECORDINGS_DIR);
        let _ = std::fs::create_dir_all(&dir);
        let camera_path = dir.join(format!("sesion_{stamp}_camara.mp4"));
        let screen_path = dir.join(format!("sesion_{stamp}_pantalla.mp4"));

        let camera = self.device.is_some().then(|| camera_path.clone());
        self.start_feed(camera.clone(), at);
        let screen = Ffmpeg::spawn(&screen_args(&screen_path), false)
            .ok()
            .map(|(process, _)| process);
        info!(
            "recording: camera {:?}, screen {} ({})",
            camera,
            screen_path.display(),
            if screen.is_some() { "ok" } else { "failed" }
        );
        self.recording = Some(Recording {
            camera_path: camera,
            screen,
            screen_path,
            started: Instant::now(),
        });
    }

    fn stop_recording(&mut self, at: IVec2) {
        if let Some(recording) = self.recording.take() {
            drop(recording.screen);
            // Restart the camera without the file output (dropping the old feed finalises it).
            if self.enabled {
                self.start_feed(None, at);
            } else {
                self.feed = None;
            }
            info!(
                "recording saved: {:?} and {}",
                recording.camera_path,
                recording.screen_path.display()
            );
        }
    }
}

fn strings(items: &[&str]) -> Vec<String> {
    items.iter().map(|s| s.to_string()).collect()
}

/// ffmpeg arguments for the camera: a 320×180 RGBA preview on stdout (for ffplay), and with
/// `record_to` also the full-resolution camera into an MP4.
fn camera_args(device: &str, record_to: Option<&PathBuf>) -> Vec<String> {
    let mut args = strings(&["-hide_banner", "-loglevel", "error", "-f", "dshow", "-rtbufsize", "100M", "-i"]);
    args.push(format!("video={device}"));
    let preview = format!("fps=15,scale={FEED_WIDTH}:{FEED_HEIGHT},format=rgba");
    match record_to {
        Some(path) => {
            args.push("-filter_complex".into());
            args.push(format!("[0:v]split=2[f][r];[f]{preview}[p]"));
            args.extend(strings(&["-map", "[p]", "-f", "rawvideo", "pipe:1"]));
            args.extend(strings(&[
                "-map", "[r]", "-c:v", "libx264", "-preset", "veryfast", "-pix_fmt", "yuv420p", "-y",
            ]));
            args.push(path.to_string_lossy().into_owned());
        }
        None => {
            args.push("-vf".into());
            args.push(preview);
            args.extend(strings(&["-f", "rawvideo", "pipe:1"]));
        }
    }
    args
}

/// ffplay arguments: raw RGBA from stdin in a borderless, always-on-top window at `at`.
fn preview_args(at: IVec2) -> Vec<String> {
    let mut args = strings(&[
        "-hide_banner",
        "-loglevel",
        "quiet",
        "-fflags",
        "nobuffer",
        "-flags",
        "low_delay",
        "-framedrop",
        "-f",
        "rawvideo",
        "-pixel_format",
        "rgba",
        "-framerate",
        "15",
    ]);
    args.push("-video_size".into());
    args.push(format!("{FEED_WIDTH}x{FEED_HEIGHT}"));
    args.extend(strings(&["-i", "pipe:0", "-noborder", "-alwaysontop", "-window_title", "joyreach camara"]));
    args.extend(["-left".into(), at.x.to_string(), "-top".into(), at.y.to_string()]);
    args
}

/// ffmpeg arguments for the screen (whole desktop, so it works regardless of window focus).
fn screen_args(path: &PathBuf) -> Vec<String> {
    let mut args = strings(&[
        "-hide_banner", "-loglevel", "error", "-f", "gdigrab", "-framerate", "30", "-draw_mouse", "0",
        "-i", "desktop", "-vf", "scale=1280:-2", "-c:v", "libx264", "-preset", "ultrafast",
        "-pix_fmt", "yuv420p", "-y",
    ]);
    args.push(path.to_string_lossy().into_owned());
    args
}

/// First DirectShow video device, parsed from `ffmpeg -list_devices`.
fn detect_camera() -> Option<String> {
    let mut command = Command::new("ffmpeg");
    command.args(["-hide_banner", "-list_devices", "true", "-f", "dshow", "-i", "dummy"]);
    hide_console(&mut command);
    let output = command.output().ok()?;
    parse_first_video_device(&String::from_utf8_lossy(&output.stderr))
}

pub fn parse_first_video_device(listing: &str) -> Option<String> {
    listing.lines().find_map(|line| {
        if !line.trim_end().ends_with("(video)") {
            return None;
        }
        let start = line.find('"')? + 1;
        let end = start + line[start..].find('"')?;
        Some(line[start..end].to_string())
    })
}

/// Logical-pixel position for the camera window: the game window's outer corner + offset.
/// ffplay (SDL, not DPI-aware) works in logical pixels, Bevy reports physical ones.
fn feed_position(window: &Window) -> IVec2 {
    let scale = window.scale_factor().max(0.5);
    let outer = match window.position {
        WindowPosition::At(position) => position.as_vec2() / scale,
        _ => Vec2::new(0.0, 0.0),
    };
    (outer + FEED_OFFSET).round().as_ivec2()
}

#[derive(Component)]
struct FeedLabel;

pub struct WebcamPlugin;

impl Plugin for WebcamPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup_webcam)
            .add_systems(Update, (webcam_input, update_feed_label).chain());
    }
}

fn setup_webcam(mut commands: Commands, assets: Res<AssetServer>) {
    let env = std::env::var("JOYREACH_CAM").ok();
    let enabled = env.as_deref() != Some("0");
    let (device, detecting) = match env {
        Some(name) if name != "0" && !name.is_empty() => (Some(name), None),
        Some(_) => (None, None),
        None => (None, Some(std::thread::spawn(detect_camera))),
    };
    commands.insert_resource(Webcam {
        device,
        detecting,
        feed: None,
        enabled,
        recording: None,
        prev_capture: false,
        status: if enabled {
            "Buscando cámara…".into()
        } else {
            "Cámara apagada (F8)".into()
        },
        placed_at: None,
    });

    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                top: px(56),
                left: px(18),
                padding: UiRect::axes(px(10), px(5)),
                border_radius: BorderRadius::all(px(6)),
                ..default()
            },
            BackgroundColor(theme::plate()),
        ))
        .with_child((
            FeedLabel,
            Text::new(""),
            TextFont {
                font: assets.load(theme::FONT_MEDIUM).into(),
                font_size: FontSize::Px(13.0),
                ..default()
            },
            TextColor(theme::INK),
        ));
}

fn webcam_input(
    keys: Res<ButtonInput<KeyCode>>,
    hands: Res<Hands>,
    time: Res<Time>,
    window: Single<&Window, With<PrimaryWindow>>,
    mut cam: ResMut<Webcam>,
) {
    let at = feed_position(&window);

    // Camera detection finished in the background (or the device came from JOYREACH_CAM).
    if cam.detecting.as_ref().is_some_and(|handle| handle.is_finished()) {
        let handle = cam.detecting.take().expect("checked above");
        cam.device = handle.join().ok().flatten();
        info!("webcam: {:?}", cam.device);
        if cam.device.is_none() {
            cam.status = "No se encontró cámara".into();
        }
    }
    let waiting_to_start = cam.enabled
        && cam.feed.is_none()
        && cam.detecting.is_none()
        && cam.device.is_some()
        && cam.placed_at.is_none();
    // Wait until the window reports where it is (or give up on that after a few seconds).
    if waiting_to_start
        && (matches!(window.position, WindowPosition::At(_)) || time.elapsed_secs() > 3.0)
    {
        cam.start_feed(None, at);
    }

    if keys.just_pressed(KeyCode::F8) {
        cam.enabled = !cam.enabled;
        if cam.enabled {
            cam.start_feed(None, at);
        } else if cam.recording.is_none() {
            cam.feed = None;
            cam.status = "Cámara apagada (F8)".into();
        }
    }

    let capture = hands.left.connected && hands.left.extra;
    let capture_pressed = capture && !cam.prev_capture;
    cam.prev_capture = capture;
    if keys.just_pressed(KeyCode::F9) || capture_pressed {
        if cam.recording.is_some() {
            cam.stop_recording(at);
        } else {
            cam.start_recording(at);
        }
    }

    // The game window moved: move the camera window with it (not while recording, which would
    // split the camera file).
    let moved = cam
        .placed_at
        .is_some_and(|placed| (placed - at).abs().max_element() > 24);
    if moved && cam.feed.is_some() && cam.recording.is_none() {
        cam.start_feed(None, at);
    }

    // A feed that died (phone disconnected, device busy): report it.
    let died = cam
        .feed
        .as_mut()
        .is_some_and(|feed| !feed.ffmpeg.running());
    if died {
        cam.feed = None;
        cam.status = "La cámara se desconectó (F8 para reintentar)".into();
    }
}

fn update_feed_label(cam: Res<Webcam>, mut label: Single<&mut Text, With<FeedLabel>>) {
    let recording = cam
        .recording
        .as_ref()
        .map(|r| format!(" · ● REC {:.0} s (F9)", r.started.elapsed().as_secs_f32()))
        .unwrap_or_default();
    label.0 = format!("{}{recording}", cam.status);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn finds_the_phone_camera_in_the_dshow_listing() {
        let listing = "[in#0 @ 0x1] \"Redmi Note 13 Pro (Windows Virtual Camera)\" (video)\n\
                       [in#0 @ 0x1]   Alternative name \"@device_pnp_...\"\n\
                       [in#0 @ 0x1] \"Microphone Array (Realtek(R) Audio)\" (audio)\n";
        assert_eq!(
            parse_first_video_device(listing).as_deref(),
            Some("Redmi Note 13 Pro (Windows Virtual Camera)")
        );
    }

    #[test]
    fn no_video_device_gives_none() {
        assert_eq!(parse_first_video_device("\"Mic\" (audio)\n"), None);
    }

    #[test]
    fn recording_args_split_preview_and_file() {
        let args = camera_args("Cam", Some(&PathBuf::from("out.mp4")));
        assert!(args.iter().any(|a| a.contains("split=2")));
        assert_eq!(args.last().map(String::as_str), Some("out.mp4"));
        let preview = camera_args("Cam", None);
        assert_eq!(preview.last().map(String::as_str), Some("pipe:1"));
    }

    #[test]
    fn preview_window_is_placed_where_asked() {
        let args = preview_args(IVec2::new(120, 80));
        let left = args.iter().position(|a| a == "-left").unwrap();
        assert_eq!(args[left + 1], "120");
        let top = args.iter().position(|a| a == "-top").unwrap();
        assert_eq!(args[top + 1], "80");
    }
}
