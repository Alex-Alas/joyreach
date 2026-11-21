# AGENTS.md — joyreach

Rust + Bevy 0.20 + avian3d 0.8 desktop demo: two Joy-Cons drive virtual arms on a flat screen. Single binary crate (no lib). Windows-only workflow (MSVC). Spec/milestones: `docs/BRIEF.md`; visual/feel rules and the screenshot checklist: `DESIGN.md` (§13). User-facing docs are in Spanish.

## Build / run (PowerShell)
- New shells may lack cargo/CMake on PATH: `$env:Path = [Environment]::GetEnvironmentVariable('Path','User') + ';' + [Environment]::GetEnvironmentVariable('Path','Machine')`.
- `cargo build --all-targets` / `cargo run`. Default feature `dev` = `bevy/dynamic_linking` (incremental ~20 s). A cold build compiles Bevy + SDL3 from source (~25 min): never `cargo clean`, never kill a running cargo, never use short timeouts.
- Release: `cargo build --release --no-default-features`.
- Linker is `rust-lld.exe` (`.cargo/config.toml`). `sdl3-sys` builds SDL statically (needs CMake + MSVC).
- Don't edit `Cargo.toml` without the user's explicit approval (deps/features cost rebuild time).
- `pwsh tools\snapshot.ps1` copies the dev build plus `bevy_dylib.dll`, Rust's `std-*.dll` and `assets\` into `play\` (gitignored). The user plays `play\joyreach.exe` while code changes. Refresh it after a verified milestone.

## Tests
- `cargo test` runs all unit tests (inline `mod tests` per file; no `tests/` dir).
- `cargo test --example joycon_probe` runs only the `joycon.rs` tests (fast).
- `examples/joycon_probe.rs` includes `src/joycon.rs` via `#[path]`. So `joycon.rs` must compile standalone: no `use crate::<other module>` in it. Duplicate tiny pure helpers instead.

## Runtime switches (verify features without hardware)
- `JOYREACH_FAKE=1`: animated fake hands. The demo script times a grab so the screenshot catches it.
- `cargo run -- --screenshot <path>`: saves a frame after 10 s (`JOYREACH_SHOT_SECS` overrides) and exits.
  - Meshes only draw once their pipelines finish compiling in the background. An early capture shows sky + flat ground with half the scene missing; that is loading, not a render bug. Don't "fix" it by removing features.
  - It also captures black 3D (HUD only) if the window is occluded or launched via `Start-Process`. Run it in the foreground and pixel-check the result.
- `JOYREACH_VIEW=behind|front|side`: starting camera view (screenshots of the avatar). `JOYREACH_CAM=0` disables the camera feed; `JOYREACH_CAM="<dshow name>"` picks a device.
- Don't capture the user's whole desktop to verify windows: it exposes their other apps.
- `JOYREACH_PITCH=<deg>`: initial camera pitch (e.g. −70 to show the auto-crouch).
- `JOYREACH_REACH=0|1`: forces IMU hand depth off/on. Default: on only when an accel calibration file exists.
- Probe:
  - `cargo run --example joycon_probe -- --secs 20` prints per-hand state;
  - `--rumble` tests vibration;
  - `--calibrate` is the guided 6-face accel calibration;
  - `--record file.csv` writes raw IMU (`ts_ns,hand,gx,gy,gz,ax,ay,az`), replayable in tests.

## Hardware facts
- Joy-Cons pair over Bluetooth and fall asleep. If the probe says "no Joy-Con detected", a button press is needed: don't wait in a loop, ask the user.
- Accel calibration lives in `%LOCALAPPDATA%\joyreach\imu_calib.txt` (model `a_cal = scale*(a_raw-offset)`) and is loaded at startup. Recorded CSVs are RAW, so apply the calibration in replay.
- Per-hand gravity differs uncalibrated (L ≈10.1–10.4, R ≈8.9–9.1 m/s²). Never subtract a fixed 9.81 from raw accel.
- SDL sensor timestamps drive integration; the first ~1000 samples arrive at 5 ms intervals. Rate is ~200–350 Hz.
- On the combined pair, HD Rumble low band = left Joy-Con, high band = right.

## Architecture notes
- `src/joycon.rs`: the SDL3 thread is the only hardware access. It publishes the `Hands` resource (`HandInput`: orientation, buttons, `reach`). It also contains `ImuFusion` (Mahony + variance-based gyro bias), `ReachTracker` (dead reckoning), calibration, and `Rumble` messages back to SDL.
- `src/arms.rs`: port of Daydream `GvrArmModel` (orientation → elbow/wrist) + two-bone IK for `reach`. Hands are kinematic avian bodies chasing targets by velocity, so pushes impart momentum.
- `src/player.rs`: kinematic capsule via avian `move_and_slide`. The arm rig origin is offset by `eye_height − EYE_HEIGHT`, so crouch moves the arms.
- `src/grab.rs`: direct hand grab and throw, plus beam hold: distance, L/R + stick spin, `PropMode` physics on release. `BeamStatus` tells player.rs when a stick rotates a prop instead of moving/looking. `src/handray.rs`: palm ray with aim assist (8° cone + line-of-sight check), reusable as the window pointer in a later milestone.
- `src/views.rs`: camera view modes. Avatar head/torso and the full upper arms are on render layer 1 (`AVATAR_LAYER`); the first-person camera draws layer 0 only.
- `src/webcam.rs`: camera via the `ffmpeg` CLI piped into a borderless `ffplay` window (a separate window, not a Bevy texture). Recording goes to `recordings/` (gitignored). Children are stopped with `q` in `Drop` so MP4s finalise.
- Design decision: Joy-Cons are trustworthy for orientation, not position. Depth from the IMU must stay bounded (clamp, ZUPT, slow return) and must not double-count motion the arm model already explains.

## Conventions
- Check Bevy/avian APIs against the sources in `~\.cargo\registry\src\index.crates.io-*\` (Bevy 0.20 APIs differ from older tutorials; e.g. `Query::single_mut` returns `Result`; avian `Forces` is QueryData, not `&mut`).
- Keep `cargo build --all-targets` warning-free and every test green before committing.
- Commit messages: `M<n><letter>: <summary>` per milestone. Several agents may work in parallel: stage explicit files (`git add <files>`), never `git add -A`.
- Keep `play/`, `target/` and CSV/PNG scratch out of git.
