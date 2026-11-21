//! Standalone Joy-Con probe: no Bevy window, just the SDL thread from `src/joycon.rs`.
//!
//! ```text
//! cargo run --example joycon_probe -- --secs 8
//! cargo run --example joycon_probe -- --secs 4 --rumble   # left then right, 300 ms each
//! cargo run --example joycon_probe -- --calibrate          # guided 6-face accel calibration
//! cargo run --example joycon_probe -- --secs 20 --record session.csv
//! ```
//!
//! Prints, ~10 times per second, for each hand: connection/topology/gamepad type, enabled SDL
//! sensors and their data rate, measured IMU rate, raw (SDL device frame) and mapped (body frame)
//! gyro/accel, fused yaw/pitch/roll, stick, buttons and any SDL errors.
//!
//! `--calibrate` measures each hand in six still orientations, fits a per-axis scale/offset so
//! `|a| = 9.80665 m/s²`, prints the residual before/after and saves
//! `%LOCALAPPDATA%\joyreach\imu_calib.txt` (the game loads it on the next start).
//!
//! `--record` writes every raw IMU sample (`ts_ns,hand,gx,gy,gz,ax,ay,az`) so a real session can
//! be replayed through `ImuFusion` + `ReachTracker` by the tests.
//!
//! Depth (`reach`) is ON by default when `%LOCALAPPDATA%\joyreach\imu_calib.txt` was loaded;
//! `JOYREACH_REACH=0` forces it off and `=1` forces it on. The probe prints the published reach
//! next to the part the arm model already explains and the residual dead-reckoned translation.

#[path = "../src/joycon.rs"]
#[allow(dead_code)]
mod joycon;

use std::fs::File;
use std::io::{BufWriter, Write};
use std::sync::mpsc;
use std::time::{Duration, Instant};

use bevy::math::Vec3;
use joycon::{
    CalibPose, Hand, HandDebug, HandInput, ImuCalibration, ImuSample, JoyConWorker, RumbleHand,
};

fn main() {
    let args = match parse_args() {
        Ok(args) => args,
        Err(message) => {
            eprintln!("{message}");
            print_usage();
            std::process::exit(2);
        }
    };
    if args.calibrate {
        run_calibration();
    } else {
        run_probe(args.secs, args.rumble, args.record.as_deref());
    }
}

struct Args {
    secs: f64,
    rumble: bool,
    calibrate: bool,
    record: Option<String>,
}

fn parse_args() -> Result<Args, String> {
    let mut args = Args {
        secs: 10.0,
        rumble: false,
        calibrate: false,
        record: None,
    };
    let mut iter = std::env::args().skip(1);
    while let Some(arg) = iter.next() {
        match arg.as_str() {
            "--secs" | "-s" => {
                let value = iter
                    .next()
                    .ok_or_else(|| "--secs needs a number (seconds)".to_string())?;
                args.secs = value
                    .parse::<f64>()
                    .map_err(|_| format!("invalid --secs value: {value}"))?;
                if !(args.secs.is_finite() && args.secs > 0.0) {
                    return Err("--secs must be a positive number".to_string());
                }
            }
            "--rumble" | "-r" => args.rumble = true,
            "--calibrate" | "-c" => args.calibrate = true,
            "--record" => {
                let value = iter
                    .next()
                    .ok_or_else(|| "--record needs a file path".to_string())?;
                args.record = Some(value);
            }
            "-h" | "--help" => {
                print_usage();
                std::process::exit(0);
            }
            other => return Err(format!("unknown argument: {other}")),
        }
    }
    Ok(args)
}

fn print_usage() {
    println!(
        "usage: joycon_probe [--secs N] [--rumble] [--calibrate] [--record FILE]\n\
         \n\
         --secs N        run the diagnostic for N seconds (default 10)\n\
         --rumble        pulse left then right, 300 ms each\n\
         --calibrate     guided 6-pose accelerometer calibration (saves imu_calib.txt)\n\
         --record FILE   write every raw IMU sample to FILE (CSV) during the run"
    );
}

// ---------------------------------------------------------------------------------------------
// Normal probe
// ---------------------------------------------------------------------------------------------

fn run_probe(secs: f64, rumble: bool, record: Option<&str>) {
    let (recorder, receiver) = match record {
        Some(_) => {
            let (tx, rx) = mpsc::channel::<ImuSample>();
            (Some(tx), Some(rx))
        }
        None => (None, None),
    };
    let worker = match recorder {
        Some(tx) => JoyConWorker::spawn_with_recorder(Some(tx)),
        None => JoyConWorker::spawn(),
    };

    println!(
        "Joy-Con probe: {secs:.0} s. If both hands stay disconnected, the Joy-Cons are probably \
         asleep - press a button (or press any Joy-Con button) to wake them."
    );
    println!(
        "(Sensors only exist once SDL's HIDAPI Switch driver takes the device; SDL errors are \
         reported below.)"
    );
    if rumble {
        println!("(Rumble test: left Joy-Con at 0.6 s, right Joy-Con at 1.2 s, 300 ms each.)");
    }
    if record.is_some() {
        println!("(Recording every raw IMU sample; the CSV is written when the probe exits.)");
    }

    let mut samples: Vec<ImuSample> = Vec::new();
    let start = Instant::now();
    let mut frame: u64 = 0;
    let mut rumble_stage = 0_u8;
    while start.elapsed().as_secs_f64() < secs {
        if let Some(rx) = &receiver {
            while let Ok(sample) = rx.try_recv() {
                samples.push(sample);
            }
        }
        let hands = worker.snapshot();
        let debug = worker.debug_snapshot();

        println!("\n[{:>5.1}s] frame {frame}", start.elapsed().as_secs_f64());
        print_hand("LEFT ", &hands.left, &debug.left);
        print_hand("RIGHT", &hands.right, &debug.right);
        let _ = std::io::stdout().flush();

        // Pulse left, then right, so the user can feel which physical Joy-Con buzzes.
        let elapsed = start.elapsed().as_secs_f64();
        if rumble && rumble_stage == 0 && elapsed > 0.6 {
            print_rumble(&worker, RumbleHand::Left);
            rumble_stage = 1;
        }
        if rumble && rumble_stage == 1 && elapsed > 1.2 {
            print_rumble(&worker, RumbleHand::Right);
            rumble_stage = 2;
        }

        frame += 1;
        std::thread::sleep(Duration::from_millis(100));
    }

    if let Some(rx) = &receiver {
        while let Ok(sample) = rx.try_recv() {
            samples.push(sample);
        }
    }

    // Dropping the worker stops the SDL thread and calls SDL_Quit.
    drop(worker);
    println!(
        "\nprobe finished after {:.1} s",
        start.elapsed().as_secs_f64()
    );
    if let Some(path) = record {
        write_csv(path, &samples);
    }
}

fn write_csv(path: &str, samples: &[ImuSample]) {
    let file = match File::create(path) {
        Ok(file) => file,
        Err(err) => {
            eprintln!("[record] could not create {path}: {err}");
            return;
        }
    };
    let mut out = BufWriter::new(file);
    if writeln!(out, "ts_ns,hand,gx,gy,gz,ax,ay,az").is_err() {
        eprintln!("[record] could not write the CSV header");
        return;
    }
    for s in samples {
        let line = format!(
            "{},{},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}",
            s.ts_ns,
            s.hand.short(),
            s.gyro.x,
            s.gyro.y,
            s.gyro.z,
            s.accel.x,
            s.accel.y,
            s.accel.z
        );
        if writeln!(out, "{line}").is_err() {
            eprintln!("[record] write failed at sample {}", s.ts_ns);
            return;
        }
    }
    let _ = out.flush();
    println!("[record] {} samples -> {path}", samples.len());
}

fn print_rumble(worker: &JoyConWorker, hand: RumbleHand) {
    let label = match hand {
        RumbleHand::Left => "left",
        RumbleHand::Right => "right",
    };
    match worker.rumble_checked(hand, 0.8, 300) {
        Ok(true) => println!("[rumble] {label}: SDL_RumbleGamepad -> true (300 ms, strength 0.8)"),
        Ok(false) => println!(
            "[rumble] {label}: SDL_RumbleGamepad -> false (no connected pad accepted it; see the SDL notes above)"
        ),
        Err(message) => println!("[rumble] {label}: queued but no reply ({message})"),
    }
}

// ---------------------------------------------------------------------------------------------
// Guided calibration
// ---------------------------------------------------------------------------------------------

const CALIB_PROMPTS: [&str; 6] = [
    "Cara de botones hacia ARRIBA (el stick mirando al techo).",
    "Cara de botones hacia ABAJO (apoyado sobre el dorso).",
    "Apoyado sobre el borde del riel IZQUIERDO.",
    "Apoyado sobre el borde del riel DERECHO.",
    "Vertical, con el extremo del TOPE (ZL/ZR) hacia abajo.",
    "Vertical, con el extremo del RIEL hacia abajo (invertido).",
];

fn run_calibration() {
    let (tx, rx) = mpsc::channel::<ImuSample>();
    let worker = JoyConWorker::spawn_with_recorder(Some(tx));

    println!("=== Calibración del acelerómetro (M2c) ===");
    println!(
        "Vamos a medir cada Joy-Con en 6 poses distintas. En cada paso dejá los dos Joy-Con \
         QUIETOS ~1 segundo (apoyados en la mesa o sostenidos sin moverlos)."
    );
    println!("La pose tiene que ser distinta a las anteriores (más de 25° de diferencia).");

    // Hardware may be asleep: wait only briefly, then tell the user instead of hanging.
    let deadline = Instant::now() + Duration::from_secs(3);
    let mut active = [false; 2];
    while Instant::now() < deadline && !(active[0] || active[1]) {
        while rx.try_recv().is_ok() {}
        let hands = worker.snapshot();
        active = [hands.left.connected, hands.right.connected];
        std::thread::sleep(Duration::from_millis(20));
    }
    if !active[0] && !active[1] {
        eprintln!(
            "No se detectó ningún Joy-Con. Encendelos (apretá un botón) y volvé a correr \
             `cargo run --example joycon_probe -- --calibrate`."
        );
        return;
    }
    if !active[0] || !active[1] {
        println!(
            "Aviso: solo está conectado el Joy-Con {}. Se calibra ese; el otro sigue con el \
             fallback escalar.",
            if active[0] { "IZQUIERDO" } else { "DERECHO" }
        );
    }

    let mut collectors = [joycon::CalibCollector::new(), joycon::CalibCollector::new()];
    let mut completed = [!active[0], !active[1]];

    for (step, prompt) in CALIB_PROMPTS.iter().enumerate() {
        println!("\nPaso {}/6: {prompt}", step + 1);
        println!("  Dejá los dos Joy-Con quietos en esa pose ~1 s.");
        // Throw away anything queued before the prompt so the previous pose cannot be accepted.
        while rx.try_recv().is_ok() {}
        for hand in 0..2 {
            collectors[hand].begin_step();
        }
        let mut done = completed;
        let mut warned = [false; 2];
        let deadline = Instant::now() + Duration::from_secs(20);

        while Instant::now() < deadline && !(done[0] && done[1]) {
            while let Ok(sample) = rx.try_recv() {
                let hand = match sample.hand {
                    Hand::Left => 0,
                    Hand::Right => 1,
                };
                if active[hand] && !done[hand] {
                    collectors[hand].push(sample.ts_ns, sample.accel, sample.gyro);
                }
            }
            for hand in 0..2 {
                if !active[hand] || done[hand] {
                    continue;
                }
                match collectors[hand].take_pose() {
                    CalibPose::Accepted(mean) => {
                        done[hand] = true;
                        println!(
                            "  ok {}: |a|={:.2} m/s² ({} poses)",
                            hand_name(hand),
                            mean.length(),
                            collectors[hand].poses().len()
                        );
                    }
                    CalibPose::Duplicate(_) => {
                        if !warned[hand] {
                            warned[hand] = true;
                            println!(
                                "  {} sigue en una pose ya medida: girá el Joy-Con a otra cara.",
                                hand_name(hand)
                            );
                        }
                        collectors[hand].begin_step();
                    }
                    CalibPose::Waiting => {}
                }
            }
            std::thread::sleep(Duration::from_millis(20));
        }

        for hand in [0_usize, 1] {
            if active[hand] && !done[hand] {
                eprintln!(
                    "  tiempo agotado esperando el Joy-Con {} en esta pose; esa mano no se guardará.",
                    hand_name(hand)
                );
                completed[hand] = true;
            }
        }
    }

    let mut calibration = ImuCalibration::default();
    let mut saved_any = false;
    for hand in [0_usize, 1] {
        if !active[hand] || completed[hand] {
            continue;
        }
        if collectors[hand].poses().len() < 6 {
            eprintln!(
                "[cal] {}: solo {} poses, no se guarda.",
                hand_name(hand),
                collectors[hand].poses().len()
            );
            continue;
        }
        match collectors[hand].fit() {
            Some((calib, before, after)) => {
                println!(
                    "[cal] {}: scale=({:.4},{:.4},{:.4}) offset=({:+.4},{:+.4},{:+.4})",
                    hand_name(hand),
                    calib.scale.x,
                    calib.scale.y,
                    calib.scale.z,
                    calib.offset.x,
                    calib.offset.y,
                    calib.offset.z
                );
                println!(
                    "[cal] {}: RMS |a|-g  antes {:.3} m/s² -> después {:.3} m/s²",
                    hand_name(hand),
                    before,
                    after
                );
                calibration.hands[hand] = Some(calib);
                saved_any = true;
            }
            None => eprintln!(
                "[cal] {}: el ajuste falló (poses poco diversas o medidas ruidosas). Probá de nuevo.",
                hand_name(hand)
            ),
        }
    }

    if saved_any {
        match joycon::imu_calib_path() {
            Some(path) => match calibration.save(&path) {
                Ok(()) => println!(
                    "\nGuardado en {}\nReiniciá joyreach para que lo cargue al arrancar.",
                    path.display()
                ),
                Err(err) => eprintln!("\nNo se pudo guardar {}: {err}", path.display()),
            },
            None => eprintln!("\nNo hay LOCALAPPDATA/HOME: no se pudo guardar el archivo."),
        }
    } else {
        eprintln!("\nNo se guardó ninguna calibración.");
    }

    drop(worker);
}

fn hand_name(hand: usize) -> &'static str {
    if hand == 0 { "IZQUIERDO" } else { "DERECHO" }
}

// ---------------------------------------------------------------------------------------------
// Diagnostics printing
// ---------------------------------------------------------------------------------------------

fn print_hand(label: &str, hand: &HandInput, debug: &HandDebug) {
    if !hand.connected {
        println!("{label}  disconnected");
        for note in &debug.notes {
            println!("       note: {note}");
        }
        return;
    }

    let (yaw, pitch, roll) = {
        let v = joycon::orientation_yaw_pitch_roll_deg(hand.orientation);
        (v.x, v.y, v.z)
    };

    println!(
        "{label}  connected  topology={}  pad=\"{}\"  sdl_type=\"{}\"",
        debug.topology.label(),
        debug.gamepad_name,
        debug.gamepad_type
    );

    let sensors = if debug.sensors.is_empty() {
        "none".to_string()
    } else {
        debug
            .sensors
            .iter()
            .map(|s| format!("{}@{:.0}Hz", s.name, s.rate_hz))
            .collect::<Vec<_>>()
            .join(" ")
    };
    println!(
        "       sensors: {sensors}   imu={:.1}Hz   bias={} ready={}",
        debug.imu_hz,
        fmt_vec3(debug.gyro_bias),
        debug.bias_ready
    );
    println!(
        "       raw  gyro={} accel={}   (SDL device frame)",
        fmt_vec3(debug.raw_gyro),
        fmt_vec3(debug.raw_accel)
    );
    println!(
        "       body gyro={} accel={}{}",
        fmt_vec3(debug.body_gyro),
        fmt_vec3(debug.body_accel),
        if debug.accel_calibrated {
            "  (calibrado)"
        } else {
            "  (sin calibración: ver --calibrate)"
        }
    );
    if debug.accel_calibrated {
        println!(
            "       accel calib scale={} offset={}",
            fmt_vec3(debug.accel_calib_scale),
            fmt_vec3(debug.accel_calib_offset)
        );
    }
    println!(
        "       orient yaw={:>7.1} pitch={:>7.1} roll={:>7.1} deg",
        yaw, pitch, roll
    );
    println!(
        "       reach={} (x,y,z; z- = forward)  vel={}  g_hat={:.2} m/s2",
        fmt_vec3(hand.reach * 100.0),
        fmt_vec3(hand.reach_velocity),
        debug.reach_gravity,
    );
    println!(
        "       model-explained={}  residual={} (cm; residual = dead-reckoned p_imu − model)",
        fmt_vec3(debug.reach_model_explained * 100.0),
        fmt_vec3(debug.reach_residual * 100.0),
    );
    println!(
        "       depth: {}   ({})",
        if debug.reach_enabled { "ON" } else { "OFF" },
        if debug.reach_enabled {
            if debug.accel_calibrated {
                "calibración cargada: dead reckoning calibrado"
            } else {
                "JOYREACH_REACH=1 fuerza la profundidad sin calibrar"
            }
        } else if debug.accel_calibrated {
            "desactivada por JOYREACH_REACH=0"
        } else {
            "sin calibración; corré --calibrate (JOYREACH_REACH=1 la fuerza)"
        }
    );
    println!(
        "       stick=({:+.2},{:+.2})  {}={} {}={}  face={}  menu={} click={}",
        hand.stick.x,
        hand.stick.y,
        if label.trim() == "LEFT" { "ZL" } else { "ZR" },
        if hand.trigger { 1 } else { 0 },
        if label.trim() == "LEFT" { "L " } else { "R " },
        if hand.shoulder { 1 } else { 0 },
        fmt_face(
            if label.trim() == "LEFT" {
                ["Up", "Down", "Left", "Right"]
            } else {
                ["X", "B", "Y", "A"]
            },
            hand.face
        ),
        if hand.menu { 1 } else { 0 },
        if hand.stick_click { 1 } else { 0 },
    );

    for note in &debug.notes {
        println!("       note: {note}");
    }
}

fn fmt_vec3(v: Vec3) -> String {
    format!("({:+.3},{:+.3},{:+.3})", v.x, v.y, v.z)
}

fn fmt_face(names: [&str; 4], face: [bool; 4]) -> String {
    let pressed = names
        .iter()
        .zip(face)
        .filter_map(|(name, down)| down.then_some(*name))
        .collect::<Vec<_>>();
    if pressed.is_empty() {
        "none".to_string()
    } else {
        pressed.join("+")
    }
}
