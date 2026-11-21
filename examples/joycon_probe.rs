//! Standalone Joy-Con probe: no Bevy window, just the SDL thread from `src/joycon.rs`.
//!
//! ```text
//! cargo run --example joycon_probe -- --secs 8
//! ```
//!
//! Prints, ~10 times per second, for each hand: connection/topology/gamepad type, enabled SDL
//! sensors and their data rate, measured IMU rate, raw (SDL device frame) and mapped (body frame)
//! gyro/accel, fused yaw/pitch/roll, stick, buttons and any SDL errors.

#[path = "../src/joycon.rs"]
#[allow(dead_code)]
mod joycon;

use std::io::Write;
use std::time::{Duration, Instant};

use bevy::math::Vec3;
use joycon::{HandDebug, HandInput, JoyConWorker};

fn main() {
    let secs = match parse_args() {
        Ok(secs) => secs,
        Err(message) => {
            eprintln!("{message}");
            print_usage();
            std::process::exit(2);
        }
    };

    let worker = JoyConWorker::spawn();

    println!(
        "Joy-Con probe: {secs:.0} s. If both hands stay disconnected, the Joy-Cons are probably \
         asleep - press a button (or press any Joy-Con button) to wake them."
    );
    println!("(Sensors only exist once SDL's HIDAPI Switch driver takes the device; SDL errors are \
             reported below.)");

    let start = Instant::now();
    let mut frame: u64 = 0;
    while start.elapsed().as_secs_f64() < secs {
        let hands = worker.snapshot();
        let debug = worker.debug_snapshot();

        println!(
            "\n[{:>5.1}s] frame {frame}",
            start.elapsed().as_secs_f64()
        );
        print_hand("LEFT ", &hands.left, &debug.left);
        print_hand("RIGHT", &hands.right, &debug.right);
        let _ = std::io::stdout().flush();

        frame += 1;
        std::thread::sleep(Duration::from_millis(100));
    }

    // Dropping the worker stops the SDL thread and calls SDL_Quit.
    drop(worker);
    println!("\nprobe finished after {:.1} s", start.elapsed().as_secs_f64());
}

fn parse_args() -> Result<f64, String> {
    let mut secs = 10.0_f64;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--secs" | "-s" => {
                let value = args
                    .next()
                    .ok_or_else(|| "--secs needs a number (seconds)".to_string())?;
                secs = value
                    .parse::<f64>()
                    .map_err(|_| format!("invalid --secs value: {value}"))?;
                if !(secs.is_finite() && secs > 0.0) {
                    return Err("--secs must be a positive number".to_string());
                }
            }
            "-h" | "--help" => {
                print_usage();
                std::process::exit(0);
            }
            other => return Err(format!("unknown argument: {other}")),
        }
    }
    Ok(secs)
}

fn print_usage() {
    println!("usage: joycon_probe [--secs N]   (default N = 10)");
}

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
        "       body gyro={} accel={}",
        fmt_vec3(debug.body_gyro),
        fmt_vec3(debug.body_accel)
    );
    println!(
        "       orient yaw={:>7.1} pitch={:>7.1} roll={:>7.1} deg",
        yaw, pitch, roll
    );
    println!(
        "       stick=({:+.2},{:+.2})  {}={} {}={}  face={}  menu={} click={}",
        hand.stick.x,
        hand.stick.y,
        if label.trim() == "LEFT" { "ZL" } else { "ZR" },
        if hand.trigger { 1 } else { 0 },
        if label.trim() == "LEFT" { "L " } else { "R " },
        if hand.shoulder { 1 } else { 0 },
        fmt_face(if label.trim() == "LEFT" {
            ["Up", "Down", "Left", "Right"]
        } else {
            ["X", "B", "Y", "A"]
        }, hand.face),
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
