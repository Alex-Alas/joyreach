# joyreach — brief de diseño

**Qué es:** brazos virtuales manejados por dos Joy-Con (L/R) dentro de un sandbox 3D que se ve en el monitor, sin visor VR. Después: ventanas de escritorio (navegador primero) flotando en el mundo.

**Origen:** video del autor (2026-11-20). Pasos pedidos, en orden de prioridad:
1. Entorno 3D visible en el PC.
2. Personaje con brazos que interactúan con el entorno. ← **lo más importante**
3. Apps de escritorio dentro del mundo (primero el navegador).

Controles pedidos: stick izq = moverse, stick der = cámara, gatillos = agarrar/interactuar, resto de botones = saltar/escalar/flotar ("lo que funcione mejor"). Sandbox interactivo.

## Stack (decidido)

| Pieza | Elección | Por qué |
|---|---|---|
| Motor | Bevy 0.20.1 (Rust) | Nativo, ECS, binario liviano, integración Win32 directa para el paso 3 |
| Física | avian3d 0.8 | Joints completos, ECS nativo |
| Input | SDL3 (`sdl3-sys` 0.7.3, estático) en hilo propio | Driver HIDAPI de Switch: IMU a 200 Hz con calibración de fábrica. `gilrs` no expone sensores |
| Paso 3 | `windows-capture` (WGC) + `bevy_cef` | Captura de ventanas a textura / navegador offscreen con input |

Descartado: motor propio (no aporta al demo), Godot (paso 3 exige GDExtension igual), Web (no integra apps de escritorio).

## Convenciones

- Bevy: derecha +X, arriba +Y, **adelante -Z**.
- `Res<Hands>` (`src/joycon.rs`) es la única interfaz entre input y juego.
- Pose neutra de cada Joy-Con = **"agarre de control remoto"**: la punta (lado ZL/ZR) apunta a la pantalla, la cara (stick) hacia arriba, índice en el gatillo. Pitch/roll absolutos (gravedad); yaw se recentra con −/+.

## Modelo de brazo (3DoF → posición), basado en Daydream `GvrArmModel`

Fuente: `googlevr/gvr-unity-sdk/.../ArmModel/GvrArmModel.cs` (Apache-2.0). Mano derecha; espejar X para la izquierda. Metros, relativo al cuello, que sigue solo el yaw del cuerpo.
- Hombro `(0.17, -0.2, -0.03)`, codo en reposo `(0.195, -0.5, 0.005)`, antebrazo `(0, 0, -0.25)` (muñeca), mano `(0, 0, -0.05)`. (En GvrArmModel +Z es adelante; aquí adelante es -Z.)
- `extensionRatio = clamp((pitch − 7°) / (60° − 7°), 0, 1)` con pitch hacia arriba; offset de extensión `(-0.13, 0.14, -0.08)·ratio` (X espejado por mano).
- Reparto de rotación: `elbowBendRatio = 0.6`, `extensionWeight = 0.4`. El codo absorbe `lerp(…)`, la muñeca el resto ⇒ el brazo se estira en vez de solo girar la muñeca.
- Sin IK en M0: cadena analítica.

## Fusión IMU

- Gyro bias: promediar ~1 s quieto al conectar y re-estimar cuando esté quieto (|ω| < ~0.03 rad/s sostenido).
- Mahony o Madgwick con acelerómetro solo para pitch/roll; yaw por gyro + recentrado manual.
- Integrar cada muestra de `SDL_EVENT_GAMEPAD_SENSOR_UPDATE` con su `sensor_timestamp` (3 muestras por reporte BT ⇒ ~200 Hz). Clamp de `dt`, descartar huecos > 50 ms.

## Feel (decisiones MVP)

1. Brazos en espacio de cuerpo, siguen el yaw del cuerpo, no el pitch de la cámara. FOV ~80°.
2. Lag mínimo: sin suavizado global; si hace falta, smoothing por tiers (JibbSmart) solo en movimientos pequeños.
3. Manos físicas (M1): cuerpo dinámico que persigue al objetivo por velocidad; agarre = seguimiento con límite de fuerza (400 N), el objeto resbala si se excede. Lanzar = velocidad promedio de la mano en los últimos ~5 frames × 1.6 (máx. 12 m/s).
4. Dedos que se cierran con el gatillo (~80 ms): vende el agarre sin costo.
5. Recentrado siempre disponible (−/+).
6. Escalada tipo Gorilla Tag (M2): mano agarrada a geometría estática mueve el cuerpo en sentido opuesto.

## Hitos

- **M0 — brazos vivos:** escena, cámara con sticks, brazos siguiendo los Joy-Con, empujar objetos físicos. Probe `examples/joycon_probe.rs`.
- **M1 — agarrar y lanzar** con ZL/ZR, contorno del color de la mano, HD Rumble, torso que se inclina. `src/grab.rs`.
- **M2 — profundidad y locomoción:**
  - *Profundidad por IMU (decidido con el autor, frente a cámara + MediaPipe o control manual):* la mano sigue el movimiento real empujando, jalando o golpeando. Por mano, en el hilo SDL y a tasa IMU: `a_lin = yaw_offset·q·accel − ĝ` (ĝ = gravedad estimada **por mano** en las ventanas de quietud: en reposo el Joy-Con R midió 8.9 m/s² y el L 10.06; restar 9.81 fijo haría derivar la mano). Banda muerta ~0.15 m/s²; `v += a·dt` con fuga de velocidad τ≈0.5 s; ZUPT (v = 0) al quedar quieto, así la posición **se queda** donde la dejaste; desplazamiento acotado (~0.35 m) y retorno lento a la pose del modelo (τ≈3–4 s) para no derivar; reset al recentrar. Se publica como un campo nuevo de `HandInput` (offset en espacio de cuerpo). El codo se resuelve con IK analítica de dos huesos hacia la muñeca desplazada, dentro del alcance real del brazo.
  - *Locomoción:* cuerpo físico (cápsula cinemática con gravedad que choca con el banco y los parapetos); saltar con B; agacharse automático al mirar hacia el piso (los ojos bajan de 1.60 a ~0.9 m entre −35° y −80° de pitch, sumado a la inclinación del torso de M1) más agacharse fijo con clic del stick izquierdo. Después: escalar, flotar.
- **M3 — ventana de escritorio** capturada (WGC) en un panel 3D, con puntero desde la mano.
- **M4 — navegador** embebido (CEF) interactivo.

## Referencias

- GyroWiki (Jibb Smart): player-space gyro, soft-tiered smoothing — https://gyrowiki.jibbsmart.com
- GamepadMotionHelpers (calibración continua) — https://github.com/JibbSmart/GamepadMotionHelpers
- Joy-Con reverse engineering (dekuNukem) — https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering
- SDL HIDAPI Switch driver — https://github.com/libsdl-org/SDL/blob/main/src/joystick/hidapi/SDL_hidapi_switch.c
- Half-Life: Alyx / Boneworks (manos físicas), Gorilla Tag (escalada), Human: Fall Flat (un botón por mano).
