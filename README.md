# joyreach

**Dos Joy-Con como tus brazos dentro de un mundo 3D en el monitor: manos estilo VR, sin visor.**

![M0: brazos vivos sobre el banco de trabajo](docs/media/m0.png)

Cada Joy-Con (L/R) maneja un brazo virtual. El giroscopio y el acelerómetro dan la orientación de la mano (3DoF), y un modelo de brazo deduce dónde está el codo y la muñeca. Con las manos se empujan objetos con física real. La meta final es tener tus apps de escritorio (primero el navegador) como objetos dentro del mundo.

## Probarlo

Requisitos: Windows 11, Rust estable (≥ 1.97.1), Visual Studio con C++ y CMake (SDL3 se compila estático), y dos Joy-Con emparejados por Bluetooth.

```powershell
cargo run                      # el primer build compila Bevy (~25 min); después, ~20 s
cargo run --example joycon_probe -- --secs 20   # diagnóstico de los Joy-Con en consola
```

Sin hardware: `$env:JOYREACH_FAKE='1'; cargo run` anima las manos con input falso (a los 2.2 s agarra un prop y lo muestra, para capturas). `cargo run -- --screenshot shot.png` guarda una captura a los 4 s y sale. `cargo run --example joycon_probe -- --secs 4 --rumble` prueba la vibración: izquierda y después derecha, 300 ms cada una.

## Controles

| Joy-Con | Acción |
|---|---|
| Sostenlos como un control remoto: punta (ZL/ZR) hacia la pantalla, cara hacia arriba | Pose neutra de las manos |
| − o + | Recentrar el giro horizontal de ambas manos |
| Stick izquierdo / derecho | Moverse / mirar |
| ZL / ZR | Cerrar la mano: agarra el prop más cercano (contorno del color de la mano); al soltar, lo lanza |
| Mirar hacia abajo (>20°) | El torso se inclina y las manos alcanzan donde mirás |

Teclado y mouse para desarrollo: WASD, y clic para capturar el mouse (Esc lo suelta).

## Cómo funciona

- **Input (`src/joycon.rs`):** SDL3 corre en su propio hilo con el driver HIDAPI de Switch. Integra cada muestra del IMU (~200–350 Hz por Bluetooth) con su marca de tiempo en un filtro Mahony por mano. La deriva del giroscopio se calibra sola cuando el control está quieto. El juego solo lee el recurso `Hands`. El HD Rumble va en sentido inverso: el juego escribe mensajes `Rumble` y el hilo de SDL los manda al Joy-Con que corresponde (en un par combinado, graves = izquierda, agudos = derecha; `SDL_hidapi_switch.c`).
- **Brazos (`src/arms.rs`):** port del `GvrArmModel` de Google Daydream. Al inclinar el Joy-Con hacia arriba, el codo absorbe parte de la rotación y el brazo se estira, en vez de girar solo la muñeca. Cada mano es un cuerpo cinemático de avian3d que persigue su objetivo por velocidad, así los objetos reciben un empujón con inercia real. Si mirás abajo de −20°, todo el ancla de los hombros baja y avanza con la mirada.
- **Agarre (`src/grab.rs`):** en el borde de ZL/ZR busca el prop cuyo collider esté a ≤12 cm de la palma (la mano más cercana gana si ambas lo alcanzan). Mientras lo sostiene, le fija la velocidad con un límite de 400 N: la mesa puede bloquearlo y lo pesado queda atrás. Al soltar lo lanza con el promedio de la velocidad de la mano en los últimos 5 pasos × 1.6 (máx. 12 m/s). El contorno es un casco invertido del mismo mesh, sin luz, del color de la mano.
- **Mundo (`src/world.rs`):** terraza con cielo atmosférico, sol a 55° para que la sombra de la mano diga su profundidad, y física a 120 Hz.
- **Diseño:** [`DESIGN.md`](DESIGN.md) es el sistema visual. Solo las mangas van saturadas, con el rojo y el azul de cada Joy-Con. [`docs/BRIEF.md`](docs/BRIEF.md) tiene el brief técnico.

Stack: Rust, Bevy 0.20, avian3d 0.8 y SDL3 (`sdl3-sys`, estático).

## Estado

- [x] **M0 — brazos vivos:** escena, cámara, brazos con los Joy-Con reales, empujar objetos.
- [x] **M1 — agarrar y lanzar** con ZL/ZR, contorno del color de la mano, HD Rumble y torso que se inclina al mirar abajo.
- [ ] **M2 — locomoción:** saltar, escalar, flotar.
- [ ] **M3 — una ventana de escritorio** capturada (Windows.Graphics.Capture) en un panel 3D.
- [ ] **M4 — navegador embebido** (CEF) interactivo.

## Créditos

- Modelo de brazo portado de [`GvrArmModel`](https://github.com/googlevr/gvr-unity-sdk) (Google, Apache-2.0).
- Fuente [M PLUS Rounded 1c](https://fonts.google.com/specimen/M+PLUS+Rounded+1c) (OFL, `assets/fonts/OFL.txt`).
- [SDL](https://libsdl.org) (zlib) · [Bevy](https://bevy.org) · [avian](https://github.com/avianphysics/avian).
- Joy-Con reverse engineering: [dekuNukem](https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering); gyro: [GyroWiki](https://gyrowiki.jibbsmart.com).

Licencia MIT.
