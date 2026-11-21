# DESIGN.md — joyreach

Sistema visual y de experiencia. Lo técnico vive en `docs/BRIEF.md`; esto define **cómo se ve, se siente y se lee**. Toda pieza visual nueva (escena, prop, HUD, ventana) se valida contra este archivo.

## 1. Lectura del brief

- **Tema:** tus dos Joy-Con convertidos en tus brazos, dentro de un espacio 3D que ves en el monitor. Más adelante ese espacio aloja tus apps de escritorio.
- **Público y contexto:** una persona sentada frente a su monitor, de noche, en su cuarto, con un Joy-Con en cada mano (el video original: cuarto con closet de madera, paredes verde menta, puerta crema, luz cálida de lámpara).
- **Trabajo de la imagen:** que sepas **dónde está tu mano en profundidad** y **qué vas a agarrar** sin visión estéreo.
- **Restricciones:** pantalla plana (sin estéreo), orientación 3DoF (posición inferida), Bevy 0.20 PBR, laptop RTX 3050.

**Fuentes del mundo del tema** (de aquí sale cada decisión):
1. El Joy-Con: rojo neón y azul neón oficiales, formas de cápsula, botones circulares, rieles laterales.
2. El cuarto del video: madera de closet, pared verde menta, puerta crema, luz cálida de lámpara.
3. La UI del sistema de Switch: tipografía redondeada japonesa, íconos de botón como formas físicas.

## 2. Principio rector

**Las manos son lo único saturado.** El mundo es cálido y de saturación baja; las mangas llevan el color exacto de cada Joy-Con. Lo que tienes en la mano física y lo que ves en pantalla comparten identidad: rojo = izquierda, azul = derecha, siempre, en todo (mangas, resaltes, cursores, HUD).

**El movimiento audaz:** el color de la mano marca lo que va a tocar. El prop al alcance de la mano izquierda recibe un contorno rojo; el de la derecha, uno azul. Color = qué mano y qué objeto, en un solo gesto.

## 3. Color

Hex en sRGB. Valor (L) aproximado para planear contraste.

| Token | Hex | Rol | Origen |
|---|---|---|---|
| `joy.left` | `#FF3C28` | Manga izq., contorno de agarre izq., cursor izq. | Joy-Con neón rojo |
| `joy.right` | `#0AB9E6` | Manga der., contorno de agarre der., cursor der. | Joy-Con neón azul |
| `glove` | `#EEE9DF` | Manos (guante mate) | Claro, contrasta con la madera de la mesa |
| `deck` | `#7A5A3E` | Piso de madera (L≈40%) | Closet del video |
| `walnut` | `#4E3627` | Mesa de trabajo (L≈25%) | Madera oscura: fondo para leer las manos |
| `celadon` | `#C6D7CB` | Parapetos y muros bajos (L≈84%) | Pared menta del video |
| `cream` | `#E9E1CF` | Marcos, puertas, ventanas (paneles) | Puerta del video |
| `ink` | `#2B2A28` | Texto HUD, biseles de paneles | Cálido, no negro puro |

Props agarrables (saturación media, **nunca rojo ni azul** para no competir con las manos): mostaza `#D9A13B`, salvia `#7E9F78`, ciruela `#7A5C85`, piedra `#A89F8E`, carbón `#3B3C3D`. (Tiza `#EDE8DD` se descartó: tenía el mismo valor que el guante y en pantalla se leía como una mano.)

Reglas: ningún color del mundo supera la saturación de `joy.*`. Sin degradados decorativos; los degradados vienen solo de la luz.

## 4. Luz y atmósfera

- **Hora:** tarde avanzada, no atardecer de postal. Sol a **~55° de elevación**: las sombras caen casi debajo de los objetos y la sombra de la mano sobre la mesa dice su profundidad (la señal de distancia más fuerte sin estéreo).
- Sol cálido (~5200 K) + cielo de `Atmosphere` como relleno frío. Sombras en cascada con buen rango cercano: la mesa (0–2 m) es lo que más importa.
- **Señales de profundidad obligatorias** en el área de alcance: sombra de la mano nítida, SSAO para contacto, niebla leve solo en la lejanía (>15 m) para separar planos.
- Tonemapping Tony McMapface, exposición `ev100 13.6` con sol `0.85 × RAW_SUNLIGHT`. AgX se descartó en la primera captura: lavaba el neón de las mangas a salmón/pastel y rompía el principio rector. Bloom muy bajo, solo para cursores emisivos.
- Lo que no se hace: noche/neón, viñeta fuerte, aberración cromática, grano. Restan legibilidad de la mano.

## 5. Espacio (M0)

Terraza de madera `deck`, ~30×30 m, con parapetos bajos `celadon` en los bordes (límite + referencia de escala). Frente al spawn, a ~0.9 m, la mesa `walnut` (0.75 m de alto) con los props. Cerca: una pila de cajas y 2–3 pelotas grandes en el piso. Horizonte abierto: cielo y nada más; el detalle se concentra a distancia de brazo.

Escala humana real (metros). Props de 8–15 cm: del tamaño de algo que agarras con una mano.

## 6. Cámara

- Ojos a 1.60 m, **FOV 80°**, sin head-bob (pelea con la precisión del brazo).
- Los brazos ocupan el **25–35 % inferior** del cuadro en reposo; la mesa entra entera al mirar ~20° abajo.
- **Agacharse automático:** al mirar abajo, los ojos bajan de 1.60 a ~0.90 m entre −35° y −80° de pitch (smoothstep, ~150 ms, sin head-bob). Las manos bajan con ellos, así que la franja de los brazos en el cuadro no cambia: cambia su altura en el mundo. El torso que se inclina en §7 se suma encima.
- **Agacharse fijo:** clic del stick izquierdo (o C/Ctrl) deja los ojos a 0.95 m, el paso a ~1.4 m/s y la cápsula a 1.10 m; si hay algo arriba no se puede parar. Sin head-bob al entrar ni al salir.
- **Salto:** B (Joy-Con R) o Space, ~0.6 m de alto, solo desde el piso (0.1 s de coyote). La cámara no gana ni shake ni sonido: solo acompaña al cuerpo.
- Giro con stick: curva cuadrática; nunca aceleración de mouse.

## 7. Brazos y manos

- Mangas: cápsulas con el color `joy.*` mate (roughness ~0.6). Antebrazo más delgado que el brazo superior.
- Manos: guante `glove` mate, palma + 4 dedos + pulgar. Formas redondeadas, como el propio Joy-Con: nada de aristas vivas.
  Malla del `generic-hand` de WebXR (W3C, crédito en README), material reemplazado por `glove`; las juntas de los dedos se articulan por gatillo y el pulgar queda hacia adentro.
- **Gatillo = puño:** los dedos se cierran en ~80 ms con ease-out; al soltar, se abren en ~120 ms. Es la confirmación visual del agarre.
- **Profundidad (M2a, por IMU):** la mano sigue el empuje/jale/golpe real. En el hilo de SDL, a tasa IMU: `a = yaw·q·accel − ĝ·Y`, con `ĝ` medida por mano en las ventanas de quietud (el Joy-Con L mide ~10.1 m/s² y el R ~8.9; restar 9.81 fijo haría derivar). Banda muerta 0.15 m/s², fuga de velocidad τ = 0.5 s, ZUPT (v = 0) tras 60 ms quieto para que la mano **quede** donde la dejaste, tope de 0.35 m y retorno lento a la pose del modelo (τ = 3.5 s). Se publica como `HandInput::reach` y se resetea al recentrar. El codo se resuelve con IK analítica de dos huesos (ley de cosenos, polo abajo/afuera) hacia la muñeca desplazada, dentro del alcance real del brazo.
- **Alcance:** mirar por debajo de −20° inclina el torso (hombros y manos bajan 35 cm y avanzan 25 cm, suavizado ~100 ms) para llegar a lo que muestra la cámara. A 0° el desplazamiento es cero.
- Desconectado: el brazo queda en reposo y la manga baja a 50 % de saturación. Sin mensajes de error en el mundo.
- Cerca de la cámara: se oculta el **brazo superior** mientras su hombro esté a menos de ~0.3 m del ojo (a 20 cm una manga de 10 cm ocupa un cuadro entero abajo a los lados); reaparece a ~0.5 m al inclinarse. La mano se desvanece si se acerca a la cara (escala), no atraviesa la cámara.
- **Agarre y peso:** al cerrar la mano el prop se acomoda en la palma en ~80 ms (centro a media medida bajo la palma, sobre la normal), en vez de quedar a 12–14 cm. Masas reales: props 0.2–0.6 kg, cajas ~3 kg, pelotas grandes ~0.4 kg. Límite de fuerza 60 N: las cajas se retrasan visiblemente y lo liviano va pegado. Lanzar = velocidad de la mano × 1.1 (máx. 12 m/s).

## 8. Contorno de agarre

- Prop dentro del radio de agarre (~12 cm de la palma) → contorno de 2 px en `joy.left`/`joy.right` según la mano. Si ambas manos lo alcanzan, gana la más cercana.
- Técnica: casco invertido (copia del mesh 4 mm más grande, caras traseras, material sin luz). El grosor es constante en todos los props, no un %.
- Agarrado: contorno sólido. Al soltar/lanzar, desaparece en 150 ms.
- Un solo contorno por mano a la vez.

## 9. Retroalimentación (háptica y sonido)

Los Joy-Con tienen HD Rumble (SDL `SDL_RumbleGamepad`); úsalo como canal principal, no como adorno.
- Tocar un prop: pulso ∝ velocidad de impacto (10–40 ms, con 120 ms de enfriamiento por mano).
- Agarrar: pulso firme de 40 ms. Lanzar: tick de 15 ms.
- En un par combinado, SDL separa los dos Joy-Con por frecuencia: graves = izquierda, agudos = derecha (`SDL_hidapi_switch.c:1783-1790` + `SDL_hidapi_combined.c:109-120`). Un Joy-Con suelto usa ambos actuadores.
- Sonido: golpes cortos de madera/goma según material del prop. Sin música en M0.

## 10. HUD

- **Tipografía:** M PLUS Rounded 1c (OFL). Por qué: la UI de Switch usa sans redondeada japonesa y sus terminales redondos repiten la forma del Joy-Con. Un peso para texto (500), uno para estado (700). Tamaños: 14 / 18 px a 1080p.
- **Mapeo espacial:** lo de la mano izquierda vive abajo a la izquierda; lo de la derecha, abajo a la derecha. Como en el control físico.
- **Estado de conexión:** silueta mínima de cada Joy-Con (rectángulo de cápsula) rellena con su color si está conectado, solo contorno si no, con los Hz debajo. Sin texto "conectado".
- **Pistas de botones:** íconos con la forma del botón real (círculo para −/+, círculo con anillo para stick), no letras entre corchetes. Se desvanecen tras 10 s sin uso y vuelven al presionar −/+.
- Texto `ink` sobre una placa `cream` al 85 %, radio 6 px. Sin glassmorphism ni blur.
- Copia en español, imperativa y corta: «Apunta a la pantalla y presiona − / + para recentrar».

## 11. Ventanas de escritorio (M3–M4)

Las apps son **objetos físicos**, no hologramas: losas de 15 mm de espesor con bisel `ink` y marco `cream`, ~1.2 m de ancho a ~1.8 m de distancia, inclinación solo en yaw. Debajo, una barra de agarre: se mueven con la mano como cualquier prop.
- Nitidez antes que tamaño: ~0.6–0.7 m por cada 1000 px de ancho, supersampling 1.4×.
- Puntero: rayo desde la mano, cursor = punto emisivo del color de esa mano. Clic = gatillo con 150 ms de hover estable.
- Nada de cristal esmerilado ni bordes con glow.

## 12. Anti-objetivos (lo que haría cualquier otro "sandbox de manos")

| Default | En su lugar |
|---|---|
| Texturas de prototipo gris con cuadrícula | Madera y menta del cuarto del video |
| Manos azul holograma / sci-fi | Guante mate + mangas color Joy-Con |
| Ventanas de vidrio esmerilado con glow | Losas físicas con bisel y barra de agarre |
| Vacío oscuro con niebla | Terraza de tarde con sombras que leen profundidad |
| HUD en monoespaciada con etiquetas en mayúsculas | Redondeada, mapeada a cada mano, con siluetas de botones |

## 13. Checklist de validación (por captura)

1. Squint: ¿lo primero que se ve son las dos manos?
2. Escala de grises: ¿las manos se separan de la mesa y del piso por valor?
3. ¿La sombra de cada mano se ve sobre la mesa al alcanzar un prop?
4. ¿Hay algo rojo o azul en el mundo que no sea una mano? Si sí, sobra.
5. ¿El HUD se lee sin tapar la mesa?
6. Tapa el título: ¿podría ser otro demo de manos VR? Nombra tres decisiones propias de este.
