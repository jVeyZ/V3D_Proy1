# Balloon Catch 3D - Proyecto 2 V3D

Aplicación de estereovisión 3D para el Proyecto 2 de Visión 3D.

El juego consiste en lanzar un **globo verde real**, reconstruir su posición **XYZ** con dos cámaras estéreo y controlar un **robot/cesta virtual** mediante gestos para colocarlo debajo del globo cuando toque el suelo.

## Qué cubre del Proyecto 2

- **2.1 Gemelo digital:** mundo 3D en PyBullet con globo real reconstruido y robot virtual.
- **2.2 Detección + correspondencias:** detección HSV del globo en ambas cámaras y correspondencia izquierda/derecha validada con error epipolar.
- **2.3 Estereovisión 3D:** calibración estéreo ChArUco, triangulación con `cv2.triangulatePoints`, transformación a sistema de mundo y reporte de errores.
- **2.4 Interfaz humano-máquina:** gestos con MediaPipe para controlar el robot virtual.
- **2.5 Integración multihilo:** hilo estéreo, hilo gestos y bucle principal PyBullet/OpenCV.

> Importante: durante el juego no se usan ArUco/ChArUco en la escena. ChArUco solo se usa **offline** para calibrar las cámaras y definir la geometría antes de ejecutar la demo.

---

## 1. Requisitos

- Python 3.9+ recomendado.
- Dos cámaras USB o un par estéreo.
- Una cámara para gestos, que puede ser una de las dos anteriores.
- Tablero ChArUco impreso para calibración offline.
- Un globo verde.

Instalación:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Si estás en Windows:

```bash
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```

---

## 2. Configurar cámaras

Editar `src/game_config.py` si tus índices no coinciden:

```python
CAMERA_LEFT = 0
CAMERA_RIGHT = 1
CAMERA_GESTURE = 2
```

Puedes probar índices con OpenCV o cambiarlos hasta que `Stereo Left`, `Stereo Right` y `Gesture Robot` abran las cámaras correctas.

### 2.1 Usar una cámara de laptop + un móvil por red

El proyecto permite que una fuente de cámara sea un índice local (`0`, `1`, `2`, ...) y la otra sea una URL HTTP/RTSP. Esto es útil si usas:

- cámara izquierda: webcam/laptop;
- cámara derecha: móvil emitiendo vídeo por WiFi.

#### Opción Android recomendada: IP Webcam

1. Instala en el móvil una app tipo **IP Webcam** o equivalente.
2. Conecta el móvil y el ordenador a la **misma red WiFi**.
3. Abre la app y empieza el servidor de cámara.
4. La app mostrará una URL parecida a:

```text
http://192.168.1.50:8080
```

5. En OpenCV normalmente se usa el endpoint de vídeo:

```text
http://192.168.1.50:8080/video
```

6. Prueba la captura estéreo así:

```bash
python scripts/capture_charuco_stereo.py \
  --left 0 \
  --right "http://192.168.1.50:8080/video" \
  --output calibration/capture
```

#### Otras apps

Según la app, la URL puede cambiar:

| App/tipo | URL típica |
| --- | --- |
| IP Webcam Android | `http://IP:8080/video` |
| DroidCam | `http://IP:4747/video` o cámara virtual local |
| RTSP Camera | `rtsp://IP:PUERTO/ruta` |
| Iriun/Camo/EpocCam | suele aparecer como cámara virtual local (`0`, `1`, `2`, ...) |

#### Configurar el juego con móvil por red

En `src/game_config.py` puedes dejar la laptop como izquierda y el móvil como derecha:

```python
CAMERA_LEFT = 0
CAMERA_RIGHT = "http://192.168.1.50:8080/video"
CAMERA_GESTURE = 2
```

Después ejecuta normalmente:

```bash
python -m src.balloon_catch
```

#### Recomendaciones importantes para estéreo con móvil

- Fija físicamente el móvil: no debe moverse después de calibrar.
- Usa WiFi estable y buena iluminación.
- Evita resoluciones muy altas si hay latencia.
- Repite la calibración si cambias posición, zoom, app, resolución o enfoque.
- La captura por WiFi no es perfectamente síncrona; para un globo cayendo rápido puede introducir error. Si la demo es inestable, usa movimiento más lento, más luz o dos cámaras USB.

---

## 3. Generar tablero ChArUco

Desde la raíz del proyecto (`V3D_Proy`):

```bash
python scripts/make_charuco_board.py --output calibration/charuco_5x7_4cm.png
```

Imprimir `calibration/charuco_5x7_4cm.png` al 100% de escala. No reescalar en la impresora.

Parámetros por defecto:

- `squares_x = 5`
- `squares_y = 7`
- `square_len_cm = 4.0`
- `marker_len_cm = 3.0`

---

## 4. Capturar imágenes de calibración estéreo

Captura pares sincronizados o lo más simultáneos posible:

Con dos cámaras locales:

```bash
python scripts/capture_charuco_stereo.py --left 0 --right 1 --output calibration/capture
```

Con laptop + móvil por red:

```bash
python scripts/capture_charuco_stereo.py \
  --left 0 \
  --right "http://192.168.1.50:8080/video" \
  --output calibration/capture
```

Controles:

- `s`: guardar un par izquierda/derecha.
- `q` o `ESC`: salir.

Recomendaciones para buena calibración:

- Capturar **15-25 pares válidos** como mínimo.
- Cubrir centro, bordes y esquinas de ambas imágenes.
- Variar distancia e inclinación del tablero.
- Evitar motion blur.
- Que el tablero se vea completo y nítido en ambas cámaras.
- Mantener fijas las cámaras después de calibrar.

Se generarán imágenes en:

```text
calibration/capture/left/left_000.png
calibration/capture/right/right_000.png
```

Estas imágenes deben incluirse como evidencia en la memoria.

---

## 5. Calibrar el par estéreo

Ejecutar:

```bash
python scripts/calibrate_charuco_stereo.py \
  --input calibration/capture \
  --output calibration/stereo_charuco.npz \
  --report-dir calibration/report
```

El script genera:

```text
calibration/stereo_charuco.npz
calibration/report/STEREO_CALIBRATION_REPORT.md
calibration/report/images/left_detected_*.png
calibration/report/images/right_detected_*.png
```

El `.npz` contiene:

- `K_l`, `K_r`: matrices intrínsecas.
- `dist_l`, `dist_r`: distorsión.
- `R`, `T`: pose relativa de la cámara derecha respecto a la izquierda.
- `E`, `F`: matriz esencial y fundamental.
- `rms_left`, `rms_right`, `rms_stereo`: errores de calibración.

### Criterio de aceptación recomendado

Antes de presentar, revisar:

```text
calibration/report/STEREO_CALIBRATION_REPORT.md
```

Objetivo:

- `rms_stereo < 2 px` como mínimo razonable.
- Idealmente `rms_stereo < 1 px`.

Si `rms_stereo` es alto, repetir la captura de pares. Un RMS grande hará que la triangulación XYZ sea inestable y no cumple bien el apartado 2.3.

---

## 6. Calibrar el sistema de referencia del mundo

La triangulación estéreo devuelve inicialmente coordenadas en el sistema de la **cámara izquierda**. El proyecto pide posicionar respecto a un **sistema 3D de referencia del mundo**, así que hay que calcular una transformación:

```text
X_world = R_world_from_left @ X_left_camera + t_world_from_left
```

Hay dos formas de hacerlo:

- **Automática con ChArUco**: recomendada.
- **Manual con `world_points.csv`**: alternativa si no queréis usar ChArUco para esta fase offline.

### 6.A Método automático recomendado: ChArUco como sistema mundo

Coloca el tablero ChArUco impreso **plano sobre el suelo/mesa del juego**, en la posición que quieras definir como origen del mundo. La esquina/origen del tablero será el origen `(0,0,0)` salvo que uses offsets.

Captura un par estéreo con el tablero colocado en el área de juego:

Con dos cámaras locales:

```bash
python scripts/capture_charuco_stereo.py --left 0 --right 1 --output calibration/world_charuco
```

Con laptop + móvil por red:

```bash
python scripts/capture_charuco_stereo.py \
  --left 0 \
  --right "http://192.168.1.50:8080/video" \
  --output calibration/world_charuco
```

Pulsa `s` para guardar una pareja, por ejemplo:

```text
calibration/world_charuco/left/left_000.png
calibration/world_charuco/right/right_000.png
```

Genera automáticamente `world_transform.npz`:

```bash
python scripts/calibrate_world_from_charuco.py \
  --stereo-calibration calibration/stereo_charuco.npz \
  --left-image calibration/world_charuco/left/left_000.png \
  --right-image calibration/world_charuco/right/right_000.png \
  --output calibration/world_transform.npz \
  --report calibration/WORLD_TRANSFORM_REPORT.md
```

El script detecta las esquinas ChArUco en ambas imágenes, triangula sus posiciones 3D con la calibración estéreo y calcula la transformación cámara izquierda → mundo automáticamente.

Si el tablero no está exactamente en el origen del campo, usa offsets:

```bash
python scripts/calibrate_world_from_charuco.py \
  --stereo-calibration calibration/stereo_charuco.npz \
  --left-image calibration/world_charuco/left/left_000.png \
  --right-image calibration/world_charuco/right/right_000.png \
  --world-offset-x-cm 10 \
  --world-offset-y-cm 5 \
  --world-offset-z-cm 0
```

También genera imágenes anotadas en:

```text
calibration/world_report/images/world_left_detected.png
calibration/world_report/images/world_right_detected.png
```

### 6.B Método manual alternativo: puntos conocidos

Copiar la plantilla:

```bash
cp calibration/world_points_template.csv calibration/world_points.csv
```

Editar `calibration/world_points.csv` y sustituir los placeholders por correspondencias reales:

```csv
world_x_cm,world_y_cm,world_z_cm,left_u,left_v,right_u,right_v
0,0,0,123,456,120,455
60,0,0,234,450,230,449
60,40,0,260,300,255,299
0,40,0,100,310,96,309
30,20,20,180,250,176,249
```

Cada fila contiene:

- `world_x_cm`, `world_y_cm`, `world_z_cm`: coordenada real conocida en el mundo del juego.
- `left_u`, `left_v`: píxel del mismo punto en la cámara izquierda.
- `right_u`, `right_v`: píxel homólogo en la cámara derecha.

Recomendaciones:

- Usar al menos 4 puntos no colineales.
- Mejor 5-8 puntos.
- Incluir las 4 esquinas del área de juego sobre el suelo (`z=0`).
- Añadir al menos un punto con altura conocida (`z>0`) para fijar bien el eje vertical.
- No hace falta usar marcadores durante el juego; estos puntos son solo para calibración offline.

### 6.B.2 Generar `world_transform.npz`

```bash
python scripts/calibrate_world_from_points.py \
  --stereo-calibration calibration/stereo_charuco.npz \
  --points calibration/world_points.csv \
  --output calibration/world_transform.npz \
  --report calibration/WORLD_TRANSFORM_REPORT.md
```

El script genera:

```text
calibration/world_transform.npz
calibration/WORLD_TRANSFORM_REPORT.md
```

Revisar:

```text
World RMS residual [cm]
```

Objetivo orientativo:

- `< 2 cm`: bueno para demo.
- `< 1 cm`: muy bueno.
- `> 3-5 cm`: revisar puntos o recalibración estéreo.

---

## 7. Ejecutar el juego

Cuando existan estos dos archivos:

```text
calibration/stereo_charuco.npz
calibration/world_transform.npz
```

lanzar:

```bash
python -m src.balloon_catch
```

Ventanas esperadas:

- `Stereo Left`
- `Stereo Right`
- `Gesture Robot`
- PyBullet GUI

Controles de respaldo:

| Tecla | Acción |
| --- | --- |
| `W` | avanzar robot |
| `S` | retroceder robot |
| `A` | girar izquierda |
| `D` | girar derecha |
| `R` | reset robot |
| `C` | cambiar cámara virtual |
| `P` | recalibrar HSV del globo seleccionando ROI |
| `G` | fijar referencia de suelo con altura actual |
| `Q` | salir |

Gestos:

| Mano | Gesto | Acción |
| --- | --- | --- |
| izquierda | índice levantado | rueda izquierda adelante |
| izquierda | victoria | rueda izquierda atrás |
| izquierda | puño | parar rueda izquierda |
| derecha | índice levantado | rueda derecha adelante |
| derecha | victoria | rueda derecha atrás |
| derecha | puño | parar rueda derecha |

---

## 8. Qué debe verse en la demo

1. En `Stereo Left` y `Stereo Right` debe verse el globo detectado.
2. La consola debe imprimir posiciones `X`, `Y`, `Z`.
3. Si `world_transform.npz` está cargado, el modo mostrado será `WORLD`.
4. En la imagen izquierda se proyecta una huella del robot virtual como overlay AR (`ROBOT AR`).
5. En PyBullet se mueve el globo virtual según la triangulación estéreo.
6. Al tocar el suelo, la consola muestra:

```text
[CATCH] ¡Atrapado!
```

o

```text
[CATCH] Fallo
```

---

## 9. Validación técnica para la memoria

Incluir en la memoria:

- Capturas de `calibration/capture/left` y `right`.
- `calibration/report/STEREO_CALIBRATION_REPORT.md`.
- `calibration/report/images/*detected*.png`.
- `calibration/WORLD_TRANSFORM_REPORT.md`.
- RMS estéreo.
- Baseline.
- RMS de transformación mundo.
- Explicación de la validación epipolar:

```text
x_right.T @ F @ x_left ≈ 0
```

El código rechaza correspondencias cuyo error epipolar supera:

```python
EPIPOLAR_MAX_ERROR_PX = 3.0
```

---

## 10. Solución de problemas

### El RMS estéreo es alto

Repetir captura de calibración:

- más pares;
- tablero nítido;
- cubrir todo el campo visual;
- evitar pares donde ChArUco se detecta parcialmente;
- no mover cámaras después de calibrar.

### El modo aparece como `CAM`, no `WORLD`

Falta:

```text
calibration/world_transform.npz
```

Ejecutar el paso 6.

### El globo salta mucho

- Recalibrar estéreo.
- Ajustar HSV con `P`.
- Mejorar iluminación.
- Reducir objetos verdes en escena.
- Revisar error epipolar en overlay.

### No se abre PyBullet o MediaPipe

Instalar dependencias:

```bash
pip install -r requirements.txt
```

---

## 11. Estructura relevante

```text
src/balloon_catch.py              # aplicación integrada
src/balloon_tracker.py            # detección/tracking del globo
src/stereo.py                     # calibración, epipolaridad, triangulación y proyección
src/gesture_robot.py              # gestos MediaPipe
scripts/make_charuco_board.py     # generar tablero
scripts/capture_charuco_stereo.py # capturar pares estéreo
scripts/calibrate_charuco_stereo.py # calibración estéreo + reporte
scripts/calibrate_world_from_charuco.py # cámara izquierda → mundo automático con ChArUco
scripts/calibrate_world_from_points.py # cámara izquierda → mundo manual
calibration/stereo_charuco.npz
calibration/world_transform.npz
```
