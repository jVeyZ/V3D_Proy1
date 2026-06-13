# Guía de Uso - Proyecto 2: Balloon Catch 3D

## 1. Objetivo

Aplicación interactiva de Visión 3D con dos cámaras:

- Se lanza un **globo verde real** dentro del volumen visible por las cámaras estéreo.
- El sistema detecta el globo en ambas imágenes, resuelve la correspondencia izquierda/derecha y triangula su posición **XYZ** usando únicamente estereovisión en tiempo de ejecución.
- La posición 3D del globo se mapea a un mundo virtual PyBullet.
- En el mundo virtual hay un **robot/cesta digital** controlado mediante gestos de manos.
- El juego consiste en colocar el robot debajo del globo virtual cuando el globo toque el suelo.

> Nota: durante la ejecución no se usan ArUco/ChArUco ni marcadores en la escena. El archivo `calibration/stereo_charuco.npz` contiene una calibración estéreo previa de las cámaras; en runtime la posición del globo se obtiene por detección estéreo y triangulación.

## 2. Instalación

Desde la raíz del proyecto:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Dependencias principales:

- `opencv-python` / `opencv-contrib-python`: captura, segmentación HSV, calibración y triangulación.
- `numpy`: cálculo numérico.
- `pybullet`: mundo virtual / gemelo digital.
- `pynput`: control de teclado de respaldo.
- `mediapipe`: detección de manos y gestos.

## 3. Configuración de cámaras

Las cámaras pueden ser índices locales o URLs HTTP/RTSP. Ejemplos:

```python
# Dos cámaras locales
CAMERA_LEFT = 0
CAMERA_RIGHT = 1
CAMERA_GESTURE = 2

# Laptop + móvil por red
CAMERA_LEFT = 0
CAMERA_RIGHT = "http://192.168.1.50:8080/video"
CAMERA_GESTURE = 2
```

Para móvil por red, usa una app tipo IP Webcam/DroidCam/RTSP Camera, conecta móvil y ordenador a la misma WiFi y usa la URL de vídeo en `CAMERA_RIGHT` y en los scripts de captura:

```bash
python scripts/capture_charuco_stereo.py \
  --left 0 \
  --right "http://192.168.1.50:8080/video" \
  --output calibration/capture
```

## 4. Ejecución

```bash
python -m src.balloon_catch
```

La aplicación abre:

- ventana PyBullet con el mundo virtual;
- ventana `Stereo Left`;
- ventana `Stereo Right`;
- ventana `Gesture Robot`.

## 5. Flujo de demostración recomendado

1. Colocar dos cámaras con solape visual suficiente y fijarlas físicamente.
2. Capturar 15-25 pares ChArUco con `scripts/capture_charuco_stereo.py`.
3. Generar calibración estéreo con `scripts/calibrate_charuco_stereo.py` y revisar `calibration/report/STEREO_CALIBRATION_REPORT.md`.
4. Colocar ChArUco sobre el plano del juego, capturar un par en `calibration/world_charuco` y generar `calibration/world_transform.npz` con `scripts/calibrate_world_from_charuco.py`.
5. Ejecutar:

   ```bash
   python -m src.balloon_catch
   ```

6. Confirmar en consola que el modo de triangulación es `WORLD` y que no hay warning de RMS alto.
7. Pulsar `P` y seleccionar una ROI sobre el globo verde en `Stereo Left` si el HSV no segmenta bien.
8. Poner el globo cerca del suelo y pulsar `G` para fijar la referencia de suelo si la altura inicial no es estable.
9. Lanzar/dejar caer el globo verde.
10. Controlar el robot virtual con gestos para colocarlo bajo el globo antes de que toque el suelo.
11. Observar el resultado en consola:
   - `[CATCH] ¡Atrapado!`
   - `[CATCH] Fallo`

## 6. Controles

### Robot virtual

Gestos soportados por mano izquierda/derecha:

| Gesto | Comando |
| --- | --- |
| Índice levantado (`Pointing_Up`) | rueda correspondiente hacia delante |
| Victoria (`Victory`) | rueda correspondiente hacia atrás |
| Puño (`Closed_Fist`) | parar rueda correspondiente |

Controles de teclado de respaldo:

| Tecla | Acción |
| --- | --- |
| `W` | Avanzar |
| `S` | Retroceder |
| `A` | Girar izquierda |
| `D` | Girar derecha |
| `R` | Reset del robot |
| `C` | Cambiar cámara virtual |
| `G` | Calibrar altura de suelo con la medida estéreo actual |
| `P` | Recalibrar rango HSV del globo con ROI |
| `Q` | Salir |

## 7. Módulos principales

- `src/balloon_catch.py`: aplicación integrada, hilos, mundo PyBullet, lógica del juego y mapeo estéreo→mundo virtual.
- `src/balloon_tracker.py`: detección del globo verde en ambas cámaras y tracking con Kalman.
- `src/stereo.py`: carga de calibración estéreo y triangulación 3D.
- `src/gesture_robot.py`: detección/clasificación de gestos con MediaPipe y generación de comandos.
- `scripts/make_charuco_board.py`: generación de tablero de calibración offline.
- `scripts/capture_charuco_stereo.py`: captura de pares de calibración offline.
- `scripts/calibrate_charuco_stereo.py`: calibración estéreo offline con reporte.
- `scripts/calibrate_world_from_charuco.py`: calibración automática cámara izquierda → mundo usando ChArUco.
- `scripts/calibrate_world_from_points.py`: calibración manual alternativa cámara izquierda → mundo.

## 8. Cumplimiento de requisitos del Proyecto 2

### 2.1 Escena virtual compleja / gemelo digital

**Estado: cumplido con PyBullet.**

- La escena virtual contiene un campo 3D, límites, porterías decorativas, un globo virtual y un robot/cesta digital.
- El globo virtual representa el objeto real detectado por las cámaras.
- El robot digital es el agente controlado por el usuario.

Archivos:

- `src/balloon_catch.py`

### 2.2 Detección, seguimiento y correspondencia estéreo

**Estado: cumplido para un objeto real principal: el globo verde.**

- Se detecta el globo por segmentación HSV y contornos en la cámara izquierda y derecha.
- Cada cámara mantiene tracking suavizado con Kalman.
- La correspondencia izquierda/derecha se resuelve directamente porque el objeto de interés es único y está segmentado en ambas vistas.
- El usuario puede recalibrar el color con una ROI (`P`) sin usar marcadores.

Archivos:

- `src/balloon_tracker.py`
- `src/balloon_catch.py`

### 2.3 Posicionamiento 3D y actualización continua

**Estado: cumplido si existen una calibración estéreo válida y `calibration/world_transform.npz`.**

- `src/stereo.py` usa las matrices de calibración `K_l`, `K_r`, `R`, `T`, `F` y `cv2.triangulatePoints`.
- La correspondencia izquierda/derecha se valida con error epipolar antes de triangular.
- `calibration/world_transform.npz` transforma las coordenadas de cámara izquierda al sistema 3D de mundo del juego.
- La posición 3D triangulada en mundo se actualiza continuamente en PyBullet.
- El globo virtual sube/baja usando la componente `Z` y se compara con una referencia de suelo.
- No se usan ArUco/ChArUco durante la ejecución; solo durante la calibración offline.

Archivos:

- `src/stereo.py`
- `calibration/stereo_charuco.npz`
- `calibration/world_transform.npz`
- `src/balloon_catch.py`

Evidencias para la memoria:

- `calibration/report/STEREO_CALIBRATION_REPORT.md`
- `calibration/report/images/*detected*.png`
- `calibration/WORLD_TRANSFORM_REPORT.md`
- RMS estéreo, baseline y RMS de transformación mundo.

### 2.4 Interfaz humano-máquina basada en visión

**Estado: cumplido.**

- `src/gesture_robot.py` usa MediaPipe para detectar manos.
- Si existe `assets/gesture_recognizer.task`, usa el recognizer de MediaPipe Tasks.
- Si no existe, usa un fallback con `mediapipe.solutions.hands` y un clasificador geométrico propio basado en landmarks.
- Se clasifican gestos finitos y se convierten en comandos de rueda izquierda/derecha.
- Los comandos controlan directamente el robot virtual del mundo PyBullet.

Archivos:

- `src/gesture_robot.py`
- `src/balloon_catch.py`

### 2.5 Aplicación 3D integrada con procesamiento paralelo

**Estado: cumplido.**

La aplicación integra:

- hilo estéreo para cámaras, detección, tracking y triangulación;
- hilo de gestos para la interfaz humano-máquina;
- hilo principal para PyBullet, ventanas OpenCV, lógica del juego y scoring.

El juego completo consiste en coordinar un objeto real 3D medido por estereovisión con un robot virtual controlado por gestos.

## 9. Limitaciones conocidas

- La precisión 3D depende mucho de la calibración estéreo y del ajuste cámara→mundo. Si el globo virtual salta o la altura no es estable, repetir calibración.
- El sistema actual está optimizado para un único globo verde. Si hay varios objetos verdes, la correspondencia puede confundirse.
- El mapeo al mundo virtual depende de los puntos de `calibration/world_points.csv`; si esos puntos se marcan mal, el mundo queda sesgado.
- La realidad aumentada proyecta la huella del robot sobre la cámara izquierda cuando existe `world_transform.npz`.

## 10. Checklist para presentación

- [ ] Mostrar cámaras izquierda/derecha detectando el globo.
- [ ] Mostrar valores XYZ triangulados en consola/overlay.
- [ ] Mostrar el globo virtual moviéndose en PyBullet.
- [ ] Mostrar gestos controlando el robot.
- [ ] Lanzar el globo y conseguir al menos un `[CATCH] ¡Atrapado!`.
- [ ] Explicar que runtime no usa marcadores; la calibración estéreo es previa.
- [ ] Reportar RMS de calibración y limitaciones.