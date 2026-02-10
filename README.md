# Posicionamiento 3D Planar — Mini-Golf Virtual

Proyecto de Visión por Computador (V3D) — Universidad Politécnica de Valencia.

Aplicación interactiva de posicionamiento planar con una sola cámara que integra
detección, seguimiento y posicionamiento de objetos reales sobre un plano de
trabajo, combinado con un juego de mini-golf virtual.

---

## Estructura del Proyecto

```
proyecto_posicionamiento/
├── config.py            # Configuración global (cámara, colores, dimensiones)
├── calibration.py       # Calibración con homografía (manual + ArUco)
├── detection.py         # Detección manual y automática (HSV + contornos)
├── tracking.py          # Seguimiento de objetos (color + OpenCV trackers)
├── positioning.py       # Posicionamiento planar + corrección de altura
├── virtual_scene.py     # Escena virtual con Open3D (gemelo digital)
├── ar_viewer.py         # Visor de realidad aumentada
├── game_engine.py       # Motor del juego de mini-golf
├── demo_camera.py       # Cámara simulada para pruebas sin hardware
├── main.py              # Aplicación principal
├── requirements.txt     # Dependencias Python
└── README.md            # Este archivo
```

## Apartados del proyecto
Archivo |	Función |	Apartado
------- | ------- | ------- |
config.py	|	Configuración global (cámara, colores, dimensiones, juego)	|	Transversal |
calibration.py	|	Homografía imagen↔plano (manual + ArUco)	|	1.3 |
detection.py	|	Detección manual (clic) y automática (HSV + contornos + Hough)	|	1.2 |
tracking.py	|	Seguimiento por color (ROI adaptativa) y OpenCV (CSRT/KCF)	|	1.2 |
positioning.py	|	Posicionamiento planar + corrección de altura	|	1.3 + 1.4 |
virtual_scene.py	|	Gemelo digital 3D (Open3D) + vista 2D alternativa	|	1.1 |
ar_viewer.py	|	Superposición AR de hoyo, trayectoria, puntuación	|	1.3 |
game_engine.py	|	Motor de mini-golf (niveles, golpes, embocada, colisiones)	|	1.5 |
demo_camera.py	|	Cámara simulada con pelota movible (ratón/teclado)	|	Pruebas |
main.py	|	Aplicación principal que integra todo el pipeline	|	1.5 |
README.md	|	Documentación del proyecto	|	— |

---

## Requisitos

- Python 3.8+
- OpenCV (con contrib)
- NumPy
- Open3D (opcional, para escena 3D)

### Instalación

```bash
pip install -r requirements.txt
```

---

## Uso

### Modo demo (sin cámara)

```bash
python main.py --demo --auto-calibrate --auto-detect
```

### Modo cámara real

```bash
python main.py
```

### Opciones

| Argumento | Descripción |
|-----------|-------------|
| `--demo` | Cámara simulada (sin hardware) |
| `--auto-calibrate` | Calibración automática (ArUco o esquinas demo) |
| `--auto-detect` | Detección automática por color |
| `--no-3d` | Desactivar escena Open3D |
| `--camera N` | Índice de la cámara (por defecto 0) |
| `--tracker {color,csrt,kcf,mosse}` | Método de seguimiento |
| `--correct-height` | Aplicar corrección de altura (apt. 1.4) |

### Controles en ejecución

| Tecla | Acción |
|-------|--------|
| `q` / `ESC` | Salir |
| `c` | Recalibrar cámara |
| `d` | Cambiar detección (manual ↔ auto) |
| `f` | Forzar re-detección |
| `r` | Reiniciar nivel |
| `n` | Nueva partida |
| `v` | Toggle escena 3D |
| `h` | Mostrar/ocultar ayuda |
| `e` | Explicar corrección de altura |
| `WASD` | Mover pelota (modo demo) |
| `t` | Toggle modo automático (modo demo) |

---

## Apartados del Proyecto

### 1.1 — Escena Virtual (Gemelo Digital)

**Archivo:** `virtual_scene.py`

Se implementan dos visores del gemelo digital:

- **VirtualScene** (Open3D): escena 3D interactiva con mesa, pelota, hoyo,
  bandera, obstáculos y trayectoria. Ejecuta en un hilo separado.
- **SimpleVirtualScene**: vista cenital 2D renderizada con OpenCV como
  alternativa ligera.

Elementos representados:
- Mesa/green (objeto real → representación virtual)
- Pelota (objeto real → posición actualizada desde la cámara)
- Hoyo + bandera (objetos virtuales)
- Obstáculos (virtuales, configurables)
- Trayectoria de la pelota (virtual)

### 1.2 — Detección y Seguimiento

**Archivos:** `detection.py`, `tracking.py`

**Detección** (solo primer frame):
- **Manual** (`ManualDetector`): el usuario hace clic en el centro de la pelota
  y se estima el radio por análisis HSV.
- **Automática** (`AutomaticDetector`): umbralización HSV, operaciones
  morfológicas, detección de contornos con filtrado por circularidad y ratio de
  llenado, con fallback a HoughCircles.

**Seguimiento** (frames siguientes):
- **ColorTracker**: seguimiento por color HSV con ROI adaptativa y predicción
  de velocidad.
- **OpenCVTracker**: wrapper sobre CSRT, KCF o MOSSE de OpenCV.

### 1.3 — Posicionamiento Planar

**Archivos:** `calibration.py`, `positioning.py`

- Calibración mediante homografía (4 correspondencias imagen ↔ plano).
- Modos: manual (clic en esquinas) o automático (marcadores ArUco).
- Transformación píxel → coordenadas mundo (cm) sobre el plano Z=0.
- Transformación inversa para proyectar elementos virtuales en la imagen (AR).
- Error de reproyección calculado y mostrado.

### 1.4 — Corrección de Altura

**Archivo:** `positioning.py` → método `correct_height()`

Cuando el centro del objeto no está sobre el plano (por ejemplo, una pelota
esférica cuyo centro está a altura h = radio), se corrige la posición usando
la geometría del rayo de proyección:

$$
k = \frac{C_z}{C_z - h}
$$

$$
X_{\text{real}} = C_x + k \cdot (X_{\text{aparente}} - C_x)
$$

$$
Y_{\text{real}} = C_y + k \cdot (Y_{\text{aparente}} - C_y)
$$

Donde $(C_x, C_y, C_z)$ es la posición estimada de la cámara y $h$ es la
altura del centro del objeto sobre el plano.

### 1.5 — Aplicación Integrada (Mini-Golf)

**Archivos:** `game_engine.py`, `main.py`

Juego interactivo de mini-golf tipo "putter" sobre un green plano:
- La pelota real se detecta y sigue sobre la mesa.
- Hoyos virtuales se proyectan mediante realidad aumentada.
- Se detecta cuándo la pelota se detiene (conteo de golpes).
- Se detecta embocada (pelota sobre el hoyo).
- Progresión de niveles con diferentes posiciones del hoyo.
- Puntuación acumulada.

---

## Cámara Simulada (Demo)

**Archivo:** `demo_camera.py`

Permite probar el pipeline completo sin hardware:
- Genera frames sintéticos con perspectiva simulada.
- Pelota movible con ratón (arrastrar) o teclado (WASD).
- Modo automático con trayectoria predefinida.
- Compatible con la interfaz de `cv2.VideoCapture`.
- Marcadores de referencia en las esquinas para calibración.

---

## Configuración

Todos los parámetros ajustables están en `config.py`:
- Resolución de cámara y FPS
- Dimensiones del área de juego (cm)
- Rangos HSV para detección de la pelota
- Parámetros de seguimiento
- Posiciones de hoyos y obstáculos
- Colores de la escena virtual y AR


