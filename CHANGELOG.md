# CHANGELOG


---

### `config.py` — Integrado: 2026-02-04
- Descripción: centralización de parámetros de cámara, juego y detector; valores por defecto para desarrollo y demo.

**Prompts empleadas**
```text
Crear un módulo Python `config.py` para un proyecto de visión por computador (Python 3.8+) que centralice: resolución (por defecto 640x480), FPS, dimensiones del área de juego en cm, rangos HSV para detección de la pelota, parámetros de tracking (alpha, min_radius), posiciones por defecto de hoyos, colores para AR y flags de debug. Incluir docstrings y validaciones básicas (valores positivos, HSV min < HSV max).
```
```text
Escribe tests unitarios que verifiquen que los valores por defecto son válidos (resolución > 0, HSV min/max coherentes) y que las constantes se puedan sobrescribir por entorno.
```

---

### `calibration.py` — Integrado: 2026-02-05
- Descripción: métodos de calibración manual (4 puntos) y automático (ArUco); cálculo de homografía y error de reproyección.

**Prompts empleadas**
```text
Implementa un módulo `calibration.py` que proporcione: (1) calibración manual mediante 4 clicks imagen↔plano, (2) calibración automática basada en ArUco, (3) función que calcule homografía y devuelva error de reproyección. Documenta API y añade ejemplos de uso.
```
```text
Añade tests que validen la correcta reconversión pixel→mundo con una homografía sintética y que el error de reproyección sea numéricamente razonable (< 1e-6 para pruebas sintéticas).
```

---

### `detection.py` — Integrado: 2026-02-06
- Descripción: detector híbrido (HSV + contornos + Hough) con API `AutomaticDetector` y `ManualDetector`.

**Prompts empleadas**
```text
Diseña `detection.py` con dos detectores: `ManualDetector` (click para centro+radio) y `AutomaticDetector` (umbral HSV parametrizable, morfología, filtrado por circularidad y fallback a HoughCircles). Entradas: frame BGR. Salidas: centro (x,y), radio y máscara binaria. Añadir logging y parámetros en `config.py`.
```
```text
Crea casos de prueba que generen imágenes sintéticas (círculos coloreados) y verifiquen que el detector devuelve la posición esperada y una máscara coherente.
```

---

### `tracking.py` — Integrado: 2026-02-07
- Descripción: seguimiento por color (ROI adaptativa) y wrappers para trackers OpenCV (CSRT/KCF/MOSSE); estimación de velocidad.

**Prompts empleadas**
```text
Implementa `ColorTracker` y un wrapper `OpenCVTracker` con interfaz común: init(frame,bbox), update(frame) -> position, predict(). `ColorTracker` debe usar ROI adaptativa y suavizado exponencial para velocidad (parámetro alpha en config). Añade manejo de pérdidas y re-detección.
```
```text
Escribe tests unitarios para la lógica de predicción/velocidad usando secuencias sintéticas y para la correcta conmutación entre trackers OpenCV.
```

---

### `positioning.py` — Integrado: 2026-02-08
- Descripción: transformación píxel↔mundo usando homografía; `correct_height()` para compensar centros elevados (geometría de rayo).

**Prompts empleadas**
```text
Crear funciones `pixel_to_world`, `world_to_pixel` y `correct_height()` que, dadas homografía y parámetros de cámara (Cx,Cy,Cz), corrijan la posición de una pelota cuyo centro está a altura h. Incluir pruebas numéricas con homografías sintéticas.
```
```text
Describe criterios de aceptación: reproyección < 2 px en condiciones controladas; `correct_height()` documentado matemáticamente con ejemplos.
```

---

### `virtual_scene.py` — Integrado: 2026-02-09
- Descripción: Gemelo digital 3D (Open3D) más alternativa 2D con OpenCV; API para actualizar posición de la pelota y renderizar.

**Prompts empleadas**
```text
Implementa `VirtualScene` (Open3D) para representar mesa, pelota, hoyo y trayectoria; debe ejecutarse en hilo separado y exponer `update(ball_world_pos)`. Añadir `SimpleVirtualScene` que renderice vista cenital 2D con OpenCV para entornos sin Open3D.
```
```text
Especifica pruebas de integración que validen que al actualizar `ball_world_pos` la representación 2D conserva las coordenadas y colores esperados.
```

---

### `ar_viewer.py` — Integrado: 2026-02-10
- Descripción: superposición AR (hoyo, trayectoria, puntuación) sobre la imagen usando homografía.

**Prompts empleadas**
```text
Desarrolla `ar_viewer.py` que dibuje elementos virtuales sobre frames reales: hoyo, bandera, trayectoria, HUD de puntuación. Debe usar funciones de `positioning.py` para proyectar coordenadas mundo→imagen y permitir toggles (--no-3d).
```
```text
Incluye un test visual manual y un pequeño script de smoke-test que renderice 5 frames de ejemplo y verifique que no se producen excepciones.
```

---

### `game_engine.py` — Integrado: 2026-02-11
- Descripción: lógica del mini-golf — niveles, golpes, detección de embocada, colisiones sencillas.

**Prompts empleadas**
```text
Implementa un `GameEngine` para mini-golf: gestión de niveles, conteo de golpes, detección de embocada (colisión pelota↔hoyo), reinicio de nivel y API para consultar estado (score, strokes, level). Diseñar con pruebas unitarias para reglas clave.
```
```text
Añade tests que simulen secuencias de golpe y verifiquen transiciones de estado (en juego → embocado → siguiente nivel).
```

---

### `demo_camera.py` — Integrado: 2026-02-12
- Descripción: cámara simulada compatible con `cv2.VideoCapture` — física simple de la pelota, controles WASD/ratón, marcadores para calibración.

**Prompts empleadas**
```text
Crea un `DemoCamera` que genere frames sintéticos con perspectiva y una pelota manipulable por teclado/ratón. Debe exponer la misma API que `cv2.VideoCapture` y permitir modos automáticos para pruebas de integración.
```
```text
Incluir pruebas de integración que ejecuten el pipeline en modo demo y verifiquen que la pelota es detectada y seguida por al menos 5 frames consecutivos.
```

---

### `main.py` — Integrado: 2026-02-13
- Descripción: CLI y orquestador del pipeline; argumentos: `--demo`, `--auto-calibrate`, `--auto-detect`, `--tracker`, `--correct-height`.

**Prompts empleadas**
```text
Diseñar `main.py` que arme el pipeline: captura (real o demo), calibración, detección, tracking, posicionamiento y render AR. Debe soportar argumentos CLI claros, logging configurable y manejo de excepciones para terminar limpiamente.
```
```text
Añade tests de integración end-to-end en modo `--demo` que validen ejecución sin hardware y salida sin errores durante N frames.
```

---

### `README.md`, `requirements.txt`, `structure.txt` — Integrado: 2026-02-14 → 2026-02-15
- Descripción: documentación, dependencias y mapa de estructura para facilitar onboarding.

**Prompts empleadas**
```text
Redacta un `README.md` completo que incluya: propósito del proyecto, estructura de ficheros, instrucciones de instalación (pip), uso (modo demo y real), opciones CLI y controles en ejecución. Incluir sección de desarrollo y SCRUM ligera.
```
```text
Genera `requirements.txt` con versiones mínimas: opencv-contrib-python>=4.5.0, numpy, open3d (opcional), pytest. Añade `structure.txt` con árbol resumido del repo.
```

---

### Notas finales
- Tiempo total estimado de integración (estado actual del repo):  ≤ 2 semanas (trabajo incremental con revisiones y pruebas). 
- Las *Prompts empleadas* se han formulado como especificaciones claras y ejecutables (código + criterios de aceptación + pruebas) para garantizar reproducibilidad en la implementación.

## v0.1.0 - Inicial
- Estado inicial del repositorio: detección, tracking, positioning, demo y juego.
