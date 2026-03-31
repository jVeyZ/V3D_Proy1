# Guía de Uso del Proyecto

## 1. Objetivo
Esta guía explica cómo instalar, ejecutar y demostrar el proyecto de posicionamiento planar con mini-golf virtual, incluyendo un apartado de cumplimiento de los requerimientos 1.1 a 1.5.

## 2. Requisitos Previos
- Sistema operativo: macOS, Linux o Windows.
- Python: 3.8 o superior.
- Cámara:
  - Opción A: webcam local.
  - Opción B: stream IP (HTTP/RTSP).
  - Opción C: modo demo sin cámara real.

Dependencias Python (ver requirements.txt):
- numpy
- opencv-python
- opencv-contrib-python
- open3d

## 3. Instalación
Desde la raíz del proyecto:

```bash
python -m venv .venv
source .venv/bin/activate   # En Windows: .venv\\Scripts\\activate
pip install -r requirements.txt
```

## 4. Ejecución Rápida
### 4.1 Modo demo (recomendado para primera prueba)

```bash
python pyScripts/main.py --demo --auto-calibrate --auto-detect
```

Qué hace este comando:
- Usa cámara simulada.
- Calibra automáticamente con las esquinas conocidas del entorno demo.
- Realiza detección automática inicial de la pelota.

### 4.2 Modo cámara real (webcam)

```bash
python pyScripts/main.py --camera 0 --auto-calibrate --auto-detect
```

### 4.3 Modo cámara real (stream IP)

```bash
python pyScripts/main.py --camera "http://IP:PUERTO/stream" --auto-calibrate --auto-detect
```

### 4.4 Ejecutar sin Open3D (solo AR + gemelo 2D)

```bash
python pyScripts/main.py --demo --auto-calibrate --auto-detect --no-3d
```

### 4.5 Ver todas las opciones

```bash
python pyScripts/main.py --help
```

### 4.6 Tabla de flags y usos

| Flag | Tipo | Valor por defecto | Uso | Ejemplo |
| --- | --- | --- | --- | --- |
| `--demo` | booleano (`store_true`) | `False` | Usa cámara simulada para pruebas sin webcam real. | `python pyScripts/main.py --demo` |
| `--auto-calibrate` | booleano (`store_true`) | `False` | Activa calibración automática: en demo usa esquinas conocidas y en cámara real intenta ArUco. | `python pyScripts/main.py --demo --auto-calibrate` |
| `--auto-detect` | booleano (`store_true`) | `False` | Activa detección automática inicial de la pelota (en lugar de clic manual). | `python pyScripts/main.py --auto-detect` |
| `--no-3d` | booleano (`store_true`) | `False` | Desactiva Open3D y deja solo el gemelo digital 2D + visor AR. | `python pyScripts/main.py --demo --no-3d` |
| `--camera` | texto (`str`) | `config.CAMERA_SOURCE` | Selecciona origen de vídeo: índice de webcam (`0`, `1`, ...) o URL de stream (`http/rtsp`). | `python pyScripts/main.py --camera 0` |
| `--tracker` | enum (`color`, `csrt`, `kcf`, `mosse`) | `config.TRACKING_METHOD` | Elige el método de seguimiento para los frames posteriores a la detección inicial. | `python pyScripts/main.py --tracker csrt` |
| `--correct-height` | booleano (`store_true`) | `False` | Aplica corrección por altura en el posicionamiento cuando el centro no está en el plano. | `python pyScripts/main.py --correct-height` |

Combinaciones recomendadas:
- Demo estable para presentación: `python pyScripts/main.py --demo --auto-calibrate --auto-detect`
- Cámara real con robustez de tracking: `python pyScripts/main.py --camera 0 --auto-calibrate --auto-detect --tracker csrt`
- Equipos sin soporte Open3D: `python pyScripts/main.py --demo --auto-calibrate --auto-detect --no-3d`

## 5. Flujo de Uso Recomendado
1. Iniciar en modo demo para validar instalación y ventanas.
2. Verificar que aparecen:
   - Ventana AR principal.
   - Ventana Gemelo Digital 2D.
   - Ventana Open3D (si no se usa --no-3d).
3. Comprobar detección inicial de la pelota.
4. Mover la pelota (demo o escena real) y validar seguimiento.
5. Confirmar que la posición se refleja en:
   - Gemelo digital.
   - Superposición AR.
6. Activar corrección de altura si procede:

```bash
python pyScripts/main.py --demo --auto-calibrate --auto-detect --correct-height
```

## 6. Controles Durante la Ejecución
- q o ESC: salir.
- c: recalibrar.
- d: cambiar modo de detección (manual/automático).
- f: forzar re-detección.
- m: forzar detección manual inmediata.
- n: nueva partida.
- r: reiniciar nivel.
- v: activar/desactivar escena 3D.
- h: mostrar/ocultar panel de ayuda.
- e: mostrar explicación de corrección de altura.

Solo en modo demo:
- WASD o flechas: mover pelota.
- t: alternar movimiento automático/manual.
- Click y arrastre: mover pelota con ratón.

## 7. Estructura Funcional (Resumen)
- pyScripts/main.py: integración completa de la aplicación.
- pyScripts/calibration.py: homografía manual y automática (ArUco).
- pyScripts/detection.py: detección manual y automática.
- pyScripts/tracking.py: seguimiento por color o trackers OpenCV.
- pyScripts/positioning.py: posicionamiento planar y corrección de altura.
- pyScripts/virtual_scene.py: gemelo digital (Open3D + fallback 2D).
- pyScripts/ar_viewer.py: overlay AR de elementos virtuales.
- pyScripts/game_engine.py: lógica del juego de mini-golf.
- pyScripts/demo_camera.py: cámara simulada para pruebas.

## 8. Cumplimiento de Requerimientos
### 1.1 Escena virtual tipo gemelo digital (2 puntos)
Estado: Cumplido.
- Se implementa visor Open3D interactivo y vista 2D alternativa.
- Se representa al menos un objeto real (pelota) y objetos virtuales (hoyo, bandera, obstáculos).
- El estado de la pelota se actualiza continuamente en el gemelo digital.

Módulos implicados:
- pyScripts/virtual_scene.py
- pyScripts/main.py

### 1.2 Detección y seguimiento de objeto real en movimiento (2 puntos)
Estado: Cumplido.
- Detección manual disponible (clic sobre la imagen).
- Detección automática disponible (HSV + contornos + fallback Hough).
- Seguimiento en frames sucesivos con tracker por color o trackers OpenCV.
- La app prioriza detección inicial y seguimiento posterior; además incluye re-detección de recuperación cuando el tracker se pierde para mayor robustez.

Módulos implicados:
- pyScripts/detection.py
- pyScripts/tracking.py
- pyScripts/main.py

### 1.3 Posicionamiento planar con una cámara + actualización en visor virtual y AR (2 puntos)
Estado: Cumplido.
- Posicionamiento con homografía imagen-plano.
- Calibración manual y automática con ArUco.
- Actualización continua de posición estimada en gemelo digital.
- Proyección de elementos virtuales en el visor de realidad aumentada.

Módulos implicados:
- pyScripts/calibration.py
- pyScripts/positioning.py
- pyScripts/virtual_scene.py
- pyScripts/ar_viewer.py
- pyScripts/main.py

### 1.4 Corrección por altura cuando el centro no está sobre el plano (2 puntos)
Estado: Cumplido.
- Se implementa corrección geométrica de altura usando el centro de cámara estimado.
- Existe explicación teórica integrada para justificar el método.
- Se puede activar en ejecución con --correct-height.

Módulos implicados:
- pyScripts/positioning.py
- pyScripts/main.py

### 1.5 Aplicación integrada de posicionamiento planar (2 puntos)
Estado: Cumplido.
- Integración completa en una aplicación interactiva (mini-golf).
- Se integran detección, seguimiento, posicionamiento, AR y escena virtual.
- Incluye gestión de estado de juego, niveles, golpes y colisiones.

Módulos implicados:
- pyScripts/main.py
- pyScripts/game_engine.py

## 9. Checklist de Demostración
Usa esta lista para la presentación:
1. Lanzar en demo y mostrar calibración automática.
2. Mostrar detección inicial (manual o automática).
3. Mostrar seguimiento de la pelota en movimiento.
4. Mostrar actualización simultánea en AR y gemelo digital.
5. Activar --correct-height y explicar cuándo aplica.
6. Completar un hoyo para demostrar la integración del juego.

## 10. Solución de Problemas Frecuentes
- No abre cámara real:
  - Verificar índice o URL de --camera.
  - Probar modo demo.
- Open3D falla:
  - Ejecutar con --no-3d y mantener gemelo 2D.
- Detección inestable:
  - Ajustar rangos HSV en pyScripts/config.py.
  - Mejorar iluminación y contraste de la pelota.
- Calibración ArUco no converge:
  - Confirmar 4 marcadores visibles y con IDs correctos.
  - Usar calibración manual como alternativa.
