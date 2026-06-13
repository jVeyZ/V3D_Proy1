Balloon Catch 3D - Proyecto 2 V3D

Lee README.md para las instrucciones completas de instalación, calibración estéreo, calibración del sistema de mundo y ejecución.

Resumen rápido:
  1) pip install -r requirements.txt
  2) python scripts/make_charuco_board.py --output calibration/charuco_5x7_4cm.png
  3) python scripts/capture_charuco_stereo.py --left 0 --right 2 --output calibration/capture
  4) python scripts/calibrate_charuco_stereo.py --input calibration/capture --output calibration/stereo_charuco.npz --report-dir calibration/report
  5) Automático recomendado: colocar ChArUco sobre el plano de juego y capturar un par en calibration/world_charuco.
  6) python scripts/calibrate_world_from_charuco.py --stereo-calibration calibration/stereo_charuco.npz --left-image calibration/world_charuco/left/left_000.png --right-image calibration/world_charuco/right/right_000.png --output calibration/world_transform.npz --report calibration/WORLD_TRANSFORM_REPORT.md
  7) Alternativa manual: editar calibration/world_points.csv y usar scripts/calibrate_world_from_points.py.
  8) python -m src.balloon_catch

Archivos clave:
  src/balloon_catch.py
  src/balloon_tracker.py
  src/stereo.py
  scripts/calibrate_charuco_stereo.py
  scripts/calibrate_world_from_charuco.py
  scripts/calibrate_world_from_points.py
