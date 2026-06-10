Balloon Catch 3D - Proyecto 2 V3D

Lee README.md para las instrucciones completas de instalación, calibración estéreo, calibración del sistema de mundo y ejecución.

Resumen rápido:
  1) pip install -r requirements.txt
  2) python scripts/make_charuco_board.py --output calibration/charuco_5x7_4cm.png
  3) python scripts/capture_charuco_stereo.py --left 0 --right 2 --output calibration/capture
  4) python scripts/calibrate_charuco_stereo.py --input calibration/capture --output calibration/stereo_charuco.npz --report-dir calibration/report
  5) Editar calibration/world_points.csv con puntos conocidos del mundo y sus píxeles homólogos.
  6) python scripts/calibrate_world_from_points.py --stereo-calibration calibration/stereo_charuco.npz --points calibration/world_points.csv --output calibration/world_transform.npz --report calibration/WORLD_TRANSFORM_REPORT.md
  7) python -m src.balloon_catch

Archivos clave:
  src/balloon_catch.py
  src/balloon_tracker.py
  src/stereo.py
  scripts/calibrate_charuco_stereo.py
  scripts/calibrate_world_from_points.py
