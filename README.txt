Charuco stereo calibration (recommended for 40cm x 40cm scene)

Suggested board size for A4:
- squares_x = 5
- squares_y = 7
- square_len_cm = 4.0
- marker_len_cm = 3.0
Board size: 20cm x 28cm

1) Generate board image (print at 100% scale):
   python scripts/make_charuco_board.py --output calibration/charuco_5x7_4cm.png

2) Capture stereo pairs (press 's' to save):
   python scripts/capture_charuco_stereo.py --left 0 --right 1 --output calibration/capture

3) Calibrate and save parameters:
   python scripts/calibrate_charuco_stereo.py --input calibration/capture --output calibration/stereo_charuco.npz

After calibration, share the .npz and I can wire it into penaltis_coches.py to fix Z scale.
