# Automatic World Calibration from ChArUco

This report documents the automatic offline calibration from left-camera stereo coordinates to the Balloon Catch world frame.

## Inputs

- Stereo calibration: `calibration/stereo_charuco.npz`
- Left image: `calibration/world_charuco/left/left_000.png`
- Right image: `calibration/world_charuco/right/right_000.png`
- Output transform: `calibration/world_transform.npz`
- Common ChArUco corners: 8
- Board world offset: (0.0, 0.0, 0.0) cm

## Result

- RMS world residual: 0.8079 cm
- Mean residual: 0.6781 cm
- Max residual: 1.7488 cm
- Mean epipolar error: 22.6241 px
- Max epipolar error: 23.1891 px

## Convention

`X_world = R_world_from_left @ X_left_camera + t_world_from_left`

## R_world_from_left

```
[[-0.17774646 -0.36498551  0.91388827]
 [ 0.98349305 -0.03391629  0.17773885]
 [-0.03387641  0.93039522  0.36498921]]
```

## t_world_from_left

```
[-79.69238999  -7.37770119 -95.02950383]
```

## Per-corner residuals

| id | camera-left XYZ cm | world target XYZ cm | transformed XYZ cm | residual cm | epipolar px |
| --- | --- | --- | --- | --- | --- |
| 0 | -7.02, 57.71, 113.08 | 4.00, 4.00, 0.00 | 3.83, 3.86, 0.18 | 0.284 | 22.350 |
| 4 | -3.20, 57.64, 114.30 | 4.00, 8.00, 0.00 | 4.29, 7.83, 0.43 | 0.543 | 21.666 |
| 8 | 0.83, 56.70, 112.95 | 4.00, 12.00, 0.00 | 2.69, 11.59, -1.08 | 1.749 | 22.927 |
| 9 | 0.29, 55.60, 119.01 | 8.00, 12.00, 0.00 | 8.72, 12.18, 0.12 | 0.754 | 22.314 |
| 12 | 4.76, 57.41, 115.21 | 4.00, 16.00, 0.00 | 3.80, 15.84, 0.27 | 0.375 | 22.801 |
| 13 | 4.42, 55.17, 119.20 | 8.00, 16.00, 0.00 | 8.32, 16.28, -0.34 | 0.545 | 22.975 |
| 16 | 8.91, 57.24, 115.83 | 4.00, 20.00, 0.00 | 3.69, 20.03, 0.21 | 0.375 | 22.771 |
| 17 | 8.39, 55.43, 120.45 | 8.00, 20.00, 0.00 | 8.66, 20.40, 0.22 | 0.801 | 23.189 |
