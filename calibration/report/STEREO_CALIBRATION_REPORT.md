# Stereo Calibration Report

## Summary

- Status: **REVISAR**
- Input folder: `calibration/capture`
- Output calibration: `calibration/stereo_charuco.npz`
- Image size: (640, 480)
- Captured pairs found: 18
- Valid mono Charuco detections per camera: 18
- Valid stereo pairs with common corners: 18
- RMS left: 0.475506 px
- RMS right: 0.407072 px
- RMS stereo: 2.401365 px
- Baseline norm: 26.165204 cm
- Recommended maximum RMS: 2.00 px

## Calibration target

- Charuco squares: 5 x 7
- Square length: 4.0 cm
- Marker length: 3.0 cm

## Matrices

### K_l

```
[[481.4217099    0.         297.47880688]
 [  0.         470.37352513 208.95448287]
 [  0.           0.           1.        ]]
```

### dist_l

```
[ 0.01903198 -0.12579288 -0.04928969 -0.02851188  0.07651806]
```

### K_r

```
[[412.95610891   0.         307.73051232]
 [  0.         557.48193049 172.19828707]
 [  0.           0.           1.        ]]
```

### dist_r

```
[-0.04218916  0.22398762 -0.03992283 -0.01608676 -0.50885956]
```

### R_right_from_left

```
[[ 0.99493961 -0.0386832   0.09272959]
 [ 0.02698839  0.99188927  0.12420667]
 [-0.09678219 -0.12107552  0.98791393]]
```

### T_right_from_left

```
[-21.71144707 -14.53050374  -1.44755524]
```

## How to use this report

Include this file and the images in `calibration/report/images/` in the project memory. If RMS stereo is high, recapture more pairs with the board covering the full field of view.
