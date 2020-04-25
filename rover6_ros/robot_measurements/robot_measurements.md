# Robot Measurements

## Overall Dimensions

### Side View
Dimension Name | Measurement (m)
--- | ---
Height  |  0.1869
Length  |  0.25021

![Side_View](Side_View.png)

### Top View
Dimension Name | Measurement (m)
--- | ---
Minor Diameter  |  0.261
Major Diameter  |  0.28727

![Top_View](Top_View.png)

## Transforms

The coordinate frame for the Solidworks CAD vs. ROS is different. Here is the mapping:

Solidworks axis | ROS axis
--- | ---
-Z  |  +X
-X  |  +Y
+Y  |  +Z

Transforms will be listed according to the ROS coordinate frame

### Origin to Laser
Axis | Measurement (m)
--- | ---
X  |  0.030
Y  |  0.0
Z  |  0.10698

![Laser_TF](Laser_TF.png)

### Origin to IMU
Axis | Measurement (m)
--- | ---
X  |  -0.05886
Y  |  0.00088
Z  |  0.04535

![IMU_TF](IMU_TF.png)

### Origin to Dual Encoder Midpoint
Axis | Measurement (m)
--- | ---
X  |  -0.0004
Y  |  0.0
Z  |  0.040

![Dual_Encoder_Midpoint_TF](Dual_Encoder_Midpoint_TF.png)

### Origin to Camera Pan-Tilt Origin
Axis | Measurement (m)
--- | ---
X  |  0.0945
Y  |  0.0
Z  |  0.07171

![Camera_Pan-Tilt_Origin](Camera_Pan-Tilt_Origin.png)

### RPi Camera Pan-Tilt Origin to Camera Image Center
Axis | Measurement (m)
--- | ---
X  |  0.0264
Y  |  0.0
Z  |  0.00241

![Camera_Pan-Tilt_Origin_to_Camera_Image_Center](Camera_Pan-Tilt_Origin_to_Camera_Image_Center.png)

### RealSense D435 Camera Pan-Tilt Origin to Camera Image Center
Axis | Measurement (m)
--- | ---
X  |  0.04529
Y  |  0.0
Z  |  0.0

![D435_Camera_Pan-Tilt_Origin_to_Camera_Image_Center](D435-Camera-Pan-Tilt-Origin-to-Camera-Image-Center.png)
