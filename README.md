# Robot-Dog

PULSE RANGES

LEG 1
HIP_LEG(81,591)
KNEE(81,581)

servos
1-(81,590)
2-(82,588)
3-(80,585)

for 180
92-520
80-506
105-545
111-545
100-532
AVERAGE DIFF=430


body_hip phase +90 // mapping world coordinate to servo coordinate system 
leg_hip phase +150
knee phase -180 or 0


Run cmd prompt as administrator and run:
mklink /D "C:\Users\sakar\OneDrive\Documents\Arduino\libraries\LegControl" "E:\THIRD WORLD NERD\robotDog\libraries\LegControl"
do the same for rest of the library folders as well.


Calibration Data:
    Sampling Frequency: 100Hz
    Magnetometer:
        const float magn_ellipsoid_center[3] = {-16.444, 123.131, -60.0703};
        const float magn_ellipsoid_transform[3][3] = {{0.887227, 0.0186386, 0.0175599}, {0.0186386, 0.878481, 0.0445495}, {0.0175599, 0.0445495, 0.978254}};

    Accelerometer and Gyroscope:
        #define GYRO_X_OFFSET 0.0003547
        #define GYRO_Y_OFFSET 0.0002904
        #define GYRO_Z_OFFSET 0.0000442

        #define ACCEL_X_OFFSET -1.7587781
        #define ACCEL_Y_OFFSET -8.6502850
        #define ACCEL_Z_OFFSET -9.2377464

