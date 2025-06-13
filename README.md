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
mklink /D "C:\Users\sakar\OneDrive\Documents\Arduino\libraries\LegControl" "E:\THIRD WORLD NERD\robotDog\LegControl"