# Quanum 3-Axis Mobius Camera Based Gimbal

Documentation for the Quanum 3-Axis Mobius Camera Based Gimbal. (Purchase link [here](https://hobbyking.com/en_us/quanum-3-axis-mobius-camera-based-gimbal.html?gclid=CjwKCAjwgabeBRBuEiwACD4R5pzr1hS_OftXpa5OPtl5WfkAG35Ie4TQcBOMSMw8y5pVsvsa1bz7rRoC8JMQAvD_BwE&gclsrc=aw.ds&___store=en_us) on Hobbyking). The official manual is sparse and available [here](https://cdn-global-hk.hobbyking.com/media/file/74871598X976235X4.pdf). 

### Tuning Setup for the Gimbal

The tuning of the PID and motor speed is to be done on **SimpleBGC_GUI_2_40b8**. This version is specific to the gimbal. The download is available [here](http://www.basecamelectronics.com/files/SimpleBGC_GUI_2_40b8.zip). Ensure that the gimbal is tuned with the camera or whichever weights you'd intend to carry in the actual flight/operation of the gimbal.

1. Set all power to 0
2. Set all PID to 0
3. Set Pitch power up in increments of 50 until you feel satisfactory resistance when you push on the camera
4. Increment Pitch P as much as possible until it begins to wobble, then drop it a bit
5. Increment pitch D until the wobbling stops, then continue to increasing until D starts to oscillate a high frequency, then drop D a bit
6. Observe camera as you change the angle, if you observe that the camera does not remain completely level it means that there is some steady state error which means we have to tune pitch I, else skip ahead to step 8
7. Increment Pitch I very slowly until the camera remains level throughout the entire range of motions
8. Set the same power and PID values for roll as well and test. 
   1. If camera over compensates roll, then reduce roll I
   2. If camera wobbles too much, then reduce roll D
9. Set same power and PID values for Yaw from Pitch
   1. If camera over compensates yaw, then reduce yaw I
   2. If camera wobbles too much, then reduce yaw D

### Custom Mount for the Runcam 2

To attach the Runcam 2, a custom mount was made and printed out. The CAD file is named "runcam2-mountv2.SLDPRT" and can be found in this folder. 

1. Remove the screws to the original Runcam mount
2. Using a sharp tool, pry the IMU from the base of the mount
3. The IMU will now be free, but the wires of the IMU will run through the existing mount - unsolder these wires. However, the wires will be 1 red and 3 black, which means that **labelling the wires is necessary**.
4. If labelling of the wires is not possible, remove the top plate of the gimbal (the gimbal base) to look at the wiring going straight into the board. Feel free to conduct continuity checks with a multimeter to check on which wire is which
5. Thread the wires through the wire ports on the 3d-printed mount. 
6. Solder the wires back onto the IMU through the wire ports on the new mount. Do continuity checks with a multimeter to check your work if necessary.
7. Attach the IMU firmly in the IMU slot using mounting tape or any form of flat adhesion - **this is critical**
8. Reattach the mount onto the gimbal arm using the original screws
9. The new mount should have been installed. Calibration can now begin

