# [CubeSatBox/LightDemo/sun_estimator.py]

"""
        Reads in the satellite sun sensor values and uses those to 
    estimate the unit vector in the direction of the sun. Intended to
    be run as part of the interactive CubeSat test box LED demo to 
    evaluate the estimation algorithm and eventually help calibrate 
    the sun sensors. 

        Intended to be run with sun_estimator.jl to generate the sun
    location and microcontroller/microcontroller.ino to operate the LEDs.

"""

import time  
from pycubed import cubesat 
import adafruit_tsl2561 # sun sensor library

import ulab.numpy as np 
import msgpack as mp
from io import BytesIO


# Initialize sun sensors
#   (Note that the + direction is i2c2 and the - direction is i2c1)
sun_xp = adafruit_tsl2561.TSL2561(cubesat.i2c2, address = 0x49)   # THIS IS NOT WHAT IT SHOULD BE ? (0x29 vs 0x49)
sun_xn = adafruit_tsl2561.TSL2561(cubesat.i2c1, address = 0x49)
sun_yp = adafruit_tsl2561.TSL2561(cubesat.i2c2, address = 0x39)
sun_yn = adafruit_tsl2561.TSL2561(cubesat.i2c1, address = 0x39)  # try 0xa9, 0xb9, 0x129, 0x139, 0x1a9, 0x1b9
sun_zp = adafruit_tsl2561.TSL2561(cubesat.i2c2, address = 0x29)  # 0xa9 + 0x129 + 0x1a9 = +X | 0x139 + 0xb9 + 0x1b9 = +Y 
sun_zn = adafruit_tsl2561.TSL2561(cubesat.i2c1, address = 0x29)   
sun_sensors = [sun_xp, sun_xn, sun_yp, sun_yn, sun_zp, sun_zn]


imu_data = {  
    'accel': 0, 
    'mag':   0,
    'gyro':  0,
}

WHITE = (255, 255, 255) 
OFF   = (0, 0, 0)
RED   = (255, 0, 0) 
GREEN = (0, 255, 0)  
BLUE  = (0, 0, 255)
TEAL  = (0, 255, 255)

while True: 
    cubesat.RGB = WHITE

    diodes = np.zeros(6)
    for i in range(6):  
        if sun_sensors[i].lux is not None:  
            diodes[i] = sun_sensors[i].lux   
        else:     
            diodes[i] = 0.0 

    sun_vec = np.array([
        diodes[0] - diodes[1],     
        diodes[2] - diodes[3],       
        diodes[4] - diodes[5]    
    ])

   
    # # Make it unit     
    sun_vec = sun_vec / np.linalg.norm(sun_vec)

  
    # pkg = BytesIO()   
    # mp.pack([1.0, 2.0, 3.0], pkg)  
    # pkg.seek(0)        
 
    # # temp = mp.unpack(pkg)   
    # # pkg = mp.pack([1.0, 2.0, 3.0])
    # print(pkg) 
    
    print("[{:.3f}, {:.3f}, {:.3f}]\n".format(sun_vec[0], sun_vec[1], sun_vec[2]) )
   

    time.sleep(0.2)  

   