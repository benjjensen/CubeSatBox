# [CubeSatBox/PerformanceEvaluation/simulate_sun.py]  

"""
     Contains the code intended to be run on the CubeSat during evaluation 
    of the sun vector estimation. 

    Note that, for convenience, this only actually reads in the diode values which are passed back.
    The .jl file then uses the raw data to estimate the sun vector so we can test using the existing 
    estimate_sun_vector code and don't have to deal with the DIODES struct (TODO for now)

    (See CubeSatBox/LightDemo/sun_estimator.py for a simple on-sat version that isn't as accurate)
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

# Helpful colors for use in debugging 
WHITE = (255, 255, 255)    
OFF   = (0, 0, 0)  
RED   = (255, 0, 0)  
GREEN = (0, 255, 0)    
BLUE  = (0, 0, 255) 
TEAL  = (0, 255, 255)      
        
diodes_old = np.zeros(6) 

while True: 
    cubesat.RGB = WHITE      
      
    diodes = np.zeros(6)   
    for i in range(6):  
        # try:  
        lux = sun_sensors[i].lux 
        if lux is not None:
            diodes[i] = lux     
        else:      
            diodes[i] = 0.0 # diodes_old[i]  # Just use last value 
           

    # Send measured data back to the laptop so we can use our Julia code   
    if not all(diodes == diodes_old):
        print("[{:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}]\n".format(diodes[0], diodes[1], diodes[2], diodes[3], diodes[4], diodes[5]) )
   
    diodes_old = diodes      

    time.sleep(0.3)   
  
             