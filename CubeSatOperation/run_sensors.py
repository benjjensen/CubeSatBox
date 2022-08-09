
""" Scan for ports
from pycubed import cubesat 
import time 

while not cubesat.i2c1.try_lock():
    pass 

while True:
    print("I2C addresses found: ", [hex(device_add) for device_add in cubesat.i2c1.scan()])
    time.sleep(2)
"""
import time  
from pycubed import cubesat 
import adafruit_tsl2561 # sun sensor library
import drv8830          # h-bridge library  
 
 
""" Addresses:
    - V-R3x +X panel v02
        - sun sensor I2C bus #2 (i2c2) address: 0x49 
        - h-bridge   I2C bus #2 (i2c2) address: 0x66
    - V-R3x -X panel v02
        - sun sensor I2C bus #1 (i2c1) address: 0x49
        - h-bridge   I2C bus #1 (i2c1) address: 0x66
    - V-R3x +Y panel v02
        - sun sensor I2C bus #2 (i2c2) address: 0x39
        - h-bridge   I2C bus #2 (i2c2) address: 0x62
    - V-R3x -Y panel v02
        - sun sensor I2C bus #1 (i2c1) address: 0x39
        - h-bridge   I2C bus #1 (i2c1) address: 0x64
    - V-R3x +Z panel v03
        - sun sensor I2C bus #2 (i2c2) address: 0x29 
        - h-bridge   I2C bus #2 (i2c2) address: 0x60
    - V-R3x -Z panel v02
        - sun sensor I2C bus #1 (i2c1) address: 0x29
        - h-bridge   I2C bus #1 (i2c1) address: 0x60

I2C1 addresses found: ['0x29', '0x39', '0x49', '0x4a', '0x60', '0x64', '0x66', '0x68']
I2C2 addresses found: ['0x29', '0x39', '0x49', '0x60', '0x62', '0x66']
"""

# Initialize sun sensors
#   (Note that the + direction is i2c2 and the - direction is i2c1)
sun_xp = adafruit_tsl2561.TSL2561(cubesat.i2c2, address = 0x49)   
sun_xn = adafruit_tsl2561.TSL2561(cubesat.i2c1, address = 0x49)
sun_yp = adafruit_tsl2561.TSL2561(cubesat.i2c2, address = 0x39)
sun_yn = adafruit_tsl2561.TSL2561(cubesat.i2c1, address = 0x39)  
sun_zp = adafruit_tsl2561.TSL2561(cubesat.i2c2, address = 0x29) 
sun_zn = adafruit_tsl2561.TSL2561(cubesat.i2c1, address = 0x29)   
sun_sensors = [sun_xp, sun_xn, sun_yp, sun_yn, sun_zp, sun_zn]

# Initialize the H-Bridges
hb_xp = drv8830.DRV8830(cubesat.i2c2, 0x66)
hb_xp.panel = '+X'
hb_xn = drv8830.DRV8830(cubesat.i2c1, 0x66)
hb_xn.panel = '-X'
hb_yp = drv8830.DRV8830(cubesat.i2c2,0x62)
hb_yp.panel = '+Y'
hb_yn = drv8830.DRV8830(cubesat.i2c1,0x64)
hb_yn.panel = '-Y'
hb_zp = drv8830.DRV8830(cubesat.i2c2,0x60)
hb_zp.panel = '+Z'
hb_zn = drv8830.DRV8830(cubesat.i2c1,0x60)
hb_zn.panel = '-Z'
hbridges = [hb_xp, hb_yp, hb_yn, hb_zp, hb_zn]


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

    # cache IMU data 
    imu_data.update({
        'accel':cubesat.acceleration,
        'mag':  cubesat.magnetic,
        'gyro': cubesat.gyro
    })

    # print IMU data 
    print('[IMU]')
    for imu_type in imu_data:
        print('{:>8} x: {:.6f}, \ty: {:.6f},\tz: {:.6f}'.format(imu_type,*imu_data[imu_type]))
  
    # print sun sensor data
    print('[Sun]')
    print('\t+X: {}, -X: {}'.format(sun_sensors[0].lux, sun_sensors[1].lux))
    print('\t+Y: {}, -Y: {}'.format(sun_sensors[2].lux, sun_sensors[3].lux))
    print('\t+Z: {}, -Z: {}'.format(sun_sensors[4].lux, sun_sensors[5].lux))

    # Exercise h-bridges (in isolation)
    for h in hbridges:
        print(h.panel)
        h.vout = 0x3F 
        h.mode = drv8830.REVERSE 
        time.sleep(1.0)
        h.mode = drv8830.FORWARD
        time.sleep(1.0)

    time.sleep(3.0)

  