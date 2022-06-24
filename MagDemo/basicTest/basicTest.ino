/* [CubeSatBox/MagDemo/basicTest/basicTest.ino

      Simple script to initialize magnetometers, connect to them,
    and print the readings to a serial line. Intended to be used in
    conjunction with the basicTest.jl script.

    (NOTE - we are using LIS3MDL Triple-axis Magnetometers, as well as a
    TCA9548A 1-to-8 I2C Multiplexer)
*/

#include "Wire.h"   // For Serial communication 
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>

#define TCAADDR 0x70
#define NUM_PORTS 6

Adafruit_LIS3MDL lis3mdl;
#define LIS3MDL_CLK  13
#define LIS3MDL_MISO 12
#define LIS3MDL_MOSI 11
#define LIS3MDL_CS   10

// TCS Port => Axis:
// 0 => +Z
// 1 => +X
// 2 => -X
// 3 => -Y
// 4 => +Y
// 5 => -Z


void tcaselect(uint8_t i) {
  // Adjusts which magnetometer the Multiplexer maps to

  if (i > NUM_PORTS) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void init_magnetometer(uint8_t port) {
  // Initialize a magnetometer at a given port (through the Mux)

  Serial.print("Trying to init magnetometer at "); Serial.println(port);
  tcaselect(port);

  if (!lis3mdl.begin_I2C()) {
    // This is bad
    Serial.println("\tFailed to find LIS3MDL chip!\n");
    delay(100);
    return;
  }


  Serial.println("\tLIS3MDL found!\n");

  // Default parameters
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          true); // enabled!
}

void read_magnetometer(uint8_t port) {
  // Reads the magnetometer sensor input and prints it out

  tcaselect(port);


  lis3mdl.read();   // Update the data in our global object
  //
  //    // Human Format
//  Serial.print("\tX:    "); Serial.print(lis3mdl.x);
//  Serial.print("\tY:    "); Serial.print(lis3mdl.y);
//  Serial.print("\tZ:    "); Serial.println(lis3mdl.z);

  // Machine Format
  Serial.print("["); Serial.print(lis3mdl.x); Serial.print(", ");
  Serial.print(lis3mdl.y); Serial.print(", ");
  Serial.print(lis3mdl.z); Serial.println("]");

}

void setup() {
  // Runs once at the beginning to set up everything
  Wire.begin();
  while (!Serial);   // Wait for a connection
  delay(4000);       // Wait for x milliseconds  (I make it long so I have time to see if the init fails)

  Serial.begin(115200);
  Wire.begin();
  Serial.println("\nTCA Setting up!");

  for (uint8_t port = 0; port < NUM_PORTS; port++) {
    init_magnetometer(port);
  }

}

int axes[NUM_PORTS] = {1, 2, 4, 3, 0, 5};

void loop() {
  // Runs repeatedly after 'setup()'

//  Serial.println("\nReading!");
  for (uint8_t port = 0; port < NUM_PORTS; port++) {
    read_magnetometer(axes[port]);
    delay(100);
  }
  Serial.println("|");

  delay(500);

// // Verify the right order is being read
//  for (uint8_t port = 0; port < NUM_PORTS; port++) {
//    Serial.print("[");
//    Serial.print("0"); Serial.print(", "); 
//    Serial.print("0"); Serial.print(", "); 
//    Serial.print(axes[port]); Serial.println("]");
////    Serial.print(", ");
//    delay(100);
//  }
//  Serial.println("|");
//
//  delay(500);
}
