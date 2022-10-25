// [ArduinoComm/LightDemo/microcontroller.ino]

/*
    Simple script to receive data from Julia and light up LEDS appropriately.

      For more information about serial communication on an Arduino, check out
    https://forum.arduino.cc/t/serial-input-basics-updated/382007 
*/
#include "Wire.h"

#define NUM_LEDS 6 
#define INITIAL_FLASH_COUNT 3
#define FLASH_DURATION 1000 // Milliseconds

// Arrays for holding received data 
const byte maxChars = 32;
char receivedChars[maxChars];  // Stores received characters 
char tempChars[maxChars];      // Used to hold data while parsing 

bool receivedData = false;     // Flag used to notify of new data

// Sun locations
float sx = 0.0;
float sy = 0.0;
float sz = 0.0;


int LEDpins[NUM_LEDS] = {6, 11, 10, 9, 5, 3}; // +X, -X, +Y, -Y, +Z, -Z

// HELPER FUNCTIONS
void flash_lights(int dur) {
    // Flashes lights on and off, which is helpful for communication with human 

    // Light up each LED
    for (uint8_t i = 0; i < NUM_LEDS; i++) 
        digitalWrite(LEDpins[i], HIGH);

//    while (true)
    delay(dur);

    for (uint8_t i = 0; i < NUM_LEDS; i++) 
        digitalWrite(LEDpins[i], LOW);
}

void lightLEDs() {
    // Updates the appropriate LEDs by the right values (e.g., only +/- X, not both)

    int led_xp = (sx > 0) ? round(sx * 255) : 0;  // Map from [0, 1] -> [0, 255]
    int led_xn = (sx < 0) ? round(-sx * 255) : 0;  

    int led_yp = (sy > 0) ? round(sy * 255) : 0;  // Map from [0, 1] -> [0, 255]
    int led_yn = (sy < 0) ? round(-sy * 255) : 0;  

    int led_zp = (sz > 0) ? round(sz * 255) : 0;  // Map from [0, 1] -> [0, 255]
    int led_zn = (sz < 0) ? round(-sz * 255) : 0;  

    // Use PWM to adjust brightness 
    analogWrite(LEDpins[0], led_xp);
    analogWrite(LEDpins[1], led_xn);
    analogWrite(LEDpins[2], led_yp);
    analogWrite(LEDpins[3], led_yn);
    analogWrite(LEDpins[4], led_zp);
    analogWrite(LEDpins[5], led_zn);

}

void parseData() {
    char* strTokIdx = strtok(tempChars, ",");  // Indices of each string token 
    sx = atof(strTokIdx);   // First element is sx 

    strTokIdx = strtok(NULL, ",");
    sy = atof(strTokIdx); 

    strTokIdx = strtok(NULL, ",");
    sz = atof(strTokIdx);
}

void receiveData() {
    static bool rxInProgress = false;  // Used so we dont interrupt ourselves 
    static byte idx = 0;

    char startMarker = '[';
    char endMarker = ']';
    char rc;

    // Start updating once data is available and we are not currently working with new data 
    while (Serial.available() > 0 && !receivedData) {
        rc = Serial.read();

        // If currently receiving data 
        if (rxInProgress) {

            // If End-of-Data reached 
            if (rc == endMarker) {
                receivedChars[idx] = '\0';
                rxInProgress = false;
                idx = 0;
                receivedData = true;
            } else {        // Still receiving data, store value 
                receivedChars[idx++] = rc;
                if (idx >= maxChars) 
                  idx = maxChars - 1;             
            }
        } else if (rc == startMarker) {
                rxInProgress = true;  // Note that we do not store the start marker 
        }
    }
}


void setup() {
    // Runs once 

    delay(1000);
    Serial.begin(115200);

    // Set up pins as outputs
    for (uint8_t i = 0; i < NUM_LEDS; i++) 
        pinMode(LEDpins[i], OUTPUT);

    for (uint8_t i = 0; i < INITIAL_FLASH_COUNT; i++) 
        flash_lights(FLASH_DURATION);

}


void loop() {
    receiveData();

    if (receivedData) {
        strcpy(tempChars, receivedChars);  // Copy over received characters to empty array 
        parseData();
        lightLEDs();
        receivedData = false;
    }

    delay(20);
}
