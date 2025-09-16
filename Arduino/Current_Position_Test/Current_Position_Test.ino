#include <Arduino.h>
#include "TCA9548A.h"
#include <Adafruit_INA260.h>
#include <Adafruit_MCP3008.h>

Adafruit_MCP3008 adc;
TCA9548A I2CMux;  // Address can be passed into the constructor
Adafruit_INA260 ina260 = Adafruit_INA260();

void setup() {
  Serial.begin(115200);

  // Wait until serial port is opened
  while (!Serial) { delay(10); }

  I2CMux.begin(Wire);  // Wire instance is passed to the library
  I2CMux.closeAll();   // Set a base state which we know (also the default state on power on)

  Serial.println("Adafruit INA260 Test");

  I2CMux.openChannel(0);
  if (!ina260.begin()) {
    Serial.println("Couldn't find INA260 chip");
    while (1)
      ;
  }
  Serial.println("Found INA260 chip");
  I2CMux.closeChannel(0);

  adc.begin(5, 19, 21, 4);
}

int channels[] = {0, 1, 7, 6, 5};   // Channels you want to read
int numChannels = sizeof(channels) / sizeof(channels[0]);

int loopFrequencyHz = 100;               // Desired frequency in Hz
unsigned long sampleIntervalMs = 1000 / loopFrequencyHz; // Interval between samples
unsigned long lastSampleTime = 0;      // Tracks last sample time

void loop() {
  unsigned long currentTime = millis();
  
  // Only proceed if enough time has passed
  if (currentTime - lastSampleTime >= sampleIntervalMs) {
    lastSampleTime = currentTime;

    String message = "";

    // --- Current sensors ---
    for (int i = 0; i < numChannels; i++) {
      int ch = channels[i];
      I2CMux.openChannel(ch);

      message += String(ina260.readCurrent(), 3); // 3 decimal places
      message += ",";

      I2CMux.closeChannel(ch);
    }

    // --- Position sensors ---
    for (int chan = 0; chan < 5; chan++) {
      message += String(adc.readADC(chan));
      if (chan < 4) message += ",";
    }

    // Send the complete message in one go
    Serial.println(message);
  }
}



