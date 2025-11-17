#include <Wire.h>
#include <Adafruit_DRV2605.h>

#define I2C_SDA 21
#define I2C_SCL 22

Adafruit_DRV2605 drv;

void setup() {
  Serial.begin(115200);
  delay(500);

  // Start I2C on ESP32 with our chosen pins
  Wire.begin(I2C_SDA, I2C_SCL);

  // Initialize DRV2605L (address 0x5A by default)
  if (!drv.begin()) {
    Serial.println("DRV2605L not found on I2C, check wiring!");
    while (1) {
      delay(100);
    }
  }

  // Use internal trigger and library 1 (you can experiment)
  drv.selectLibrary(1);
  drv.setMode(DRV2605_MODE_INTTRIG);

  // Choose the effect to play.
  // Effect numbers are in the DRV2605 datasheet or Adafruit examples.
  // Here, use effect 1 which is a simple click.
  drv.setWaveform(0, 1);  // Slot 0: effect 1
  drv.setWaveform(1, 0);  // Slot 1: 0 = end of sequence

  Serial.println("DRV2605L init done");
}

void loop() {
  Serial.println("Playing haptic effect");
  drv.go();          // Start the effect in slot 0

  // Wait 5 seconds before triggering again
  delay(5000);
}
