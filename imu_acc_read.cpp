#include <Arduino.h>
#include <SPI.h>
#include <BMI088.h>

// ==== ESP32 VSPI pins ====
static const int PIN_SPI_SCK   = 18;
static const int PIN_SPI_MISO  = 19;
static const int PIN_SPI_MOSI  = 23;

// Accelerometer CSB1
static const int PIN_CS_ACC    = 5;

// Accelerometer-only object over SPI
Bmi088Accel accel(SPI, PIN_CS_ACC);

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("BMI088Accel (BolderFlight) + ESP32, accel only");

  // Init SPI bus on your pins
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

  // Init accelerometer
  int status = accel.begin();
  if (status < 0) {
    Serial.print("Accel begin() FAILED, status = ");
    Serial.println(status);
    while (1) {
      delay(1000);
    }
  }
  Serial.println("Accel initialized OK.");

  // Configure accel
  // This enum definitely exists in your version (we used it earlier)
  accel.setOdr(Bmi088Accel::ODR_200HZ_BW_38HZ);
  accel.setRange(Bmi088Accel::RANGE_6G);
}

void loop() {
  // Read latest accel sample into internal buffers
  accel.readSensor();

  // Get acceleration in m/s^2
  float ax = accel.getAccelX_mss();
  float ay = accel.getAccelY_mss();
  float az = accel.getAccelZ_mss();

  Serial.print("ACC [m/s^2]  x=");
  Serial.print(ax, 3);
  Serial.print("  y=");
  Serial.print(ay, 3);
  Serial.print("  z=");
  Serial.print(az, 3);
  Serial.println();

  delay(100);  // ~10 Hz print
}
