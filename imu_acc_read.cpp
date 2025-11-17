#include <Arduino.h>
#include <SPI.h>
#include <BMI088.h>
#include <math.h>

// ==== ESP32 VSPI pins ====
static const int PIN_SPI_SCK   = 18;
static const int PIN_SPI_MISO  = 19;
static const int PIN_SPI_MOSI  = 23;

// Accelerometer CSB1
static const int PIN_CS_ACC    = 5;

// Accelerometer-only object over SPI
Bmi088Accel accel(SPI, PIN_CS_ACC);

// Simple integrator state for velocity (m/s)
static float velX_mps = 0.0f;
static float velY_mps = 0.0f;
static float velZ_mps = 0.0f;

// Calibration state (estimate bias including gravity from initial samples)
static bool calibrated = false;
static const uint32_t CALIB_SAMPLES = 500;
static uint32_t calibCount = 0;
static float sumAx = 0.0f;
static float sumAy = 0.0f;
static float sumAz = 0.0f;
static float biasAx = 0.0f;
static float biasAy = 0.0f;
static float biasAz = 0.0f;

// Timestamp for integration
static uint32_t lastUpdateMicros = 0;

// Constants
static const float G_MSS = 9.80665f;        // Gravity in m/s^2
static const float ACC_THRESHOLD_MSS = 0.40f;  // Deadzone threshold (m/s^2)

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

  // Initialize timestamp after sensor setup
  lastUpdateMicros = micros();
}

void loop() {
  // Read latest accel sample into internal buffers
  accel.readSensor();

  // Get acceleration in m/s^2
  float ax = accel.getAccelX_mss();
  float ay = accel.getAccelY_mss();
  float az = accel.getAccelZ_mss();

  // Collect initial samples to estimate biases (including gravity)
  if (!calibrated) {
    sumAx += ax;
    sumAy += ay;
    sumAz += az;
    calibCount++;

    if ((calibCount % 50) == 0 || calibCount == 1) {
      Serial.print("Calibrating... ");
      Serial.print(calibCount);
      Serial.print("/");
      Serial.println(CALIB_SAMPLES);
    }

    if (calibCount >= CALIB_SAMPLES) {
      biasAx = sumAx / (float)calibCount;
      biasAy = sumAy / (float)calibCount;
      biasAz = sumAz / (float)calibCount;
      calibrated = true;
      lastUpdateMicros = micros();  // start timing after calibration

      Serial.print("Calibration done. Biases [m/s^2] ax=");
      Serial.print(biasAx, 4);
      Serial.print(" ay=");
      Serial.print(biasAy, 4);
      Serial.print(" az=");
      Serial.println(biasAz, 4);
    }
  }

  // Remove estimated biases
  ax -= biasAx;
  ay -= biasAy;
  az -= biasAz;

  // Deadzone clipping to suppress noise and small biases
  if (fabsf(ax) < ACC_THRESHOLD_MSS) ax = 0.0f;
  if (fabsf(ay) < ACC_THRESHOLD_MSS) ay = 0.0f;
  if (fabsf(az) < ACC_THRESHOLD_MSS) az = 0.0f;

  // Time delta for integration (seconds)
  uint32_t nowMicros = micros();
  float dt = 0.0f;
  if (calibrated) {
    dt = (nowMicros - lastUpdateMicros) * 1e-6f;
    // Protect against unusually large dt (e.g., if Serial blocks)
    if (dt > 0.5f) dt = 0.5f;
    lastUpdateMicros = nowMicros;
  } else {
    // Keep timestamp fresh while calibrating
    lastUpdateMicros = nowMicros;
  }

  // Integrate acceleration to velocity
  if (calibrated && dt > 0.0f) {
    velX_mps += ax * dt;
    velY_mps += ay * dt;
    velZ_mps += az * dt;
  }

  Serial.print("ACC [m/s^2]  x=");
  Serial.print(ax, 3);
  Serial.print("  y=");
  Serial.print(ay, 3);
  Serial.print("  z=");
  Serial.print(az, 3);
  Serial.print("  |  VEL [m/s]  x=");
  Serial.print(velX_mps, 3);
  Serial.print("  y=");
  Serial.print(velY_mps, 3);
  Serial.print("  z=");
  Serial.print(velZ_mps, 3);
  Serial.println();

  delay(100);  // ~10 Hz print
}
