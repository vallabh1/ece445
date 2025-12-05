#include <Arduino.h>
#include <SPI.h>
#include <BMI088.h>
#include <math.h>
#define USE_NIMBLE
#include <BleMouse.h>

// ==== ESP32 VSPI pins ====
static const int PIN_SPI_SCK   = 18;
static const int PIN_SPI_MISO  = 19;
static const int PIN_SPI_MOSI  = 23;

// Accelerometer CSB1
static const int PIN_CS_ACC    = 5;
// Gyro CSB2 (matches IMU1 gyro CS in imuekf.cpp)
static const int PIN_CS_GYRO   = 25;

// Accelerometer-only object over SPI
Bmi088Accel accel(SPI, PIN_CS_ACC);
// Gyro object over SPI
Bmi088Gyro gyro(SPI, PIN_CS_GYRO);
// BLE Mouse
BleMouse bleMouse("GloveMouse","ECE445",100);

// Simple integrator state for velocity (m/s)
static float velX_mps = 0.0f;
static float velY_mps = 0.0f;
static float velZ_mps = 0.0f;

// Calibration state (estimate bias including gravity from initial samples)
static bool calibrated = false;
static const uint32_t CALIB_SAMPLES = 100;
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
static const float ACC_THRESHOLD_MSS = 0.15f;  // Deadzone threshold (m/s^2)
// Drift mitigation
static const float VEL_DAMPING_PER_SEC = 1.5f;  // Exponential decay rate for velocity (s^-1)
static const uint32_t ZUPT_MIN_SAMPLES = 8;     // Number of consecutive near-zero accel samples to zero velocity
static uint32_t zuptCount = 0;

// Tilt-based cursor control (no integration)
static const bool USE_TILT_CURSOR = true;      // Set true to use tilt-to-cursor mapping
static float neutralRoll_rad = 0.0f;           // Calibrated neutral orientation (roll)
static float neutralPitch_rad = 0.0f;          // Calibrated neutral orientation (pitch)
static const float CUR_DEADZONE_RAD = 2.0f * (3.1415926535f / 180.0f);  // 2 deg deadzone
static const float CUR_GAIN = 1200.0f;         // pixels/sec per radian (tune)
static const float CUR_MAX_VEL = 2000.0f;      // pixels/sec clamp
static const float CUR_SMOOTH_ALPHA = 0.2f;    // 0..1, higher = more smoothing
static float cursorVx = 0.0f;                  // px/s
static float cursorVy = 0.0f;                  // px/s

// Complementary filter for roll/pitch using gyro + accel
static float filtRoll_rad = 0.0f;
static float filtPitch_rad = 0.0f;
static const float ORIENT_ALPHA = 0.98f;    // Blend factor (gyro pred vs accel tilt)
// Gyro bias estimation
static float sumGx = 0.0f, sumGy = 0.0f, sumGz = 0.0f;
static float biasGx = 0.0f, biasGy = 0.0f, biasGz = 0.0f;

// BLE scheduling and cursor mapping
static const float BLE_HZ = 90.0f;
static const uint32_t BLE_PERIOD_US = (uint32_t)(1000000.0f / BLE_HZ);
static uint32_t lastBleMicros = 0;
static const bool INVERT_Y = true;          // Invert Y for typical screen coordinates
// Mapping for integrated velocity path (m/s -> px/s)
static const float PIXELS_PER_M = 3800.0f;  // Tune to taste
// Logging scheduler
static const uint32_t PRINT_PERIOD_MS = 100;
static uint32_t lastPrintMs = 0;

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

  // Init gyro
  int gstatus = gyro.begin();
  if (gstatus < 0) {
    Serial.print("Gyro begin() FAILED, status = ");
    Serial.println(gstatus);
    while (1) {
      delay(1000);
    }
  }
  Serial.println("Gyro initialized OK.");

  // Configure accel
  // This enum definitely exists in your version (we used it earlier)
  accel.setOdr(Bmi088Accel::ODR_200HZ_BW_38HZ);
  accel.setRange(Bmi088Accel::RANGE_6G);

  // Start BLE mouse
  bleMouse.begin();

  // Initialize timestamp after sensor setup
  lastUpdateMicros = micros();
  lastBleMicros = lastUpdateMicros;
}

void loop() {
  // Read latest accel sample into internal buffers
  accel.readSensor();
  gyro.readSensor();

  // Get acceleration in m/s^2
  float ax = accel.getAccelX_mss();
  float ay = accel.getAccelY_mss();
  float az = accel.getAccelZ_mss();
  // Get angular rates (rad/s)
  float gx = gyro.getGyroX_rads();
  float gy = gyro.getGyroY_rads();
  float gz = gyro.getGyroZ_rads();

  // Collect initial samples to estimate biases (including gravity)
  if (!calibrated) {
    sumAx += ax;
    sumAy += ay;
    sumAz += az;
    sumGx += gx;
    sumGy += gy;
    sumGz += gz;
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
      biasGx = sumGx / (float)calibCount;
      biasGy = sumGy / (float)calibCount;
      biasGz = sumGz / (float)calibCount;
      calibrated = true;
      lastUpdateMicros = micros();  // start timing after calibration

      // Compute neutral orientation from the averaged gravity vector
      // roll = atan2(ay, az), pitch = atan2(-ax, sqrt(ay^2 + az^2))
      neutralRoll_rad = atan2f(biasAy, biasAz);
      float gnorm = sqrtf(biasAy * biasAy + biasAz * biasAz);
      if (gnorm < 1e-6f) gnorm = 1e-6f;
      neutralPitch_rad = atan2f(-biasAx, gnorm);
      // Initialize filter state to neutral
      filtRoll_rad = neutralRoll_rad;
      filtPitch_rad = neutralPitch_rad;

      Serial.print("Calibration done. Biases [m/s^2] ax=");
      Serial.print(biasAx, 4);
      Serial.print(" ay=");
      Serial.print(biasAy, 4);
      Serial.print(" az=");
      Serial.println(biasAz, 4);
      Serial.print("Gyro biases [rad/s] gx=");
      Serial.print(biasGx, 5);
      Serial.print(" gy=");
      Serial.print(biasGy, 5);
      Serial.print(" gz=");
      Serial.println(biasGz, 5);
      Serial.print("Neutral roll/pitch [deg] r=");
      Serial.print(neutralRoll_rad * 180.0f / 3.1415926535f, 2);
      Serial.print(" p=");
      Serial.println(neutralPitch_rad * 180.0f / 3.1415926535f, 2);
    }
  }

  // Keep raw measurements for tilt computation
  float rawAx = ax;
  float rawAy = ay;
  float rawAz = az;
  float rawGx = gx;
  float rawGy = gy;
  float rawGz = gz;

  // Remove estimated biases for integration path
  ax -= biasAx;
  ay -= biasAy;
  az -= biasAz;
  gx -= biasGx;
  gy -= biasGy;
  gz -= biasGz;

  // Deadzone clipping to suppress noise and small biases
  if (fabsf(ax) < ACC_THRESHOLD_MSS) ax = 0.0f;
  if (fabsf(ay) < ACC_THRESHOLD_MSS) ay = 0.0f;
  if (fabsf(az) < ACC_THRESHOLD_MSS) az = 0.0f;

  // Stationary detection for Zero-Velocity Update (ZUPT)
  if (ax == 0.0f && ay == 0.0f && az == 0.0f) {
    if (zuptCount < 0xFFFFFFFFu) zuptCount++;
  } else {
    zuptCount = 0;
  }

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

  // Update complementary filter (gyro prediction + accel correction)
  if (calibrated && dt > 0.0f) {
    // Gyro integrate
    float predRoll  = filtRoll_rad  + gx * dt; // roll rate ~ gx
    float predPitch = filtPitch_rad + gy * dt; // pitch rate ~ gy
    // Accel tilt
    float accRoll  = atan2f(rawAy, rawAz);
    float gnorm_now = sqrtf(rawAy * rawAy + rawAz * rawAz);
    if (gnorm_now < 1e-6f) gnorm_now = 1e-6f;
    float accPitch = atan2f(-rawAx, gnorm_now);
    // Gate accel when linear acceleration is high
    float amag = sqrtf(rawAx*rawAx + rawAy*rawAy + rawAz*rawAz);
    float alpha = ORIENT_ALPHA;
    if (fabsf(amag - G_MSS) > 0.8f) { // >0.8 m/s^2 away from 1g -> trust gyro only
      alpha = 1.0f;
    }
    filtRoll_rad  = alpha * predRoll  + (1.0f - alpha) * accRoll;
    filtPitch_rad = alpha * predPitch + (1.0f - alpha) * accPitch;
  }

  // Integrate acceleration to velocity
  if (calibrated && dt > 0.0f) {
    velX_mps += ax * dt;
    velY_mps += ay * dt;
    velZ_mps += az * dt;

    // Apply exponential velocity damping to limit drift
    float decay = 1.0f - (VEL_DAMPING_PER_SEC * dt);
    if (decay < 0.0f) decay = 0.0f;
    velX_mps *= decay;
    velY_mps *= decay;
    velZ_mps *= decay;

    // If stationary for enough samples, zero velocities (ZUPT)
    if (zuptCount >= ZUPT_MIN_SAMPLES) {
      velX_mps = 0.0f;
      velY_mps = 0.0f;
      velZ_mps = 0.0f;
    }
  }

  // Tilt-to-cursor mapping (recommended for natural, drift-free control)
  if (calibrated && USE_TILT_CURSOR) {
    // Use fused roll/pitch from complementary filter
    float roll = filtRoll_rad;
    float pitch = filtPitch_rad;

    float dRoll = roll - neutralRoll_rad;
    float dPitch = pitch - neutralPitch_rad;

    // Wrap deltas to [-pi, pi] to avoid large jumps
    auto wrapPi = [](float a) -> float {
      while (a > 3.1415926535f) a -= 2.0f * 3.1415926535f;
      while (a < -3.1415926535f) a += 2.0f * 3.1415926535f;
      return a;
    };
    dRoll = wrapPi(dRoll);
    dPitch = wrapPi(dPitch);

    // Deadzone for small tilts
    auto applyDeadzone = [](float x, float dz) -> float {
      if (fabsf(x) <= dz) return 0.0f;
      return (x > 0.0f) ? (x - dz) : (x + dz);
    };
    dRoll = applyDeadzone(dRoll, CUR_DEADZONE_RAD);
    dPitch = applyDeadzone(dPitch, CUR_DEADZONE_RAD);

    // Map tilt to cursor velocity (px/s). Pitch -> up/down (Y), Roll -> left/right (X)
    float targetVx = CUR_GAIN * (-dRoll);   // left/right
    float targetVy = CUR_GAIN * (-dPitch);  // up/down

    // Clamp
    if (targetVx > CUR_MAX_VEL) targetVx = CUR_MAX_VEL;
    if (targetVx < -CUR_MAX_VEL) targetVx = -CUR_MAX_VEL;
    if (targetVy > CUR_MAX_VEL) targetVy = CUR_MAX_VEL;
    if (targetVy < -CUR_MAX_VEL) targetVy = -CUR_MAX_VEL;

    // Smooth
    cursorVx = CUR_SMOOTH_ALPHA * cursorVx + (1.0f - CUR_SMOOTH_ALPHA) * targetVx;
    cursorVy = CUR_SMOOTH_ALPHA * cursorVy + (1.0f - CUR_SMOOTH_ALPHA) * targetVy;
  }

  // BLE mouse move at fixed rate
  uint32_t nowBle = nowMicros;
  if (bleMouse.isConnected() && (nowBle - lastBleMicros) >= BLE_PERIOD_US) {
    float dtBle = (nowBle - lastBleMicros) * 1e-6f;
    lastBleMicros = nowBle;
    // Decide mapping source: tilt cursor (px/s) or integrated velocity (m/s)
    float vx_px_s = 0.0f;
    float vy_px_s = 0.0f;
    if (USE_TILT_CURSOR) {
      vx_px_s = cursorVx;
      vy_px_s = cursorVy;
    } else {
      vx_px_s = PIXELS_PER_M * velX_mps;
      vy_px_s = PIXELS_PER_M * velY_mps;
    }
    // Integrate to pixel steps for this tick
    float dx_f = vx_px_s * dtBle;
    float dy_f = vy_px_s * dtBle;
    // Clamp to int8 range
    dx_f = constrain(dx_f, -127.0f, 127.0f);
    dy_f = constrain(dy_f, -127.0f, 127.0f);
    int8_t dx = (int8_t)lrintf(dx_f);
    int8_t dy = (int8_t)lrintf(dy_f);
    if (INVERT_Y) dy = (int8_t)-dy;
    bleMouse.move(dx, dy);
  }

  // Non-blocking print at ~10 Hz
  uint32_t nowMs = millis();
  // if (nowMs - lastPrintMs >= PRINT_PERIOD_MS) {
  //   lastPrintMs = nowMs;
  //   Serial.print("ACC [m/s^2]  x=");
  //   Serial.print(ax, 3);
  //   Serial.print("  y=");
  //   Serial.print(ay, 3);
  //   Serial.print("  z=");
  //   Serial.print(az, 3);
  //   Serial.print("  |  VEL [m/s]  x=");
  //   Serial.print(velX_mps, 3);
  //   Serial.print("  y=");
  //   Serial.print(velY_mps, 3);
  //   Serial.print("  z=");
  //   Serial.print(velZ_mps, 3);
  //   if (USE_TILT_CURSOR) {
  //     Serial.print("  |  CUR [px/s]  vx=");
  //     Serial.print(cursorVx, 1);
  //     Serial.print(" vy=");
  //     Serial.print(cursorVy, 1);
  //   }
  //   Serial.println();
  }
}
