#include <Arduino.h>
#include <SPI.h>
#include <BMI088.h>
#include <math.h>

#include <BleMouse.h>
#include <Wire.h>
#include <Adafruit_DRV2605.h>

// ======================= BLE Mouse =======================

BleMouse bleMouse("TestMouse", "ECE445", 100);

// ======================= Pins ============================

// SPI for BMI088 (ESP32 VSPI)
static const int PIN_SPI_SCK   = 14;
static const int PIN_SPI_MISO  = 40;
static const int PIN_SPI_MOSI  = 15;
static const int PIN_CS_ACC    = 10;   // Accelerometer CSB1

// Hall effect inputs (active LOW)
const int HALL1_PIN = 1;  // Hall 1 → left click
const int HALL2_PIN = 2;  // Hall 2 → right click
const int HALL3_PIN = 3;  // Hall 3 → scroll down (NO debounce)
const int HALL4_PIN = 4;  // Hall 4 → scroll up (NO debounce)

// Reset button (active LOW)
const int RESET_PIN = 5;  // Recalibrate gravity and orientation

// Custom I2C pins for DRV2605
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9

// ======================= DRV2605 =========================

#define DRV2605_REG_MODE       0x01
#define DRV2605_MODE_INTTRIG   0x00

Adafruit_DRV2605 haptics;

// ======================= BMI088 accel ====================

Bmi088Accel accel(SPI, PIN_CS_ACC);

// Gravity and scaling
static const float G_MSS = 9.80665f;

// Cursor scaling: ±9.8 m/s^2 maps to ±CURSOR_MAX_STEP pixels per update
static const int   CURSOR_MAX_STEP = 12;    // tune this for speed
static const float DEADZONE_MSS    = 1.0f; // deadzone on linear accel

// Smoothed values in canonical "right/up" space (normalized by g)
static float filtRightNorm = 0.0f;
static float filtUpNorm    = 0.0f;
static const float SMOOTH_ALPHA = 0.2f;   // higher means more responsive, less smooth

// ---------- General gravity-based orientation ----------

// Estimate gravity by averaging initial samples
static bool     gravityInitialized   = false;
static uint32_t gravitySampleCount   = 0;
static const uint32_t GRAVITY_SAMPLES = 500;

// Reference gravity vector in sensor frame
static float gRefX = 0.0f;
static float gRefY = 0.0f;
static float gRefZ = 0.0f;

// Canonical basis vectors in sensor frame
// eRight = direction that means "mouse right"
// eUp    = direction that means "mouse up"
static float eRightX = 0.0f;
static float eRightY = 0.0f;
static float eRightZ = 0.0f;

static float eUpX = 0.0f;
static float eUpY = 0.0f;
static float eUpZ = 0.0f;

// Mouse update timing
static const uint32_t MOUSE_UPDATE_MS = 10;
static uint32_t lastMouseUpdateMs = 0;

// ======================= Debounce for Buttons ========================

struct HallButton {
  int pin;
  bool stableState;
  bool lastReading;
  unsigned long lastChange;
};

const unsigned long DEBOUNCE_MS = 20;   // for click buttons and reset

HallButton hallClicks[2] = {
  {HALL1_PIN, HIGH, HIGH, 0},
  {HALL2_PIN, HIGH, HIGH, 0},
};

// Separate debounced reset button
HallButton resetButton = {RESET_PIN, HIGH, HIGH, 0};

// Returns true on new debounced press (HIGH → LOW)
bool updateDebouncedClick(HallButton &btn) {
  unsigned long now = millis();
  bool raw = digitalRead(btn.pin);

  if (raw != btn.lastReading) {
    btn.lastReading = raw;
    btn.lastChange  = now;
  }

  if ((now - btn.lastChange) >= DEBOUNCE_MS) {
    if (raw != btn.stableState) {
      btn.stableState = raw;
      if (btn.stableState == LOW) {
        return true;  // new press
      }
    }
  }
  return false;
}

// ======================= Haptics ========================

void triggerHaptic(uint8_t effectID = 1) {
  haptics.setWaveform(0, effectID);
  haptics.setWaveform(1, 0);
  haptics.go();
}

// Helper to normalize a 3D vector; returns length (before normalization)
float normalize3(float &x, float &y, float &z) {
  float n = sqrtf(x*x + y*y + z*z);
  if (n > 1e-6f) {
    x /= n;
    y /= n;
    z /= n;
  }
  return n;
}

// Start or restart gravity calibration
void startGravityCalibration() {
  gravityInitialized   = false;
  gravitySampleCount   = 0;
  gRefX = gRefY = gRefZ = 0.0f;

  // Reset filters so motion starts clean after recalibration
  filtRightNorm = 0.0f;
  filtUpNorm    = 0.0f;

  Serial.println("Calibration reset requested. Hold the device still in new neutral pose.");
  // Optional stronger haptic to signal recalibration
  triggerHaptic(15);  // change effect if you like
}

// ======================= Accel → Mouse ===================

void updateAccelAndMouse() {
  // Read accel
  accel.readSensor();
  float ax = accel.getAccelX_mss();
  float ay = accel.getAccelY_mss();
  float az = accel.getAccelZ_mss();

  // ---------- 1) Estimate gravity direction at startup or after reset ----------
  if (!gravityInitialized) {
    gravitySampleCount++;
    gRefX += ax;
    gRefY += ay;
    gRefZ += az;

    if (gravitySampleCount >= GRAVITY_SAMPLES) {
      gRefX /= (float)gravitySampleCount;
      gRefY /= (float)gravitySampleCount;
      gRefZ /= (float)gravitySampleCount;

      // Unit gravity direction gDir
      float gDirX = gRefX;
      float gDirY = gRefY;
      float gDirZ = gRefZ;
      normalize3(gDirX, gDirY, gDirZ);

      // Decide which axis gravity is closest to
      float absX = fabsf(gDirX);
      float absY = fabsf(gDirY);
      float absZ = fabsf(gDirZ);

      // Desired "right" and "up" directions for the 6 cardinal cases
      // These match your patterns:
      // g ~ -Z: right = -X, up = -Y
      // g ~ -X: right = +Z, up = -Y
      // g ~ +Y: right = -Z, up = -X
      // Others are symmetric.
      float refRightX, refRightY, refRightZ;    // desired right
      float desiredUpX, desiredUpY, desiredUpZ; // desired up

      if (absZ >= absX && absZ >= absY) {
        // Gravity mostly along Z
        if (gDirZ < 0.0f) {
          // g ~ -Z
          refRightX = -1.0f; refRightY =  0.0f; refRightZ = 0.0f;  // right = -X
          desiredUpX =  0.0f; desiredUpY = -1.0f; desiredUpZ = 0.0f; // up = -Y
          Serial.println("Orientation: g ~ -Z");
        } else {
          // g ~ +Z
          refRightX = -1.0f; refRightY =  0.0f; refRightZ = 0.0f;   // right = -X
          desiredUpX =  0.0f; desiredUpY =  1.0f; desiredUpZ = 0.0f; // up = +Y
          Serial.println("Orientation: g ~ +Z");
        }
      } else if (absX >= absY && absX >= absZ) {
        // Gravity mostly along X
        if (gDirX < 0.0f) {
          // g ~ -X
          refRightX =  0.0f; refRightY = 0.0f; refRightZ =  1.0f;   // right = +Z
          desiredUpX =  0.0f; desiredUpY = -1.0f; desiredUpZ = 0.0f; // up = -Y
          Serial.println("Orientation: g ~ -X");
        } else {
          // g ~ +X
          refRightX =  0.0f; refRightY = 0.0f; refRightZ = -1.0f;   // right = -Z
          desiredUpX =  0.0f; desiredUpY = -1.0f; desiredUpZ = 0.0f; // up = -Y
          Serial.println("Orientation: g ~ +X");
        }
      } else {
        // Gravity mostly along Y
        if (gDirY > 0.0f) {
          // g ~ +Y
          refRightX =  0.0f; refRightY = 0.0f; refRightZ = -1.0f;    // right = -Z
          desiredUpX = -1.0f; desiredUpY = 0.0f; desiredUpZ = 0.0f;   // up = -X
          Serial.println("Orientation: g ~ +Y");
        } else {
          // g ~ -Y
          refRightX =  0.0f; refRightY = 0.0f; refRightZ =  1.0f;    // right = +Z
          desiredUpX = -1.0f; desiredUpY = 0.0f; desiredUpZ = 0.0f;   // up = -X
          Serial.println("Orientation: g ~ -Y");
        }
      }

      // Project refRight onto plane perpendicular to gravity
      float dotRG = refRightX * gDirX + refRightY * gDirY + refRightZ * gDirZ;

      float rX = refRightX - dotRG * gDirX;
      float rY = refRightY - dotRG * gDirY;
      float rZ = refRightZ - dotRG * gDirZ;

      if (normalize3(rX, rY, rZ) < 1e-3f) {
        // Very degenerate: pick any vector not parallel to gDir
        if (fabsf(gDirX) < fabsf(gDirY)) {
          rX = 0.0f;     rY = -gDirZ; rZ = gDirY;
        } else {
          rX = -gDirZ;   rY = 0.0f;   rZ = gDirX;
        }
        normalize3(rX, rY, rZ);
      }

      // Up = gDir × right (then flipped if needed)
      float uX = gDirY * rZ - gDirZ * rY;
      float uY = gDirZ * rX - gDirX * rZ;
      float uZ = gDirX * rY - gDirY * rX;
      normalize3(uX, uY, uZ);

      // Flip up if opposite to desiredUp
      float dotUp = uX * desiredUpX + uY * desiredUpY + uZ * desiredUpZ;
      if (dotUp < 0.0f) {
        uX = -uX; uY = -uY; uZ = -uZ;
      }

      eRightX = rX; eRightY = rY; eRightZ = rZ;
      eUpX    = uX; eUpY    = uY; eUpZ    = uZ;

      Serial.print("gRef = [");
      Serial.print(gRefX, 3); Serial.print(", ");
      Serial.print(gRefY, 3); Serial.print(", ");
      Serial.print(gRefZ, 3); Serial.println("] m/s^2");

      Serial.print("eRight = [");
      Serial.print(eRightX, 3); Serial.print(", ");
      Serial.print(eRightY, 3); Serial.print(", ");
      Serial.print(eRightZ, 3); Serial.println("]");

      Serial.print("eUp = [");
      Serial.print(eUpX, 3); Serial.print(", ");
      Serial.print(eUpY, 3); Serial.print(", ");
      Serial.print(eUpZ, 3); Serial.println("]");

      gravityInitialized = true;
      // Haptic ping when calibration finishes
      triggerHaptic(1);
      static uint32_t frameCount = 0;
static uint32_t lastReportMs = 0;

frameCount++;

uint32_t nowMs = millis();
if (nowMs - lastReportMs >= 1000) {
  float dtSec = (nowMs - lastReportMs) / 1000.0f;
  float freq  = frameCount / dtSec;   // Hz

  Serial.print("Orientation update rate (Hz): ");
  Serial.println(freq);

  // Simple assert check for requirement
  if (freq < 100.0f) {
    Serial.println("WARNING: update rate below 100 Hz requirement");
  }

  frameCount = 0;
  lastReportMs = nowMs;
}

    }

    // Do not move mouse until we have gravity and basis
    return;
  }

  // ---------- 2) Use accel relative to gravity as control signal ----------

  // Remove gravity reference so neutral pose is zero
  float dAx = ax - gRefX;
  float dAy = ay - gRefY;
  float dAz = az - gRefZ;

  // Deadzone on linear accel (m/s^2)
  if (fabsf(dAx) < DEADZONE_MSS) dAx = 0.0f;
  if (fabsf(dAy) < DEADZONE_MSS) dAy = 0.0f;
  if (fabsf(dAz) < DEADZONE_MSS) dAz = 0.0f;

  // Normalize by g to get something like [-1,1] range
  float nX = dAx / G_MSS;
  float nY = dAy / G_MSS;
  float nZ = dAz / G_MSS;

  // Project onto canonical "right" and "up" directions
  float rightNormRaw = nX*eRightX + nY*eRightY + nZ*eRightZ;
  float upNormRaw    = nX*eUpX    + nY*eUpY    + nZ*eUpZ;

  // Exponential smoothing
  filtRightNorm = SMOOTH_ALPHA * rightNormRaw + (1.0f - SMOOTH_ALPHA) * filtRightNorm;
  filtUpNorm    = SMOOTH_ALPHA * upNormRaw    + (1.0f - SMOOTH_ALPHA) * filtUpNorm;

  // Limit to [-1, 1]
  filtRightNorm = constrain(filtRightNorm, -1.0f, 1.0f);
  filtUpNorm    = constrain(filtUpNorm,    -1.0f, 1.0f);

  // Only update mouse at fixed rate
  uint32_t nowMs = millis();
  if (nowMs - lastMouseUpdateMs < MOUSE_UPDATE_MS) {
    return;
  }
  lastMouseUpdateMs = nowMs;

  if (!bleMouse.isConnected()) {
    return;
  }

  // Small deadzone on normalized values
  const float DEADZONE_NORM = 0.02f;
  float fx = (fabsf(filtRightNorm) < DEADZONE_NORM) ? 0.0f : filtRightNorm;
  float fy = (fabsf(filtUpNorm)    < DEADZONE_NORM) ? 0.0f : filtUpNorm;

  // Map to mouse deltas
  // Positive fx → cursor right
  // Positive fy → cursor down (if you want "tilt up → cursor up", flip sign here)
  int dx = (int)roundf(fx * CURSOR_MAX_STEP);
  int dy = (int)roundf(fy * CURSOR_MAX_STEP);

  dx = constrain(dx, -CURSOR_MAX_STEP, CURSOR_MAX_STEP);
  dy = constrain(dy, -CURSOR_MAX_STEP, CURSOR_MAX_STEP);

  if (dx != 0 || dy != 0) {
    bleMouse.move((int8_t)dx, (int8_t)-dy, 0, 0);
  }

  // Debug
  Serial.print("a=(");
  Serial.print(ax, 3); Serial.print(",");
  Serial.print(ay, 3); Serial.print(",");
  Serial.print(az, 3); Serial.print(") ");
  Serial.print("d=(");
  Serial.print(dAx, 3); Serial.print(",");
  Serial.print(dAy, 3); Serial.print(",");
  Serial.print(dAz, 3); Serial.print(") ");
  Serial.print("fx="); Serial.print(fx, 3);
  Serial.print(" fy="); Serial.print(fy, 3);
  Serial.print(" dx="); Serial.print(dx);
  Serial.print(" dy="); Serial.println(dy);
}

// ======================= Setup ==========================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("BMI088 + BLE Mouse + Hall + Haptics (general gravity + reset + 6-case anchored) ");

  // Pins for Hall switches
  pinMode(HALL1_PIN, INPUT_PULLUP);
  pinMode(HALL2_PIN, INPUT_PULLUP);
  pinMode(HALL3_PIN, INPUT_PULLUP);
  pinMode(HALL4_PIN, INPUT_PULLUP);

  // Reset button
  pinMode(RESET_PIN, INPUT_PULLUP);

  // I2C for DRV2605
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  if (!haptics.begin()) {
    Serial.println("DRV2605 not found");
    while (1) {
      delay(100);
    }
  }
  haptics.selectLibrary(1);
  haptics.setMode(DRV2605_MODE_INTTRIG);

  // SPI for BMI088
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);

  int status = accel.begin();
  if (status < 0) {
    Serial.print("Accel begin() FAILED, status = ");
    Serial.println(status);
    while (1) {
      delay(1000);
    }
  }
  Serial.println("Accel initialized OK.");

  // Initial calibration will run automatically after startup
  startGravityCalibration();

  bleMouse.begin();
}

// ======================= Loop ===========================

void loop() {
  // Check reset button first so you can recalibrate any time
  if (updateDebouncedClick(resetButton)) {
    Serial.println("Reset button pressed.");
    startGravityCalibration();
    // True centering of cursor is not possible with relative HID,
    // so we only reset orientation here.
  }

  // Update accel and cursor motion
  updateAccelAndMouse();

  // Even if mouse not connected, we still want to keep debounce state updated
  bool isConnected = bleMouse.isConnected();

  // ---------- CLICK BUTTONS WITH DEBOUNCE ----------
  // if (updateDebouncedClick(hallClicks[0]) && isConnected) {
  //   Serial.println("Left click");
  //   bleMouse.click(MOUSE_LEFT);
  //   triggerHaptic();
  // }

  // if (updateDebouncedClick(hallClicks[1]) && isConnected) {
  //   Serial.println("Right click");
  //   bleMouse.click(MOUSE_RIGHT);
  //   triggerHaptic();
  // }

  if (isConnected) {
  if (digitalRead(HALL1_PIN) == LOW) {
    Serial.println("Left click");
    bleMouse.click(MOUSE_LEFT);
    triggerHaptic();
  }

  if (isConnected) {
  if (digitalRead(HALL2_PIN) == LOW) {
    Serial.println("Right click");
    bleMouse.click(MOUSE_RIGHT);
    triggerHaptic();  }


  // ---------- SCROLL BUTTONS WITHOUT DEBOUNCE ----------
  if (isConnected) {
    if (digitalRead(HALL3_PIN) == LOW) {
      Serial.println("Scroll down");
      bleMouse.move(0, 0, -1, 0);   // scroll down
      triggerHaptic();
      delay(40);   // repeat speed limit
    }

    if (digitalRead(HALL4_PIN) == LOW) {
      Serial.println("Scroll up");
      bleMouse.move(0, 0, 1, 0);    // scroll up
      triggerHaptic();
      delay(40);   // repeat speed limit
    }
  }

  // Small base delay, rest of timing inside functions
  delay(1);
}
