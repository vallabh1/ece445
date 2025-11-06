#include <Arduino.h>
#include <SPI.h>
#include <BleMouse.h>

// Pins
static const int PIN_SPI_SCLK = 12;
static const int PIN_SPI_MOSI = 11;
static const int PIN_SPI_MISO = 13;
static const int PIN_CS_IMU1_ACC  = 5;
static const int PIN_CS_IMU1_GYRO = 6;
static const int PIN_CS_IMU2_ACC  = 7;
static const int PIN_CS_IMU2_GYRO = 8;
static const int PIN_ZUPT = 10;

// ===== BMI088 regs (verify) =====
#define BMI088_GYRO_CHIP_ID_REG   0x00
#define BMI088_GYRO_CHIP_ID_VAL   0x0F
#define BMI088_GYRO_RANGE_REG     0x0F
#define BMI088_GYRO_BW_REG        0x10
#define BMI088_GYRO_SOFTRESET_REG 0x14
#define BMI088_GYRO_SOFTRESET_VAL 0xB6
#define BMI088_GYRO_DATA_REG      0x02
#define BMI088_ACC_CHIP_ID_REG    0x00
#define BMI088_ACC_CHIP_ID_VAL    0x1E
#define BMI088_ACC_RANGE_REG      0x0F
#define BMI088_ACC_BW_REG         0x10
#define BMI088_ACC_SOFTRESET_REG  0x7E
#define BMI088_ACC_SOFTRESET_VAL  0xB6
#define BMI088_ACC_DATA_REG       0x12
static inline uint8_t BMI_READ(uint8_t r){ return r | 0x80; }
static inline uint8_t BMI_WRITE(uint8_t r){ return r & 0x7F; }

// ===== Rates / tuning =====
static const float LOOP_HZ   = 100.f;
static const float DT        = 1.f/LOOP_HZ;
static const float BLE_HZ    = 90.f;
static const float BLE_DT    = 1.f/BLE_HZ;
static const float W_GYRO_1  = 0.5f;    // TODO: set inverse-variance weights
static const float W_GYRO_2  = 0.5f;
static const float W_ACC_1   = 0.5f;
static const float W_ACC_2   = 0.5f;
static const float OMEGA_TH  = 0.3f;    // rad/s  TODO
static const float ACCG_TH   = 0.2f;    // g      TODO
static const uint32_t ZUPT_HOLD_MS = 100;
static const float PIXELS_PER_M = 3800.f; // TODO
static const float V_DEADBAND   = 0.02f;  // m/s   TODO
static const float MAX_STEP_PX  = 5.f;    // TODO

// ===== BLE HID =====
BleMouse bleMouse("GloveMouse","ECE445",100);

// ===== Types =====
struct ImuRaw { int16_t gx,gy,gz, ax,ay,az; };
struct ImuSI  { float   gx,gy,gz, ax,ay,az; };
struct Bias   { float   gx,gy,gz, ax,ay,az; };
struct Vel2   { float   vx,vy; };

// ===== Globals =====
SPIClass spiSPI(FSPI);
Bias bias1{0,0,0, 0,0,0}, bias2{0,0,0, 0,0,0};
Vel2 v_xy{0,0}, v_xy_prev{0,0};
uint32_t t_loop_us=0, t_ble_us=0, zupt_start=0;

// ===== SPI helpers =====
static inline void csSel(int cs){ digitalWrite(cs, LOW); }
static inline void csDes(int cs){ digitalWrite(cs, HIGH); }
uint8_t spiReadReg(int cs, uint8_t reg){
  csSel(cs); spiSPI.transfer(BMI_READ(reg)); uint8_t v=spiSPI.transfer(0x00); csDes(cs); return v;
}
void spiWriteReg(int cs, uint8_t reg, uint8_t val){
  csSel(cs); spiSPI.transfer(BMI_WRITE(reg)); spiSPI.transfer(val); csDes(cs);
}
void spiReadBurst(int cs, uint8_t startReg, uint8_t* buf, size_t n){
  csSel(cs); spiSPI.transfer(BMI_READ(startReg)); for(size_t i=0;i<n;++i) buf[i]=spiSPI.transfer(0x00); csDes(cs);
}

// ===== BMI088 init =====
bool bmi088InitOne(int csA,int csG){
  spiWriteReg(csG, BMI088_GYRO_SOFTRESET_REG, BMI088_GYRO_SOFTRESET_VAL); delay(50);
  spiWriteReg(csA, BMI088_ACC_SOFTRESET_REG,  BMI088_ACC_SOFTRESET_VAL);  delay(50);
  if (spiReadReg(csG,BMI088_GYRO_CHIP_ID_REG)!=BMI088_GYRO_CHIP_ID_VAL) return false;
  if (spiReadReg(csA,BMI088_ACC_CHIP_ID_REG )!=BMI088_ACC_CHIP_ID_VAL ) return false;
  // TODO: set exact ranges/ODR per datasheet
  spiWriteReg(csG, BMI088_GYRO_RANGE_REG, 0x01);
  spiWriteReg(csG, BMI088_GYRO_BW_REG,    0x02);
  spiWriteReg(csA, BMI088_ACC_RANGE_REG,  0x01);
  spiWriteReg(csA, BMI088_ACC_BW_REG,     0x02);
  delay(20); return true;
}
bool bmi088InitBoth(){
  bool ok1=bmi088InitOne(PIN_CS_IMU1_ACC,PIN_CS_IMU1_GYRO);
  bool ok2=bmi088InitOne(PIN_CS_IMU2_ACC,PIN_CS_IMU2_GYRO);
  return ok1&&ok2;
}

// ===== Read raw =====
void bmi088ReadOneRaw(int csA,int csG, ImuRaw& o){
  uint8_t g[6], a[6];
  spiReadBurst(csG, BMI088_GYRO_DATA_REG, g, 6);
  spiReadBurst(csA, BMI088_ACC_DATA_REG,  a, 6);
  o.gx=(int16_t)((g[1]<<8)|g[0]); o.gy=(int16_t)((g[3]<<8)|g[2]); o.gz=(int16_t)((g[5]<<8)|g[4]);
  o.ax=(int16_t)((a[1]<<8)|a[0]); o.ay=(int16_t)((a[3]<<8)|a[2]); o.az=(int16_t)((a[5]<<8)|a[4]);
}

// ===== Raw -> SI (TODO: exact scales) =====
void convertToSI(const ImuRaw& r, ImuSI& s, const Bias& b){
  const float gyroLSB2dps = 1.f/16.4f;  // TODO
  const float accLSB2g    = 1.f/8192.f; // TODO
  float gx_dps=r.gx*gyroLSB2dps, gy_dps=r.gy*gyroLSB2dps, gz_dps=r.gz*gyroLSB2dps;
  float ax_g=r.ax*accLSB2g, ay_g=r.ay*accLSB2g, az_g=r.az*accLSB2g;
  s.gx=(gx_dps*PI/180.f)-b.gx; s.gy=(gy_dps*PI/180.f)-b.gy; s.gz=(gz_dps*PI/180.f)-b.gz;
  s.ax=(ax_g*9.80665f)-b.ax;   s.ay=(ay_g*9.80665f)-b.ay;   s.az=(az_g*9.80665f)-b.az;
}

// ===== Fuse two IMUs =====
ImuSI fuseTwo(const ImuSI& a, const ImuSI& b){
  ImuSI o;
  o.gx=W_GYRO_1*a.gx + W_GYRO_2*b.gx;
  o.gy=W_GYRO_1*a.gy + W_GYRO_2*b.gy;
  o.gz=W_GYRO_1*a.gz + W_GYRO_2*b.gz;
  o.ax=W_ACC_1 *a.ax + W_ACC_2 *b.ax;
  o.ay=W_ACC_1 *a.ay + W_ACC_2 *b.ay;
  o.az=W_ACC_1 *a.az + W_ACC_2 *b.az;
  return o;
}

// ===== Stillness (ZUPT gate) =====
bool isStill(const ImuSI& f){
  float omg = sqrtf(f.gx*f.gx + f.gy*f.gy + f.gz*f.gz);
  float amag = sqrtf(f.ax*f.ax + f.ay*f.ay + f.az*f.az)/9.80665f;
  return (omg < OMEGA_TH) && (fabsf(amag-1.f) < ACCG_TH);
}

// ===== EKF placeholders =====
void ekfPredict(const ImuSI& fused, float dt){
  // TODO: integrate quaternion with fused.g*
  // TODO: rotate accel to nav/control frame, subtract gravity
}
void ekfUpdateZUPT(Vel2& v){
  // TODO: full Kalman correction
  v.vx=0.f; v.vy=0.f;
}
void ekfUpdateGravity(const ImuSI& fused){
  // TODO: tilt correction using accel dir
}

// ===== Planar velocity integration (x=right, y=up) =====
void integratePlanarVelocity(const ImuSI& fused, float dt, Vel2& v){
  // TODO: remove gravity via attitude; for now naive (replace in EKF completion)
  v.vx += fused.ax * dt;
  v.vy += fused.ay * dt;
}

// ===== Velocity -> HID =====
void velocityToMouse(const Vel2& v_now, const Vel2& v_prev, float dt, int8_t& dx, int8_t& dy){
  float vx = (fabsf(v_now.vx)<V_DEADBAND)?0.f:v_now.vx;
  float vy = (fabsf(v_now.vy)<V_DEADBAND)?0.f:v_now.vy;
  float dxf = PIXELS_PER_M * 0.5f * dt * (vx + v_prev.vx);
  float dyf = PIXELS_PER_M * 0.5f * dt * (vy + v_prev.vy);
  dxf = constrain(dxf, -MAX_STEP_PX, MAX_STEP_PX);
  dyf = constrain(dyf, -MAX_STEP_PX, MAX_STEP_PX);
  dx = (int8_t)lrintf(dxf);
  dy = (int8_t)lrintf(dyf);
}

// ===== Setup =====
void setup(){
  pinMode(PIN_CS_IMU1_ACC,OUTPUT);   pinMode(PIN_CS_IMU1_GYRO,OUTPUT);
  pinMode(PIN_CS_IMU2_ACC,OUTPUT);   pinMode(PIN_CS_IMU2_GYRO,OUTPUT);
  digitalWrite(PIN_CS_IMU1_ACC,HIGH); digitalWrite(PIN_CS_IMU1_GYRO,HIGH);
  digitalWrite(PIN_CS_IMU2_ACC,HIGH); digitalWrite(PIN_CS_IMU2_GYRO,HIGH);
  pinMode(PIN_ZUPT, INPUT_PULLDOWN);

  spiSPI.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI);
  spiSPI.setDataMode(SPI_MODE3);         // TODO: verify
  spiSPI.setFrequency(5'000'000);        // TODO: verify
  spiSPI.setBitOrder(MSBFIRST);

  Serial.begin(115200);
  bmi088InitBoth();                      // TODO: check return
  bleMouse.begin();

  uint32_t t=micros(); t_loop_us=t; t_ble_us=t;
}

// ===== Loop =====
void loop(){
  uint32_t now=micros();
  if (now - t_loop_us < (uint32_t)(1e6f*DT)) { delayMicroseconds(300); return; }
  t_loop_us += (uint32_t)(1e6f*DT);

  ImuRaw r1,r2; bmi088ReadOneRaw(PIN_CS_IMU1_ACC,PIN_CS_IMU1_GYRO,r1);
                 bmi088ReadOneRaw(PIN_CS_IMU2_ACC,PIN_CS_IMU2_GYRO,r2);
  ImuSI s1,s2;  convertToSI(r1,s1,bias1);
                convertToSI(r2,s2,bias2);
  ImuSI fused = fuseTwo(s1,s2);

  ekfPredict(fused, DT);

  bool pinch = digitalRead(PIN_ZUPT)==HIGH;
  bool still = isStill(fused);
  if (pinch && still){
    if (!zupt_start) zupt_start = millis();
    if (pinch || (millis()-zupt_start >= ZUPT_HOLD_MS)) { ekfUpdateZUPT(v_xy); zupt_start=0; }
  } else { zupt_start=0; }
  if (still && !pinch) ekfUpdateGravity(fused);

  v_xy_prev = v_xy;
  integratePlanarVelocity(fused, DT, v_xy);

  if (bleMouse.isConnected() && (now - t_ble_us) >= (uint32_t)(1e6f*BLE_DT)){
    t_ble_us += (uint32_t)(1e6f*BLE_DT);
    int8_t dx=0, dy=0;
    velocityToMouse(v_xy, v_xy_prev, BLE_DT, dx, dy);
    bleMouse.move(dx, -dy); // TODO: invert Y if needed for your OS
  }
}
