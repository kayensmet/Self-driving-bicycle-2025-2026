#include <Arduino.h>
#include <Wire.h>
#include <DShotRMT.h>
#include <AutoLQRStatic.h>
#include <esp_now.h>
#include <WiFi.h>
 
// ════════════════════════════════════════════════════════════
//  CONFIGURATIE 
// ════════════════════════════════════════════════════════════
 
// ── Motor pinnen ────────────────────────────────────────────
#define MOTOR1_GPIO             GPIO_NUM_18
#define MOTOR2_GPIO             GPIO_NUM_19
 
// ── I2C ─────────────────────────────────────────────────────
#define I2C_SDA_PIN             21
#define I2C_SCL_PIN             22
 
// ── IMU timing ──────────────────────────────────────────────
#define IMU_INTERVAL_MS         10      // ms tussen IMU-reads
 
// ── Kalibratie offset (graden) ──────────────────────────────
//    Meet de rusthoek van de fiets en vul hier in
#define BALANCE_OFFSET          0.0f    // pitch offset (graden)
#define ROLL_OFFSET             0.0f    // roll offset  (graden)
 
// ── LQR instellingen ────────────────────────────────────────
#define TARGET_ANGLE            -1.0f    // gewenste balanshoek (graden)
#define Q_ANGLE                30.0f    // LQR kost: hoekfout
#define Q_RATE                  1.5f    // LQR kost: hoeksnelheid
#define R_CONTROL               0.1f    // LQR kost: stuurinput
#define MODEL_DAMPING           0.98f   // model demping (0.0 – 1.0)
#define MODEL_INPUT_SCALE       1.0f    // schaling model input
#define CONTROL_SCALE          35.0f    // LQR output → throttle schaling
 
// ── Motor instellingen ──────────────────────────────────────
#define MOTOR_BASE            250.0f    // basis toerental (idle), 48 – 500
#define MOTOR_TRIM              0.0f    // trim motor 1 vs 2 (-200 tot +200)
 
// ── Filter ──────────────────────────────────────────────────
#define RATE_FILTER_ALPHA       0.20f   // laagdoorlaatfilter hoeksnelheid (0–1)
 
// ════════════════════════════════════════════════════════════
//  Einde configuratie
// ════════════════════════════════════════════════════════════
 
// ────────────────────────────────────────────────────────────
//  ESP-NOW — commando's & berichtstructuur
// ────────────────────────────────────────────────────────────
enum Commands : uint8_t {
  CMD_OK         = 0x01,
  CMD_STOP       = 0x02,
  CMD_BALANS     = 0x03,  // ← consistente naam
  CMD_BUZZ       = 0x05,
  CMD_SPEED_DATA = 0x0A
};

typedef struct struct_message {
  uint8_t sender;
  uint8_t command;
  int     throttle;
  int     brake;
  int     sturen;
  float   value;
  bool    buzzer;
  bool    balansAan;
} struct_message;

struct_message incomingMsg;

volatile bool balanceActive = false;

void onDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
  memcpy(&incomingMsg, incomingData, sizeof(incomingMsg));

  switch (incomingMsg.command) {

    case CMD_BALANS:  // ← FIX: was CMD_BALANCE
      balanceActive = !balanceActive;
      Serial.print("ESP-NOW: CMD_BALANS → regelaar ");
      Serial.println(balanceActive ? "AAN" : "UIT");
      break;

    case CMD_OK:
      // Heartbeat — geen actie nodig
      break;

    default:
      break;
  }
}
 
// ────────────────────────────────────────────────────────────
//  IMU
// ────────────────────────────────────────────────────────────
const int MPU1 = 0x68;
 
float pitch = 0.0f;
float roll  = 0.0f;
 
float absolutePitchRaw = 0.0f;
float absoluteRollRaw  = 0.0f;
bool  firstLoop        = true;
float angleRateFilt    = 0.0f;
 
unsigned long lastTime = 0;
 
// ────────────────────────────────────────────────────────────
//  LQR
// ────────────────────────────────────────────────────────────
static constexpr int LQR_STATE_SIZE   = 2;
static constexpr int LQR_CONTROL_SIZE = 1;
 
AutoLQRStatic<LQR_STATE_SIZE, LQR_CONTROL_SIZE> lqr;
 
float lqrState[2]   = {0.0f, 0.0f};
float lqrControl[1] = {0.0f};
bool  lqrReady      = false;
float controlOutput = 0.0f;
 
float A[4] = {1.0f, 0.01f, 0.0f, MODEL_DAMPING};
float B[2] = {0.00005f, 0.01f};
float Q[4] = {Q_ANGLE, 0.0f, 0.0f, Q_RATE};
float R[1] = {R_CONTROL};
 
// ────────────────────────────────────────────────────────────
//  DSHOT
// ────────────────────────────────────────────────────────────
DShotRMT motor1(MOTOR1_GPIO, DSHOT600, false);
DShotRMT motor2(MOTOR2_GPIO, DSHOT600, false);
 
uint16_t motor1Raw = 0;
uint16_t motor2Raw = 0;
 
// ────────────────────────────────────────────────────────────
//  IMU hulpfuncties
// ────────────────────────────────────────────────────────────
void setupMPU(int address) {
  Wire.beginTransmission(address);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}
 
void readRawData(int address,
    int16_t &acX, int16_t &acY, int16_t &acZ,
    int16_t &gyX, int16_t &gyY, int16_t &gyZ) {
  Wire.beginTransmission(address);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(address, 14, true);
  acX = Wire.read() << 8 | Wire.read();
  acY = Wire.read() << 8 | Wire.read();
  acZ = Wire.read() << 8 | Wire.read();
  int16_t tmp = Wire.read() << 8 | Wire.read(); // temperatuur, niet gebruikt
  (void)tmp;
  gyX = Wire.read() << 8 | Wire.read();
  gyY = Wire.read() << 8 | Wire.read();
  gyZ = Wire.read() << 8 | Wire.read();
}
 
// ────────────────────────────────────────────────────────────
//  Motor helper
// ────────────────────────────────────────────────────────────
uint16_t throttleFromValue(float value, float minVal) {
  if (value < minVal)             value = minVal;
  if (value > DSHOT_THROTTLE_MAX) value = DSHOT_THROTTLE_MAX;
  return (uint16_t)value;
}
 
// ────────────────────────────────────────────────────────────
//  LQR setup
// ────────────────────────────────────────────────────────────
void buildLqrMatrices() {
  const float dt = IMU_INTERVAL_MS / 1000.0f;
 
  A[0] = 1.0f;
  A[1] = dt;
  A[2] = 0.0f;
  A[3] = constrain((float)MODEL_DAMPING, 0.0f, 1.0f);
 
  B[0] = 0.5f * dt * dt * MODEL_INPUT_SCALE;
  B[1] = dt * MODEL_INPUT_SCALE;
 
  Q[0] = max((float)Q_ANGLE,   0.0f);
  Q[1] = 0.0f;
  Q[2] = 0.0f;
  Q[3] = max((float)Q_RATE,    0.0f);
 
  R[0] = max((float)R_CONTROL, 0.0001f);
}
 
bool rebuildLqr() {
  buildLqrMatrices();
 
  bool ok = true;
  ok = ok && lqr.setStateMatrix(A);
  ok = ok && lqr.setInputMatrix(B);
  ok = ok && lqr.setCostMatrices(Q, R);
  ok = ok && lqr.computeGains();
 
  lqrReady = ok;
  return lqrReady;
}
 
void computeLQR(float currentAngle, float currentRate) {
  lqrState[0] = TARGET_ANGLE - currentAngle;
  lqrState[1] = 0.0f        - currentRate;
 
  lqr.updateState(lqrState);
  lqr.calculateControl(lqrControl);
 
  controlOutput = lqrControl[0] * CONTROL_SCALE;
}
 
// ────────────────────────────────────────────────────────────
//  Setup
// ────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1500);
 
  // ── WiFi in station-modus voor ESP-NOW ───────────────────
  WiFi.mode(WIFI_STA);
 
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init mislukt!");
  } else {
    esp_now_register_recv_cb(esp_now_recv_cb_t(onDataRecv));
    Serial.println("ESP-NOW luistert...");
  }
 
  // ── IMU ──────────────────────────────────────────────────
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  setupMPU(MPU1);
  delay(100);
 
  // ── Motoren ──────────────────────────────────────────────
  motor1.begin();
  motor2.begin();
 
  // ESC arm-puls: 500 ms nul throttle
  unsigned long armStart = millis();
  while (millis() - armStart < 500) {
    motor1.sendThrottle(0);
    motor2.sendThrottle(0);
    delay(1);
  }
 
  // ── LQR ──────────────────────────────────────────────────
  rebuildLqr();
  Serial.println(lqrReady ? "LQR OK" : "LQR MISLUKT");
 
  lastTime = millis();
}
 
// ────────────────────────────────────────────────────────────
//  Loop
// ────────────────────────────────────────────────────────────
void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastTime < IMU_INTERVAL_MS) return;
 
  float dt = (currentMillis - lastTime) / 1000.0f;
  lastTime = currentMillis;
 
  // ── IMU lezen ──────────────────────────────────────────────
  int16_t acX1, acY1, acZ1, gyX1, gyY1, gyZ1;
  readRawData(MPU1, acX1, acY1, acZ1, gyX1, gyY1, gyZ1);
 
  float avgAcX = -(float)acX1;
  float avgAcY =  (float)acY1;
  float avgAcZ = -(float)acZ1;
  float avgGyX =  (float)gyX1;
  float avgGyY = -(float)gyY1;
 
  float accPitch  = (atan2(-avgAcX, sqrt(pow(avgAcY, 2) + pow(avgAcZ, 2))) * 180.0) / PI;
  float accRoll   = (atan2(avgAcY, avgAcZ) * 180.0) / PI;
  float gyroXrate = avgGyX / 131.0f;
  float gyroYrate = avgGyY / 131.0f;
 
  if (firstLoop) {
    absolutePitchRaw = accPitch;
    absoluteRollRaw  = accRoll;
    angleRateFilt    = gyroYrate;
    firstLoop        = false;
  } else {
    float accMagG = sqrt(pow(avgAcX, 2) + pow(avgAcY, 2) + pow(avgAcZ, 2)) / 16384.0f;
    float alpha   = (fabs(accMagG - 1.0f) < 0.15f) ? 0.02f : 0.0f;
    absolutePitchRaw = (1.0f - alpha) * (absolutePitchRaw + gyroYrate * dt) + alpha * accPitch;
    absoluteRollRaw  = (1.0f - alpha) * (absoluteRollRaw  + gyroXrate * dt) + alpha * accRoll;
    angleRateFilt    = RATE_FILTER_ALPHA * gyroYrate + (1.0f - RATE_FILTER_ALPHA) * angleRateFilt;
  }
 
  pitch = absolutePitchRaw - BALANCE_OFFSET;
  roll  = absoluteRollRaw  - ROLL_OFFSET;
 
  // ── LQR berekenen (alleen als balanceActive) ────────────────
  if (balanceActive && lqrReady) {
    computeLQR(pitch, angleRateFilt);
  } else {
    controlOutput = 0.0f;
    lqrControl[0] = 0.0f;
  }
 
  // ── Motor sturing ──────────────────────────────────────────
  motor1Raw = 0;
  motor2Raw = 0;
 
  if (balanceActive && lqrReady) {
    float maxDelta = (float)DSHOT_THROTTLE_MAX - MOTOR_BASE;
    float absOut   = constrain(fabs(controlOutput), 0.0f, maxDelta);
 
    if (controlOutput >= 0.0f) {
      motor1Raw = throttleFromValue(MOTOR_BASE + absOut + MOTOR_TRIM, MOTOR_BASE);
      motor2Raw = throttleFromValue(MOTOR_BASE           - MOTOR_TRIM, MOTOR_BASE);
    } else {
      motor1Raw = throttleFromValue(MOTOR_BASE           + MOTOR_TRIM, MOTOR_BASE);
      motor2Raw = throttleFromValue(MOTOR_BASE + absOut  - MOTOR_TRIM, MOTOR_BASE);
    }
  }
 
  // ALTIJD throttle sturen zodat ESCs armed blijven
  motor1.sendThrottle(motor1Raw);
  motor2.sendThrottle(motor2Raw);
}