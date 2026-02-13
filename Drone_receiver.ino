
// Front Right (CW) = 18
// Front Left  (CCW)= 19
// Rear Left   (CW) = 25
// Rear Right  (CCW)= 26

#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>
#include <ESP32Servo.h>
#include <math.h>

#define ESPNOW_WIFI_CHANNEL 6

#define PIN_FR 18
#define PIN_FL 19
#define PIN_RL 25
#define PIN_RR 26

#define ESC_MIN 1000
#define ESC_SAFE_MAX 2000 
#define ESC_FULL 2000

#define CONTROL_US_DELAY 500

HardwareSerial NanoSerial(1);
String mpuRawData = "";
float ax=0, ay=0, az=0, gx=0, gy=0, gz=0;

float pitch = 0.0f, roll = 0.0f;
unsigned long lastTime = 0;
const float alpha = 0.96f;

volatile int latestThrottleRaw = 0; 
volatile int latestYawRaw = 2048;
volatile int latestPitchRaw = 2048;
volatile int latestRollRaw = 2048;
volatile int latestArmCmd = 1; 
unsigned long lastRcMillis = 0;

Servo escFR, escFL, escRL, escRR;

struct PID { float Kp, Ki, Kd, integrator, lastError, intLimit; };
PID rollRatePID  = {0.12f, 0.02f, 0.002f, 0.0f, 0.0f, 200.0f};
PID pitchRatePID = {0.12f, 0.02f, 0.002f, 0.0f, 0.0f, 200.0f};
PID yawRatePID   = {0.08f, 0.01f, 0.001f, 0.0f, 0.0f, 200.0f};

typedef struct {
  int throttle_raw;
  int yaw_raw;
  int pitch_raw;
  int roll_raw;
  int arm;
} RC_Packet;

class RC_Peer : public ESP_NOW_Peer {
public:
  RC_Peer(const uint8_t *mac_addr, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk)
    : ESP_NOW_Peer(mac_addr, channel, iface, lmk) {}
  bool addPeer() { return ESP_NOW_Peer::add(); }

  void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    if (len == sizeof(RC_Packet)) {
      RC_Packet pkt;
      memcpy(&pkt, data, sizeof(pkt));
     
      latestThrottleRaw = pkt.throttle_raw;
      latestYawRaw = pkt.yaw_raw;
      latestPitchRaw = pkt.pitch_raw;
      latestRollRaw = pkt.roll_raw;
      latestArmCmd = pkt.arm;
      lastRcMillis = millis();
      Serial.printf("[RC RX] Th:%4d Yaw:%4d Pit:%4d Rol:%4d ARM:%d\n",
                    latestThrottleRaw, latestYawRaw, latestPitchRaw, latestRollRaw, latestArmCmd);
    }
  }
};

std::vector<ESP_NOW_Peer *> masters;
void register_new_master(const esp_now_recv_info_t *info, const uint8_t *data, int len, void *arg) {
  if (memcmp(info->des_addr, ESP_NOW.BROADCAST_ADDR, 6) == 0) {
    RC_Peer *p = new RC_Peer(info->src_addr, ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, nullptr);
    if (p->addPeer()) masters.push_back(p);
    Serial.printf("[RC] New master: " MACSTR " registered\n", MAC2STR(info->src_addr));
  }
}

void parseMPUData(String line) {
  int vals[6]; int idx=0; int last=-1;
  for (int i=0;i<line.length();++i) {
    if (line[i]==',' || i==line.length()-1) {
      int endIdx = (i==line.length()-1) ? i+1 : i;
      vals[idx++] = line.substring(last+1,endIdx).toInt();
      last = i;
      if (idx>=6) break;
    }
  }
  ax = vals[0]; ay = vals[1]; az = vals[2];
  gx = vals[3]; gy = vals[4]; gz = vals[5];
}

void updateAngleFromMPU(float dt) {
  float gyroXrate = gx / 131.0f;
  float gyroYrate = gy / 131.0f;
  float pitchAcc = atan2(ay, sqrt(ax*ax + az*az)) * 180.0f / M_PI;
  float rollAcc  = atan2(-ax, az) * 180.0f / M_PI;
  pitch = alpha * (pitch + gyroXrate * dt) + (1.0f - alpha) * pitchAcc;
  roll  = alpha * (roll  + gyroYrate * dt) + (1.0f - alpha) * rollAcc;
}


int throttleCurve_us(int raw) {
  float t = constrain((float)raw / 4095.0f, 0.0f, 1.0f);
  int us = ESC_MIN + (int)(t * (ESC_FULL - ESC_MIN));
  return us;
}

float pid_compute(PID &pid, float error, float dt) {
  pid.integrator += error * dt;
  if (pid.integrator > pid.intLimit) pid.integrator = pid.intLimit;
  if (pid.integrator < -pid.intLimit) pid.integrator = -pid.intLimit;
  float derivative = 0.0f;
  if (dt > 0.0f) derivative = (error - pid.lastError) / dt;
  pid.lastError = error;
  return pid.Kp * error + pid.Ki * pid.integrator + pid.Kd * derivative;
}

void calibrateESCs() {
  Serial.println("ESC calibration: full then min");
  escFR.writeMicroseconds(ESC_FULL);
  escFL.writeMicroseconds(ESC_FULL);
  escRL.writeMicroseconds(ESC_FULL);
  escRR.writeMicroseconds(ESC_FULL);
  delay(2000);
  escFR.writeMicroseconds(ESC_MIN);
  escFL.writeMicroseconds(ESC_MIN);
  escRL.writeMicroseconds(ESC_MIN);
  escRR.writeMicroseconds(ESC_MIN);
  delay(2000);
  Serial.println("ESC calibration complete");
}

unsigned long lastControlMicros = 0;
unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);
  NanoSerial.begin(115200, SERIAL_8N1, 16, 17);
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);

  escFR.attach(PIN_FR);
  escFL.attach(PIN_FL);
  escRL.attach(PIN_RL);
  escRR.attach(PIN_RR);

  escFR.writeMicroseconds(ESC_MIN);
  escFL.writeMicroseconds(ESC_MIN);
  escRL.writeMicroseconds(ESC_MIN);
  escRR.writeMicroseconds(ESC_MIN);

  if (!ESP_NOW.begin()) {
    Serial.println("ESP-NOW init failed");
    delay(2000);
    ESP.restart();
  }
  ESP_NOW.onNewPeer(register_new_master, nullptr);

  lastControlMicros = micros();
  lastTime = millis();

  Serial.println("Receiver ready. Test WITHOUT props.");
}

inline int clampMotor(int v) { return constrain(v, ESC_MIN, ESC_SAFE_MAX); }

void loop() {
 
  while (NanoSerial.available()) {
    char c = NanoSerial.read();
    if (c == '\n') {
      parseMPUData(mpuRawData);
      mpuRawData = "";
      unsigned long nowMs = millis();
      float dt = (nowMs - lastTime) / 1000.0f;
      if (dt <= 0) dt = 0.001f;
      lastTime = nowMs;
      updateAngleFromMPU(dt);
    } else mpuRawData += c;
  }

  if (millis() - lastRcMillis > 1000) {
    escFR.writeMicroseconds(ESC_MIN);
    escFL.writeMicroseconds(ESC_MIN);
    escRL.writeMicroseconds(ESC_MIN);
    escRR.writeMicroseconds(ESC_MIN);
    if (millis() - lastPrint > 1000) {
      Serial.println("[FAILSAFE] No RC - motors idle");
      lastPrint = millis();
    }
    delay(20);
    return;
  }

  unsigned long nowMicros = micros();
  unsigned long dtMicros = nowMicros - lastControlMicros;
  if (dtMicros < CONTROL_US_DELAY) {
    delayMicroseconds(CONTROL_US_DELAY - dtMicros);
    nowMicros = micros();
    dtMicros = nowMicros - lastControlMicros;
  }
  lastControlMicros = nowMicros;
  float dt = dtMicros / 1000000.0f;

  int rawThrottle = latestThrottleRaw;
  int rawYaw     = latestYawRaw;
  int rawPitch   = latestPitchRaw;
  int rawRoll    = latestRollRaw;
  bool armed     = (latestArmCmd == 0); 

  if (!armed) {
    escFR.writeMicroseconds(ESC_MIN);
    escFL.writeMicroseconds(ESC_MIN);
    escRL.writeMicroseconds(ESC_MIN);
    escRR.writeMicroseconds(ESC_MIN);
    if (millis() - lastPrint > 500) {
      Serial.println("[ARM] DISARMED - motors idle");
      lastPrint = millis();
    }
    delay(2);
    return;
  }

 
  int throttle_us = throttleCurve_us(rawThrottle);
  int baseThrottle = constrain(throttle_us, ESC_MIN, ESC_SAFE_MAX);

  
  const float maxAngleDeg = 20.0f;
  const float angleToRate = 4.0f;
  float desPitch = ((float)rawPitch - 2048.0f) / 2048.0f * maxAngleDeg;
  float desRoll  = ((float)rawRoll  - 2048.0f) / 2048.0f * maxAngleDeg;

  
  float pitchError = desPitch - pitch;
  float rollError  = desRoll  - roll;
  float desiredPitchRate = angleToRate * pitchError;
  float desiredRollRate  = angleToRate * rollError;

  float gyroPitchRate = gx / 131.0f;
  float gyroRollRate  = gy / 131.0f;
  float gyroYawRate   = gz / 131.0f;

  // inner loop PID (rate)
  float pitchCorr_us = pid_compute(pitchRatePID, desiredPitchRate - gyroPitchRate, dt);
  float rollCorr_us  = pid_compute(rollRatePID,  desiredRollRate  - gyroRollRate,  dt);
  float yawStick     = (float)map(rawYaw,0,4095,-500,500); 
  float yawCorr_us   = pid_compute(yawRatePID, yawStick - gyroYawRate, dt);

  int mFL = (int)(baseThrottle - pitchCorr_us + rollCorr_us - yawCorr_us);
  int mFR = (int)(baseThrottle - pitchCorr_us - rollCorr_us + yawCorr_us);
  int mRL = (int)(baseThrottle + pitchCorr_us + rollCorr_us + yawCorr_us);
  int mRR = (int)(baseThrottle + pitchCorr_us - rollCorr_us - yawCorr_us);

  mFL = clampMotor(mFL);
  mFR = clampMotor(mFR);
  mRL = clampMotor(mRL);
  mRR = clampMotor(mRR);

  escFR.writeMicroseconds(mFR);
  escFL.writeMicroseconds(mFL);
  escRL.writeMicroseconds(mRL);
  escRR.writeMicroseconds(mRR);

  if (micros() - lastPrint > 100000) { 
    Serial.printf("[PID] Base:%d | FR:%d FL:%d RL:%d RR:%d | Pitch:%.2f Roll:%.2f\n",
                  baseThrottle, mFR, mFL, mRL, mRR, pitch, roll);
    lastPrint = micros();
  }
}
