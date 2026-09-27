/*
  ESP-NOW VEHICLE NODE — MPU6500 + HC-SR04 + multi-node awareness
  -----------------------------------------------------------------------------
  Flash this on BOTH vehicle ESP32 boards. There are 3 boards total in this
  system:
      - Vehicle A  (this sketch, DEVICE_ID = "V1")
      - Vehicle B  (this sketch, DEVICE_ID = "V2")
      - Blind Node (blind_node.ino, DEVICE_ID = "BN1")

  Each vehicle:
    1) Reads its own MPU6500 accel/gyro + HC-SR04 distance, computes net
       accel, tilt (Danger Tilt/Normal), direction (LEFT/RIGHT/STRAIGHT/BACK)
       and forward-obstacle TTC/risk — exactly as before.
    2) Broadcasts that as a MSG_TELEMETRY packet, tagged with its own
       deviceType (VEHICLE) and deviceId ("V1"/"V2").
    3) Listens for broadcasts from everyone else and sorts them by sender:
         - MSG_TELEMETRY from the OTHER vehicle  -> "other vehicle" data
         - MSG_BEACON     from the blind node    -> "blind node detected,
                                                      approaching at X m"
                                                      (distance via RSSI)
         - MSG_RELAY_ALERT addressed to ME       -> the blind node has seen
                                                      BOTH vehicles at once
                                                      and is warning me that
                                                      the other vehicle is
                                                      coming from the far
                                                      side of the blind spot
    4) Prints everything to Serial, human-readable, AND as "DATA,..." lines
       the companion Python dashboard (vehicle_dashboard.py) parses.

  >>> CHANGE THIS on each vehicle board before flashing: <<<
*/
#define DEVICE_ID "V1"     // use "V2" on the second vehicle board

/*
  Wiring (MPU6500):
    MPU6500 VCC -> 3.3V   | MPU6500 GND -> GND
    MPU6500 SCL -> GPIO22 | MPU6500 SDA -> GPIO21
    MPU6500 AD0 -> GND (I2C addr 0x68; tie to 3.3V for 0x69 and update
                   MPU6500_I2C_ADDR below)

  Wiring (HC-SR04):
    VCC->5V/VIN, GND->GND, Trig->GPIO19, Echo->GPIO18 (through a voltage
    divider — Echo is 5V, ESP32 GPIOs are 3.3V max)

  Libraries required: NONE beyond the ESP32 core (MPU6500 is read directly
  over I2C via Wire — see the driver class below).
*/

#include <Wire.h>
#include <string.h>
#include <math.h>
#include <WiFi.h>
#include <esp_now.h>

// ---------------- CONFIGURATION ----------------
#define RSSI_AT_1M     -40.0     // calibrate: place boards 1m apart, read RSSI, update
#define PATH_LOSS_EXP   2.5      // 2.0 open space, 2.5-3.5 indoors
#define SEND_INTERVAL_MS  200

#define TRIG_PIN  19
#define ECHO_PIN  18
#define SOUND_CM_PER_US  0.01715
#define MAX_VALID_DISTANCE_CM  400.0

#define TTC_CRITICAL_SEC   3.0
#define TTC_WARNING_SEC    8.0
#define TTC_CAUTION_SEC   15.0

#define TILT_DANGER_DEG            25.0
#define GYRO_TURN_THRESH            0.35
#define ACC_FWD_THRESH              0.6
#define OBJECT_DETECT_THRESHOLD_CM  100.0
#define GRAVITY_MS2                 9.81

#define MPU6500_I2C_ADDR     0x68
#define REG_SMPLRT_DIV       0x19
#define REG_CONFIG           0x1A
#define REG_GYRO_CONFIG      0x1B
#define REG_ACCEL_CONFIG     0x1C
#define REG_ACCEL_CONFIG2    0x1D
#define REG_PWR_MGMT_1       0x6B
#define REG_WHO_AM_I         0x75
#define REG_ACCEL_XOUT_H     0x3B

// ---- Shared packet identity constants (MUST match blind_node.ino) ----
#define DEV_VEHICLE  0
#define DEV_BLIND    1

#define MSG_TELEMETRY    0
#define MSG_BEACON       1
#define MSG_RELAY_ALERT  2

// ------------------------------------

// ---------- Minimal MPU6500 driver (I2C via Wire, no external library) ----------
class MPU6500 {
  public:
    bool begin(uint8_t addr = MPU6500_I2C_ADDR) {
      _addr = addr;
      Wire.beginTransmission(_addr);
      if (Wire.endTransmission() != 0) return false;

      uint8_t who = readReg(REG_WHO_AM_I);
      if (who != 0x70 && who != 0x71 && who != 0x73) {
        Serial.print("MPU6500: unexpected WHO_AM_I 0x");
        Serial.println(who, HEX);
      }

      writeReg(REG_PWR_MGMT_1, 0x00);
      delay(50);
      writeReg(REG_GYRO_CONFIG, 0x08);    // +/-500 dps
      writeReg(REG_ACCEL_CONFIG, 0x10);   // +/-8 g
      writeReg(REG_CONFIG, 0x04);         // gyro DLPF ~20Hz
      writeReg(REG_ACCEL_CONFIG2, 0x04);  // accel DLPF ~21Hz
      writeReg(REG_SMPLRT_DIV, 0x00);

      _accelSensLsbPerG  = 4096.0;
      _gyroSensLsbPerDps = 65.5;
      return true;
    }

    void readAccelGyro(float &ax, float &ay, float &az,
                        float &gx, float &gy, float &gz) {
      uint8_t buf[14];
      readRegs(REG_ACCEL_XOUT_H, buf, 14);
      int16_t rawAx = (int16_t)((buf[0]  << 8) | buf[1]);
      int16_t rawAy = (int16_t)((buf[2]  << 8) | buf[3]);
      int16_t rawAz = (int16_t)((buf[4]  << 8) | buf[5]);
      int16_t rawGx = (int16_t)((buf[8]  << 8) | buf[9]);
      int16_t rawGy = (int16_t)((buf[10] << 8) | buf[11]);
      int16_t rawGz = (int16_t)((buf[12] << 8) | buf[13]);

      const float G_TO_MS2 = 9.80665;
      ax = (rawAx / _accelSensLsbPerG) * G_TO_MS2;
      ay = (rawAy / _accelSensLsbPerG) * G_TO_MS2;
      az = (rawAz / _accelSensLsbPerG) * G_TO_MS2;

      const float DEG_TO_RAD_ = PI / 180.0;   // renamed to avoid clashing with Arduino.h's DEG_TO_RAD macro
      gx = (rawGx / _gyroSensLsbPerDps) * DEG_TO_RAD_;
      gy = (rawGy / _gyroSensLsbPerDps) * DEG_TO_RAD_;
      gz = (rawGz / _gyroSensLsbPerDps) * DEG_TO_RAD_;
    }

  private:
    uint8_t _addr = MPU6500_I2C_ADDR;
    float _accelSensLsbPerG  = 4096.0;
    float _gyroSensLsbPerDps = 65.5;

    void writeReg(uint8_t reg, uint8_t val) {
      Wire.beginTransmission(_addr);
      Wire.write(reg);
      Wire.write(val);
      Wire.endTransmission();
    }
    uint8_t readReg(uint8_t reg) {
      Wire.beginTransmission(_addr);
      Wire.write(reg);
      Wire.endTransmission(false);
      Wire.requestFrom((int)_addr, 1);
      return Wire.available() ? Wire.read() : 0;
    }
    void readRegs(uint8_t startReg, uint8_t *buf, uint8_t len) {
      Wire.beginTransmission(_addr);
      Wire.write(startReg);
      Wire.endTransmission(false);
      Wire.requestFrom((int)_addr, (int)len);
      for (uint8_t i = 0; i < len && Wire.available(); i++) buf[i] = Wire.read();
    }
};

MPU6500 mpu;

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ---- Shared packet format (IDENTICAL layout in blind_node.ino) ----
typedef struct struct_message {
  uint8_t  msgType;          // MSG_TELEMETRY / MSG_BEACON / MSG_RELAY_ALERT
  uint8_t  deviceType;       // DEV_VEHICLE / DEV_BLIND  (sender's type)
  char     deviceId[8];      // sender's id, e.g. "V1", "V2", "BN1"
  uint32_t counter;

  float accX, accY, accZ;
  float gyroX, gyroY, gyroZ;

  float distanceCm;
  float closingSpeedCmS;
  float ttcSec;
  char  riskLevel[10];

  float netAccel;
  char  direction[10];
  uint8_t dangerTilt;
  float rollDeg;
  float pitchDeg;
  uint8_t objectDetected;

  // Only meaningful when msgType == MSG_RELAY_ALERT (sent by the blind node)
  char  targetId[8];         // which vehicle this alert is FOR
  char  otherVehicleId[8];   // the OTHER vehicle, on the far side
  float otherVehicleDistM;   // that other vehicle's distance from the blind node
} struct_message;

struct_message outgoingMsg;
unsigned long lastSend = 0;
uint32_t sendCounter = 0;

float previousDistanceCm = -1.0;
unsigned long previousDistanceTime = 0;

// ---------- HC-SR04 ----------
float readUltrasonicCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);
  if (duration == 0) return -1.0;
  float distance = duration * SOUND_CM_PER_US;
  if (distance > MAX_VALID_DISTANCE_CM || distance <= 0) return -1.0;
  return distance;
}

void classifyRisk(float ttcSec, char *outBuf) {
  if (ttcSec < 0) strcpy(outBuf, "SAFE");
  else if (ttcSec < TTC_CRITICAL_SEC) strcpy(outBuf, "CRITICAL");
  else if (ttcSec < TTC_WARNING_SEC) strcpy(outBuf, "WARNING");
  else if (ttcSec < TTC_CAUTION_SEC) strcpy(outBuf, "CAUTION");
  else strcpy(outBuf, "SAFE");
}

void classifyDirection(float accY, float gyroZ, char *outBuf) {
  if (gyroZ > GYRO_TURN_THRESH) strcpy(outBuf, "RIGHT");
  else if (gyroZ < -GYRO_TURN_THRESH) strcpy(outBuf, "LEFT");
  else if (accY > ACC_FWD_THRESH) strcpy(outBuf, "STRAIGHT");
  else if (accY < -ACC_FWD_THRESH) strcpy(outBuf, "BACK");
  else strcpy(outBuf, "STRAIGHT");
}

// Returns meters (calibrated so result == 1.0 at RSSI_AT_1M)
float rssiToDistance(int rssi) {
  return pow(10.0, (RSSI_AT_1M - rssi) / (10.0 * PATH_LOSS_EXP));
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {}
#else
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {}
#endif

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  struct_message incomingMsg;
  memcpy(&incomingMsg, incomingData, sizeof(incomingMsg));

  int rssi = info->rx_ctrl->rssi;
  float rssiDistM = rssiToDistance(rssi);

  if (incomingMsg.msgType == MSG_TELEMETRY && incomingMsg.deviceType == DEV_VEHICLE) {
    if (strcmp(incomingMsg.deviceId, DEVICE_ID) == 0) return; // ignore any echo of self

    Serial.println("---- OTHER VEHICLE ----");
    Serial.printf("  ID: %s | Pkt #%u\n", incomingMsg.deviceId, incomingMsg.counter);
    Serial.printf("  Net accel: %.2f m/s^2 | Direction: %s | %s\n",
                  incomingMsg.netAccel, incomingMsg.direction,
                  incomingMsg.dangerTilt ? "DANGER TILT" : "NORMAL");
    Serial.printf("  Forward obstacle: %.1f cm | Risk: %s\n",
                  incomingMsg.distanceCm, incomingMsg.riskLevel);
    Serial.printf("  RSSI: %d dBm | Estimated distance to this vehicle: %.2f m\n", rssi, rssiDistM);
    Serial.println();

    Serial.printf("DATA,REMOTE_VEHICLE,%s,%lu,%.2f,%s,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%s,%d,%.2f\n",
                  incomingMsg.deviceId, incomingMsg.counter, incomingMsg.netAccel,
                  incomingMsg.direction, incomingMsg.dangerTilt, incomingMsg.rollDeg,
                  incomingMsg.pitchDeg, incomingMsg.distanceCm, incomingMsg.closingSpeedCmS,
                  incomingMsg.ttcSec, incomingMsg.objectDetected, incomingMsg.riskLevel,
                  rssi, rssiDistM);
  }
  else if (incomingMsg.msgType == MSG_BEACON && incomingMsg.deviceType == DEV_BLIND) {
    Serial.printf("Blind node %s is approaching at ~%.2f m (RSSI %d dBm)\n",
                  incomingMsg.deviceId, rssiDistM, rssi);
    Serial.printf("DATA,BLIND,%s,%.2f,%d\n", incomingMsg.deviceId, rssiDistM, rssi);
  }
  else if (incomingMsg.msgType == MSG_RELAY_ALERT) {
    if (strcmp(incomingMsg.targetId, DEVICE_ID) != 0) return; // not addressed to me
    Serial.printf("*** BLIND-SPOT ALERT: Vehicle %s is approaching from the other side "
                  "of the blind node (~%.2f m from the blind node) ***\n",
                  incomingMsg.otherVehicleId, incomingMsg.otherVehicleDistM);
    Serial.printf("DATA,RELAY,%s,%.2f\n", incomingMsg.otherVehicleId, incomingMsg.otherVehicleDistM);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(21, 22);
  Wire.setClock(400000);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6500 — check wiring and I2C address (0x68/0x69)!");
    while (1) { delay(10); }
  }
  Serial.println("MPU6500 initialized.");

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  WiFi.mode(WIFI_STA);
  Serial.print("This board's MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Device ID: ");
  Serial.println(DEVICE_ID);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add broadcast peer");
    return;
  }

  outgoingMsg.msgType = MSG_TELEMETRY;
  outgoingMsg.deviceType = DEV_VEHICLE;
  memset(outgoingMsg.deviceId, 0, sizeof(outgoingMsg.deviceId));
  strncpy(outgoingMsg.deviceId, DEVICE_ID, sizeof(outgoingMsg.deviceId) - 1);
}

void loop() {
  unsigned long now = millis();
  if (now - lastSend >= SEND_INTERVAL_MS) {
    lastSend = now;

    float ax, ay, az, gx, gy, gz;
    mpu.readAccelGyro(ax, ay, az, gx, gy, gz);

    sendCounter++;
    outgoingMsg.counter = sendCounter;
    outgoingMsg.accX = ax; outgoingMsg.accY = ay; outgoingMsg.accZ = az;
    outgoingMsg.gyroX = gx; outgoingMsg.gyroY = gy; outgoingMsg.gyroZ = gz;

    float accMag = sqrt(ax * ax + ay * ay + az * az);
    outgoingMsg.netAccel = accMag - GRAVITY_MS2;
    outgoingMsg.rollDeg  = atan2(ay, az) * 180.0 / PI;
    outgoingMsg.pitchDeg = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
    outgoingMsg.dangerTilt = (fabs(outgoingMsg.rollDeg) > TILT_DANGER_DEG ||
                              fabs(outgoingMsg.pitchDeg) > TILT_DANGER_DEG) ? 1 : 0;

    char dirBuf[10];
    classifyDirection(ay, gz, dirBuf);
    strcpy(outgoingMsg.direction, dirBuf);

    float distanceCm = readUltrasonicCm();
    float closingSpeedCmS = 0.0;
    float ttcSec = -1.0;
    if (distanceCm > 0) {
      if (previousDistanceCm > 0) {
        float elapsedSec = (now - previousDistanceTime) / 1000.0;
        if (elapsedSec > 0) closingSpeedCmS = (previousDistanceCm - distanceCm) / elapsedSec;
      }
      previousDistanceCm = distanceCm;
      previousDistanceTime = now;
      if (closingSpeedCmS > 0.5) ttcSec = distanceCm / closingSpeedCmS;
    }

    char risk[10];
    classifyRisk(ttcSec, risk);

    outgoingMsg.distanceCm = (distanceCm > 0) ? distanceCm : 0.0;
    outgoingMsg.closingSpeedCmS = closingSpeedCmS;
    outgoingMsg.ttcSec = ttcSec;
    strcpy(outgoingMsg.riskLevel, risk);
    outgoingMsg.objectDetected = (distanceCm > 0 && distanceCm <= OBJECT_DETECT_THRESHOLD_CM) ? 1 : 0;

    Serial.println("==== LOCAL (this vehicle) ====");
    Serial.printf("  ID: %s | Pkt #%u\n", DEVICE_ID, sendCounter);
    Serial.printf("  Net accel: %.2f m/s^2 | Direction: %s | %s\n",
                  outgoingMsg.netAccel, outgoingMsg.direction,
                  outgoingMsg.dangerTilt ? "DANGER TILT" : "NORMAL");
    if (distanceCm > 0) {
      Serial.printf("  Forward obstacle: %.1f cm | Object detected: %s | Risk: %s\n",
                    distanceCm, outgoingMsg.objectDetected ? "YES" : "no", risk);
    } else {
      Serial.println("  HC-SR04: no valid reading");
    }
    Serial.println();

    Serial.printf("DATA,LOCAL,%s,%lu,%.2f,%s,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%s,%d,%.2f\n",
                  DEVICE_ID, sendCounter, outgoingMsg.netAccel, outgoingMsg.direction,
                  outgoingMsg.dangerTilt, outgoingMsg.rollDeg, outgoingMsg.pitchDeg,
                  outgoingMsg.distanceCm, outgoingMsg.closingSpeedCmS, outgoingMsg.ttcSec,
                  outgoingMsg.objectDetected, outgoingMsg.riskLevel, 0, 0.0);

    esp_now_send(broadcastAddress, (uint8_t *)&outgoingMsg, sizeof(outgoingMsg));
  }
}
