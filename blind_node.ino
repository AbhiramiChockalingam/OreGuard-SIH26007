/*
  ESP-NOW BLIND NODE — stationary relay for a blind curve/junction
  -----------------------------------------------------------------------------
  Flash this on the third ESP32, placed at the blind spot itself. It has NO
  sensors (no MPU, no HC-SR04) — its only job is:
    1) Broadcast a small "I'm here" beacon every BEACON_INTERVAL_MS, tagged
       with deviceType = DEV_BLIND and DEVICE_ID = "BN1". Vehicles hear this
       and estimate their distance to the blind spot via RSSI.
    2) Listen for MSG_TELEMETRY broadcasts from vehicles (deviceType =
       DEV_VEHICLE), and track each one by its deviceId ("V1"/"V2"...) with
       an RSSI-estimated distance and a last-seen timestamp.
    3) A tracked vehicle is "detected" once it's within BLIND_DETECT_RANGE_M
       of the blind node, and is forgotten if not heard from for
       VEHICLE_TIMEOUT_MS.
    4) The moment TWO different vehicles are detected at the same time, the
       blind node sends a MSG_RELAY_ALERT to EACH of them — addressed by
       targetId — telling it the OTHER vehicle's id and its distance from
       the blind node. This is what lets two vehicles that can't "see" each
       other around the blind corner learn about each other.
    5) Everything is also printed as "DATA,..." lines for the companion
       blind_node_dashboard.py (a staff-facing explainer view of what the
       blind node is doing, not a driving dashboard).

  No wiring beyond power — this board just needs to be an ESP32 running this
  sketch, physically positioned at the blind spot / junction.
*/
#define DEVICE_ID "BN1"    // change if you ever deploy more than one blind node

#include <string.h>
#include <math.h>
#include <WiFi.h>
#include <esp_now.h>

// ---------------- CONFIGURATION ----------------
#define RSSI_AT_1M     -40.0     // must match the value used on the vehicle boards
#define PATH_LOSS_EXP   2.5

#define BEACON_INTERVAL_MS        300    // how often the blind node announces itself
#define BLIND_DETECT_RANGE_M       20.0  // vehicles farther than this aren't "at the junction" yet
#define VEHICLE_TIMEOUT_MS         3000  // forget a vehicle if not heard from this long
#define MAX_TRACKED_VEHICLES       4
#define RELAY_RESEND_INTERVAL_MS   400   // re-send the cross-alert this often while both are present
#define STATUS_PRINT_INTERVAL_MS   500   // how often to emit DATA,VEHICLE,... status lines

// ---- Shared packet identity constants (MUST match vehicle_node.ino) ----
#define DEV_VEHICLE  0
#define DEV_BLIND    1

#define MSG_TELEMETRY    0
#define MSG_BEACON       1
#define MSG_RELAY_ALERT  2

// ---- Shared packet format (IDENTICAL layout to vehicle_node.ino) ----
typedef struct struct_message {
  uint8_t  msgType;
  uint8_t  deviceType;
  char     deviceId[8];
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

  char  targetId[8];
  char  otherVehicleId[8];
  float otherVehicleDistM;
} struct_message;

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

struct VehicleEntry {
  bool used = false;
  char id[8] = {0};
  float distM = 0.0;
  unsigned long lastSeen = 0;
};
VehicleEntry vehicles[MAX_TRACKED_VEHICLES];

unsigned long lastBeacon = 0;
unsigned long lastRelay = 0;
unsigned long lastStatus = 0;
uint32_t beaconCounter = 0;

// Returns meters (calibrated so result == 1.0 at RSSI_AT_1M)
float rssiToDistance(int rssi) {
  return pow(10.0, (RSSI_AT_1M - rssi) / (10.0 * PATH_LOSS_EXP));
}

int findOrAddVehicle(const char *id) {
  int freeSlot = -1;
  for (int i = 0; i < MAX_TRACKED_VEHICLES; i++) {
    if (vehicles[i].used && strcmp(vehicles[i].id, id) == 0) return i;
    if (!vehicles[i].used && freeSlot < 0) freeSlot = i;
  }
  if (freeSlot >= 0) {
    vehicles[freeSlot].used = true;
    memset(vehicles[freeSlot].id, 0, sizeof(vehicles[freeSlot].id));
    strncpy(vehicles[freeSlot].id, id, sizeof(vehicles[freeSlot].id) - 1);
  }
  return freeSlot; // -1 if table is full
}

void sendRelay(const char *targetId, const char *otherId, float otherDistM) {
  struct_message alert = {};
  alert.msgType = MSG_RELAY_ALERT;
  alert.deviceType = DEV_BLIND;
  strncpy(alert.deviceId, DEVICE_ID, sizeof(alert.deviceId) - 1);
  strncpy(alert.targetId, targetId, sizeof(alert.targetId) - 1);
  strncpy(alert.otherVehicleId, otherId, sizeof(alert.otherVehicleId) - 1);
  alert.otherVehicleDistM = otherDistM;
  esp_now_send(broadcastAddress, (uint8_t *)&alert, sizeof(alert));
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {}
#else
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {}
#endif

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  struct_message incomingMsg;
  memcpy(&incomingMsg, incomingData, sizeof(incomingMsg));

  // The blind node only cares about vehicle telemetry broadcasts.
  if (incomingMsg.msgType != MSG_TELEMETRY || incomingMsg.deviceType != DEV_VEHICLE) return;

  int rssi = info->rx_ctrl->rssi;
  float distM = rssiToDistance(rssi);

  int idx = findOrAddVehicle(incomingMsg.deviceId);
  if (idx < 0) {
    Serial.println("Vehicle table full — increase MAX_TRACKED_VEHICLES");
    return;
  }
  vehicles[idx].distM = distM;
  vehicles[idx].lastSeen = millis();

  bool withinRange = distM <= BLIND_DETECT_RANGE_M;
  Serial.printf("Vehicle %s %s at %.2f m (RSSI %d dBm)\n",
                incomingMsg.deviceId,
                withinRange ? "DETECTED" : "seen (outside detection range)",
                distM, rssi);
}

void setup() {
  Serial.begin(115200);
  delay(500);

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

  Serial.println("Blind node ready.");
}

void loop() {
  unsigned long now = millis();

  // ---- 1) Periodic presence beacon ----
  if (now - lastBeacon >= BEACON_INTERVAL_MS) {
    lastBeacon = now;
    struct_message beacon = {};
    beacon.msgType = MSG_BEACON;
    beacon.deviceType = DEV_BLIND;
    strncpy(beacon.deviceId, DEVICE_ID, sizeof(beacon.deviceId) - 1);
    beacon.counter = ++beaconCounter;
    esp_now_send(broadcastAddress, (uint8_t *)&beacon, sizeof(beacon));
  }

  // ---- 2) Forget stale vehicles ----
  for (int i = 0; i < MAX_TRACKED_VEHICLES; i++) {
    if (vehicles[i].used && (now - vehicles[i].lastSeen) > VEHICLE_TIMEOUT_MS) {
      Serial.printf("Vehicle %s timed out — no longer tracked\n", vehicles[i].id);
      vehicles[i].used = false;
    }
  }

  // ---- 3) Collect currently-detected (in-range) vehicles ----
  int activeIdx[MAX_TRACKED_VEHICLES];
  int activeCount = 0;
  for (int i = 0; i < MAX_TRACKED_VEHICLES; i++) {
    if (vehicles[i].used && vehicles[i].distM <= BLIND_DETECT_RANGE_M) {
      activeIdx[activeCount++] = i;
    }
  }

  // ---- 4) If 2+ vehicles are detected at once, relay a cross-alert ----
  if (activeCount >= 2 && (now - lastRelay) >= RELAY_RESEND_INTERVAL_MS) {
    lastRelay = now;
    int a = activeIdx[0];
    int b = activeIdx[1];
    sendRelay(vehicles[a].id, vehicles[b].id, vehicles[b].distM); // tell A about B
    sendRelay(vehicles[b].id, vehicles[a].id, vehicles[a].distM); // tell B about A

    Serial.printf("*** BOTH SIDES OCCUPIED: %s @ %.2fm and %s @ %.2fm — relaying cross-alert ***\n",
                  vehicles[a].id, vehicles[a].distM, vehicles[b].id, vehicles[b].distM);
    Serial.printf("DATA,ALERT,%s,%s,%.2f,%.2f\n",
                  vehicles[a].id, vehicles[b].id, vehicles[a].distM, vehicles[b].distM);
  }

  // ---- 5) Periodic status dump for the blind-node dashboard ----
  if (now - lastStatus >= STATUS_PRINT_INTERVAL_MS) {
    lastStatus = now;
    for (int i = 0; i < MAX_TRACKED_VEHICLES; i++) {
      if (vehicles[i].used) {
        Serial.printf("DATA,VEHICLE,%s,%.2f,%lu\n",
                      vehicles[i].id, vehicles[i].distM, now - vehicles[i].lastSeen);
      }
    }
  }
}
