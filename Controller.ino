#include "ESP32_NOW.h"
#include "WiFi.h"

#define THROTTLE_PIN 34
#define YAW_PIN      35
#define PITCH_PIN    32
#define ROLL_PIN     33
#define ARM_BTN      26   // Digital arm/disarm button

#define ESPNOW_WIFI_CHANNEL 6

typedef struct {
  int throttle_raw;
  int yaw_raw;
  int pitch_raw;
  int roll_raw;
  int arm;
} RC_Packet;

RC_Packet rcPacket;

// Broadcast peer object using ESP32_NOW library
class ESP_NOW_Broadcast_Peer : public ESP_NOW_Peer {
public:
  ESP_NOW_Broadcast_Peer(uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) :
    ESP_NOW_Peer(ESP_NOW.BROADCAST_ADDR, channel, iface, lmk) {}
  ~ESP_NOW_Broadcast_Peer() { remove(); }
  bool begin() { return ESP_NOW.begin() && add(); }
  bool send_message(const uint8_t *data, size_t len) { return send(data, len); }
};

ESP_NOW_Broadcast_Peer broadcast_peer(ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, nullptr);

void setup() {
  Serial.begin(115200);
  pinMode(ARM_BTN, INPUT_PULLUP);
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);

  Serial.println("Broadcast RC transmitter starting...");

  if (!broadcast_peer.begin()) {
    Serial.println("Failed to initialize broadcast peer. Rebooting...");
    delay(3000);
    ESP.restart();
  }
}

void loop() {
  rcPacket.throttle_raw = analogRead(THROTTLE_PIN);
  rcPacket.yaw_raw      = analogRead(YAW_PIN);
  rcPacket.pitch_raw    = analogRead(PITCH_PIN);
  rcPacket.roll_raw     = analogRead(ROLL_PIN);
  rcPacket.arm          = digitalRead(ARM_BTN);

  // Send as broadcast
  if (!broadcast_peer.send_message((uint8_t *)&rcPacket, sizeof(rcPacket))) {
    Serial.println("Failed to broadcast RC packet!");
  } else {
    Serial.printf("Broadcast RC: Thrt:%4d Yaw:%4d Ptch:%4d Roll:%4d ARM:%d\n",
                  rcPacket.throttle_raw, rcPacket.yaw_raw, rcPacket.pitch_raw, rcPacket.roll_raw, rcPacket.arm);
  }
  delay(20); // ~50Hz control rate
}
