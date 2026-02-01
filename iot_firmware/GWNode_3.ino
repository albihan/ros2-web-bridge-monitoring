#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define NODE_ID 3
#define LDR_PIN 34
#define GW_CHANNEL 10

// MAC Gateway
uint8_t GATEWAY_MAC[] = {0x08,0xA6,0xF7,0x64,0xA4,0xE4};

typedef struct {
  uint8_t  node_id;
  uint8_t  sensor_type;   // 1=SUHU,2=KELEMBAPAN,3=LDR,9=HELLO,10=TIME_SYNC
  float    value;
  uint32_t timestamp;
} NodePacket;

bool time_ready = false;
uint32_t base_epoch = 0;
unsigned long last_send = 0, last_req = 0;

uint32_t nowEpoch() {
  return base_epoch + millis() / 1000;
}

void onReceive(const esp_now_recv_info_t*, const uint8_t* data, int len) {
  if (len != sizeof(NodePacket)) return;

  NodePacket p;
  memcpy(&p, data, sizeof(p));

  if (p.sensor_type == 10) { // TIME_SYNC
    base_epoch = p.timestamp - millis() / 1000;
    time_ready = true;
    Serial.printf("[NODE 3] TIME | SYNC=OK | BASE=%lu | CH=%d\n", base_epoch, GW_CHANNEL);
  }
}

void sendHello() {
  NodePacket p{NODE_ID, 9, 0, 0};
  esp_now_send(GATEWAY_MAC, (uint8_t*)&p, sizeof(p));
  Serial.printf("[NODE 3] TIME | REQUEST | CH=%d\n", GW_CHANNEL);
}

void sendLDR() {
  int raw = analogRead(LDR_PIN);
  uint32_t ts = nowEpoch();

  NodePacket p{NODE_ID, 3, (float)raw, ts};
  esp_now_send(GATEWAY_MAC, (uint8_t*)&p, sizeof(p));

  Serial.printf("[NODE 3] SEND LDR | %d | TS=%lu | OK\n", raw, ts);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("[NODE 3] BOOT");

  pinMode(LDR_PIN, INPUT);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  esp_wifi_set_channel(GW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) return;
  esp_now_register_recv_cb(onReceive);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, GATEWAY_MAC, 6);
  peer.channel = GW_CHANNEL;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  Serial.printf("[NODE 3] ESPNOW | INIT OK | CH=%d\n", GW_CHANNEL);
  sendHello();
}

void loop() {
  if (!time_ready) {
    if (millis() - last_req > 2000) {
      last_req = millis();
      sendHello();
    }
    return;
  }

  if (millis() - last_send > 2000) {
    last_send = millis();
    sendLDR();
  }
}
