#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <DHT.h>

#define NODE_ID 4
#define DHTPIN 27
#define DHTTYPE DHT11
#define GW_CHANNEL 6

uint8_t GATEWAY_MAC[] = {0x08,0xA6,0xF7,0x64,0xA4,0xE4};
DHT dht(DHTPIN, DHTTYPE);

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

  if (p.sensor_type == 10) {
    base_epoch = p.timestamp - millis() / 1000;
    time_ready = true;
    Serial.printf("[NODE 4] TIME | SYNC=OK | BASE=%lu | CH=%d\n", base_epoch, GW_CHANNEL);
  }
}

void sendHello() {
  NodePacket p{NODE_ID, 9, 0, 0};
  esp_now_send(GATEWAY_MAC, (uint8_t*)&p, sizeof(p));
  Serial.printf("[NODE 4] TIME | REQUEST | CH=%d\n", GW_CHANNEL);
}

void sendDHT() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  uint32_t ts = nowEpoch();

  if (!isnan(t)) {
    NodePacket ps{NODE_ID, 1, t, ts};
    esp_now_send(GATEWAY_MAC, (uint8_t*)&ps, sizeof(ps));
    Serial.printf("[NODE 4] SEND SUHU | %.2f | TS=%lu | OK\n", t, ts);
  }

  if (!isnan(h)) {
    NodePacket ph{NODE_ID, 2, h, ts};
    esp_now_send(GATEWAY_MAC, (uint8_t*)&ph, sizeof(ph));
    Serial.printf("[NODE 4] SEND KELEMBAPAN | %.2f | TS=%lu | OK\n", h, ts);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("[NODE 4] BOOT");

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

  dht.begin();
  Serial.printf("[NODE 4] ESPNOW | INIT OK | CH=%d\n", GW_CHANNEL);
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
    sendDHT();
  }
}
