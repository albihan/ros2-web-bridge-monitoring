#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define NODE_ID 1
#define LDR_PIN 34
#define GW_CHANNEL 6

uint8_t GATEWAY_MAC[] = {0x08,0xA6,0xF7,0x64,0xA4,0xE4};

typedef struct {
  uint8_t  node_id;
  uint8_t  sensor_type;   // 3=LDR,9=HELLO,10=TIME_SYNC
  float    value;
  uint32_t timestamp;
} NodePacket;

bool time_ready = false;
uint32_t base_epoch = 0;
unsigned long last_send = 0, last_req = 0;

uint32_t nowEpoch() { return base_epoch + millis()/1000; }

void onReceive(const esp_now_recv_info_t*, const uint8_t* data, int len) {
  if (len != sizeof(NodePacket)) return;
  NodePacket pkt; memcpy(&pkt, data, sizeof(pkt));
  if (pkt.sensor_type == 10) {
    base_epoch = pkt.timestamp - millis()/1000;
    time_ready = true;
    Serial.printf("[NODE 1] TIME | SYNC=OK | BASE=%lu | CH=%d\n", base_epoch, GW_CHANNEL);
  }
}

void sendHello() {
  NodePacket p{NODE_ID,9,0,0};
  esp_now_send(GATEWAY_MAC, (uint8_t*)&p, sizeof(p));
  Serial.printf("[NODE 1] TIME | REQUEST | CH=%d\n", GW_CHANNEL);
}

void sendLDR() {
  int raw = analogRead(LDR_PIN);
  NodePacket p{NODE_ID,3,(float)raw, nowEpoch()};
  if (esp_now_send(GATEWAY_MAC,(uint8_t*)&p,sizeof(p))==ESP_OK)
    Serial.printf("[NODE 1] SEND LDR | %d | TS=%lu | OK\n", raw, p.timestamp);
}

void setup() {
  Serial.begin(115200); delay(1000);
  Serial.println("[NODE 1] BOOT");

  WiFi.mode(WIFI_STA); WiFi.disconnect(); delay(100);
  esp_wifi_set_channel(GW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init()!=ESP_OK) return;
  esp_now_register_recv_cb(onReceive);

  esp_now_peer_info_t peer={};
  memcpy(peer.peer_addr,GATEWAY_MAC,6);
  peer.channel=GW_CHANNEL; peer.encrypt=false;
  esp_now_add_peer(&peer);

  pinMode(LDR_PIN, INPUT);
  Serial.printf("[NODE 1] ESPNOW | INIT OK | CH=%d\n", GW_CHANNEL);
  sendHello();
}

void loop() {
  if (!time_ready) {
    if (millis()-last_req>2000){ last_req=millis(); sendHello(); }
    return;
  }
  if (millis()-last_send>1500){ last_send=millis(); sendLDR(); }
}
