#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define WIFI_CHANNEL 6

// ================= DATA =================
typedef struct {
  uint8_t node_id;
  float suhu;
  float kelembapan;
  float ldr;
} sensor_packet_t;

sensor_packet_t rx;

// ================= CALLBACK =================
void onDataRecv(const esp_now_recv_info_t *info,
                const uint8_t *incomingData,
                int len) {

  if (len == sizeof(sensor_packet_t)) {
    memcpy(&rx, incomingData, sizeof(rx));

    char uart_msg[128];

    // ===== FORMAT UART (AMAN SEMUA SENSOR) =====
    snprintf(
      uart_msg,
      sizeof(uart_msg),
      "node:%d,suhu:%.2f,kelembapan:%.2f,ldr:%.2f",
      rx.node_id,
      rx.suhu,
      rx.kelembapan,
      rx.ldr
    );

    Serial.println(uart_msg);
    Serial2.println(uart_msg);
  }
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, -1, 17);
  delay(1000);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW INIT FAILED");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);

  Serial.println("=== GATEWAY A READY ===");
}

// ================= LOOP =================
void loop() {
  delay(5000);
}
