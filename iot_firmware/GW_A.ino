#include <WiFi.h>
#include <esp_now.h>
#include <time.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ================= WIFI & ROS =================
char WIFI_SSID[] = "1ab";
char WIFI_PASS[] = "gvyg6249";

char AGENT_IP[]  = "10.212.75.156";
const uint16_t AGENT_PORT = 8888;

// ================= LCD & BUZZER ===============
LiquidCrystal_I2C lcd(0x27, 16, 2);
#define BUZZER_PIN 25

// ================= ROS OBJECTS =================
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_publisher_t pub_ldr;
rcl_publisher_t pub_suhu;
rcl_publisher_t pub_kelembapan;

std_msgs__msg__String msg;
bool ros_ready = false;
bool ros_beeped = false;

// ================= PACKET =====================
typedef struct {
  uint8_t  node_id;
  uint8_t  sensor_type;   // 1=SUHU,2=HUM,3=LDR,9=HELLO,10=TIME_SYNC
  float    value;
  uint32_t timestamp;
} NodePacket;

// ================= STATE ======================
unsigned long boot_time = 0;
unsigned long last_lcd_switch = 0;
uint8_t page = 0;

struct NodeState {
  float t = NAN;
  float h = NAN;
  float ldr = NAN;
  unsigned long last_ms = 0;
};

NodeState n1, n2, n3, n4;

// ================= UTIL =======================
String macToString(const uint8_t *mac) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

void beep(int times, int on=120, int off=120) {
  for (int i=0;i<times;i++){
    digitalWrite(BUZZER_PIN, HIGH);
    delay(on);
    digitalWrite(BUZZER_PIN, LOW);
    delay(off);
  }
}

String epochToHuman(uint32_t ts) {
  time_t t = ts;
  struct tm *tm_info = localtime(&t);
  char buffer[32];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", tm_info);
  return String(buffer);
}

void ensurePeer(const uint8_t *mac) {
  if (!esp_now_is_peer_exist(mac)) {
    esp_now_peer_info_t p = {};
    memcpy(p.peer_addr, mac, 6);
    p.channel = WiFi.channel();
    p.encrypt = false;
    if (esp_now_add_peer(&p) == ESP_OK) {
      Serial.printf("[GW] PEER ADDED | %s\n", macToString(mac).c_str());
    }
  }
}

// ================= ESPNOW CALLBACK =================
void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(NodePacket)) {
    Serial.println("[GW] WARN | INVALID PACKET");
    return;
  }

  NodePacket pkt;
  memcpy(&pkt, data, sizeof(pkt));
  const uint8_t *mac = info->src_addr;

  // ---- HELLO ----
  if (pkt.sensor_type == 9) {
    Serial.printf("[GW] HELLO     | NODE=%d | MAC=%s\n",
                  pkt.node_id, macToString(mac).c_str());

    ensurePeer(mac);

    NodePacket reply{0, 10, 0, (uint32_t)time(NULL)};
    esp_now_send(mac, (uint8_t *)&reply, sizeof(reply));

    Serial.printf("[GW] TIME_SYNC| NODE=%d | SENT | %lu\n",
                  pkt.node_id, reply.timestamp);
    return;
  }

  // ---- UPDATE STATE ----
  NodeState *ns = nullptr;
  if (pkt.node_id==1) ns=&n1;
  if (pkt.node_id==2) ns=&n2;
  if (pkt.node_id==3) ns=&n3;
  if (pkt.node_id==4) ns=&n4;

  if (ns) {
    ns->last_ms = millis();
    if (pkt.sensor_type==1) ns->t = pkt.value;
    if (pkt.sensor_type==2) ns->h = pkt.value;
    if (pkt.sensor_type==3) ns->ldr = pkt.value;
  }

  String sensor = "UNK";
  if (pkt.sensor_type==1) sensor="SUHU";
  if (pkt.sensor_type==2) sensor="HUM";
  if (pkt.sensor_type==3) sensor="LDR";

  Serial.printf("[GW] RX %-4s | N%d | %.2f | %lu (%s)\n",
                sensor.c_str(), pkt.node_id, pkt.value,
                pkt.timestamp, epochToHuman(pkt.timestamp).c_str());

  if (!ros_ready) {
    Serial.printf("[GW] ROS PUB | /%s | SKIP (ROS OFFLINE)\n", sensor.c_str());
    return;
  }

  static char buffer[64];
  snprintf(buffer, sizeof(buffer), "%d,%.2f,%lu",
           pkt.node_id, pkt.value, pkt.timestamp);

  msg.data.data = buffer;
  msg.data.size = strlen(buffer);
  msg.data.capacity = sizeof(buffer);

  if (pkt.sensor_type == 3) rcl_publish(&pub_ldr, &msg, NULL);
  else if (pkt.sensor_type == 1) rcl_publish(&pub_suhu, &msg, NULL);
  else if (pkt.sensor_type == 2) rcl_publish(&pub_kelembapan, &msg, NULL);

  Serial.printf("[GW] ROS PUB | /%s | OK\n", sensor.c_str());
}

// ================= LCD PAGES =================
bool offline(const NodeState &n){ return (millis() - n.last_ms) > 10000 || n.last_ms==0; }

void drawStatus(){
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("GW ONLINE");
  lcd.setCursor(0,1);
  lcd.print("ROS:");
  lcd.print(ros_ready ? "OK " : "OFF");
  lcd.print(WiFi.status()==WL_CONNECTED ? " WIFI" : " NO-WI");
}

void drawN1(){
  lcd.clear(); lcd.print("N1 LDR");
  lcd.setCursor(0,1);
  if (offline(n1)) lcd.print("OFFLINE");
  else { lcd.print("VAL: "); lcd.print((int)n1.ldr); }
}
void drawN2(){
  lcd.clear(); lcd.print("N2 DHT22");
  lcd.setCursor(0,1);
  if (offline(n2)) lcd.print("OFFLINE");
  else { lcd.print("T:"); lcd.print(n2.t,1); lcd.print(" H:"); lcd.print((int)n2.h); lcd.print("%"); }
}
void drawN3(){
  lcd.clear(); lcd.print("N3 LDR");
  lcd.setCursor(0,1);
  if (offline(n3)) lcd.print("OFFLINE");
  else { lcd.print("VAL: "); lcd.print((int)n3.ldr); }
}
void drawN4(){
  lcd.clear(); lcd.print("N4 DHT11");
  lcd.setCursor(0,1);
  if (offline(n4)) lcd.print("OFFLINE");
  else { lcd.print("T:"); lcd.print(n4.t,1); lcd.print(" H:"); lcd.print((int)n4.h); lcd.print("%"); }
}

// ================= SETUP ======================
void setup() {
  Serial.begin(115200);
  Serial.println("\n[GW] BOOT");

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  lcd.init();
  lcd.backlight();

  boot_time = millis();
  lcd.clear();
  lcd.print("GRID DATA");
  lcd.setCursor(0,1);
  lcd.print("Starting...");
  beep(2);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("[GW] WIFI | CONNECTING");
  unsigned long t0 = millis();
  while (WiFi.status()!=WL_CONNECTED && millis()-t0<15000) {
    delay(300); Serial.print(".");
  }
  Serial.println();

  if (WiFi.status()==WL_CONNECTED) {
    Serial.printf("[GW] WIFI | OK | IP=%s | CH=%d\n",
                  WiFi.localIP().toString().c_str(), WiFi.channel());
  } else {
    Serial.println("[GW] WIFI | FAILED");
  }

  configTime(7*3600, 0, "pool.ntp.org");

  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(onReceive);
    Serial.println("[GW] ESPNOW | INIT OK");
  }

  set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, AGENT_IP, AGENT_PORT);

  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, NULL, &allocator) == RCL_RET_OK) {
    rclc_node_init_default(&node, "gateway_node", "", &support);

    rclc_publisher_init_default(&pub_ldr, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/ldr");
    rclc_publisher_init_default(&pub_suhu, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/suhu");
    rclc_publisher_init_default(&pub_kelembapan, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/kelembapan");

    ros_ready = true;
    Serial.println("[GW] ROS | CONNECTED");
  } else {
    Serial.println("[GW] ROS | OFFLINE");
  }
}

// ================= LOOP ======================
void loop() {
  if (millis() - boot_time < 5000) return;

  if (ros_ready && !ros_beeped) {
    beep(1);
    ros_beeped = true;
  }

  if (millis() - last_lcd_switch > 2500) {
    last_lcd_switch = millis();
    page = (page + 1) % 5;
    if (page==0) drawStatus();
    if (page==1) drawN1();
    if (page==2) drawN2();
    if (page==3) drawN3();
    if (page==4) drawN4();
  }
}
