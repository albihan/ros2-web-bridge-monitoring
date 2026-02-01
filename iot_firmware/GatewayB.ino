#include <micro_ros_arduino.h>
#include <WiFi.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float32.h>
#include <rclc/executor.h> 

char WIFI_SSID[] = "1ab";
char WIFI_PASS[] = "gvyg6249";
char AGENT_IP[]  = "10.0.2.15";
uint16_t AGENT_PORT = 8888;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

// ===== 3 PUBLISHER =====
rcl_publisher_t pub_suhu;
rcl_publisher_t pub_kelembapan;
rcl_publisher_t pub_ldr;

std_msgs__msg__Float32 msg_suhu;
std_msgs__msg__Float32 msg_kelembapan;
std_msgs__msg__Float32 msg_ldr;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, -1);

  set_microros_wifi_transports(
    WIFI_SSID,
    WIFI_PASS,
    AGENT_IP,
    AGENT_PORT
  );

  delay(2000);

  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "gateway_b_node", "", &support);

  rclc_publisher_init_default(
    &pub_suhu,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/suhu"
  );

  rclc_publisher_init_default(
    &pub_kelembapan,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/kelembapan"
  );

  rclc_publisher_init_default(
    &pub_ldr,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/ldr"
  );

  // 🔴 INI YANG KAMU KURANG
  rclc_executor_init(&executor, &support.context, 3, &allocator);

  Serial.println("=== GATEWAY B READY (ROS2 ACTIVE) ===");
}

void loop() {
  if (Serial2.available()) {
    String line = Serial2.readStringUntil('\n');
    line.trim();

    float suhu = -1, kelembapan = -1, ldr = -1;
    int idx;

    idx = line.indexOf("suhu:");
    if (idx >= 0) suhu = line.substring(idx + 5).toFloat();

    idx = line.indexOf("kelembapan:");
    if (idx >= 0) kelembapan = line.substring(idx + 11).toFloat();

    idx = line.indexOf("ldr:");
    if (idx >= 0) ldr = line.substring(idx + 4).toFloat();

    if (suhu >= 0) {
      msg_suhu.data = suhu;
      rcl_publish(&pub_suhu, &msg_suhu, NULL);
    }

    if (kelembapan >= 0) {
      msg_kelembapan.data = kelembapan;
      rcl_publish(&pub_kelembapan, &msg_kelembapan, NULL);
    }

    if (ldr >= 0) {
      msg_ldr.data = ldr;
      rcl_publish(&pub_ldr, &msg_ldr, NULL);
    }

    Serial.println(line);
  }

  // 🔴 WAJIB
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
