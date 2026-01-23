import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import random
import time

class DemoSidangNode(Node):
    def __init__(self):
        super().__init__('demo_sidang_node')
        
        # Publisher untuk SEMUA Tim
        self.iot_pub = self.create_publisher(String, '/iot_data', 10)
        self.ai_pub = self.create_publisher(String, '/ai_data', 10)
        self.de_pub = self.create_publisher(String, '/data_engineer_data', 10)
        self.event_pub = self.create_publisher(String, '/dashboard_events', 10)
        
        # Timer: Update data setiap 1.5 detik (Biar Stream Indikator tetap HIJAU)
        self.timer = self.create_timer(1.5, self.publish_all)
        self.get_logger().info("🚀 DEMO SIDANG DIMULAI: Semua data sedang dikirim...")

    def publish_all(self):
        # 1. DATA IOT (Suhu naik turun dikit)
        temp = round(random.uniform(29.0, 34.0), 1)
        hum = random.randint(50, 80)
        iot_msg = String()
        iot_msg.data = json.dumps({"temp": temp, "hum": hum})
        self.iot_pub.publish(iot_msg)

        # 2. DATA AI (Ganti-ganti status)
        ai_msg = String()
        status_ai = random.choice(["Aman", "Manusia Terdeteksi", "Kendaraan Masuk", "-"])
        ai_msg.data = status_ai
        self.ai_pub.publish(ai_msg)

        # 3. DATA ENGINEER (Status Server)
        de_msg = String()
        db_load = random.randint(200, 800)
        de_msg.data = f"DB Load: {db_load} req/s | Status: OK"
        self.de_pub.publish(de_msg)

        # 4. EVENT DRIVEN (Hanya kirim kalau random pas angka tertentu, misal 10% peluang)
        if random.random() < 0.10: 
            evt_msg = String()
            evt_msg.data = f"PERINGATAN: Anomali Suhu {temp}°C Terdeteksi!"
            self.event_pub.publish(evt_msg)
            self.get_logger().warn("⚠️ MENGIRIM ALERT KE DASHBOARD!")

        self.get_logger().info(f"Update dikirim: Temp={temp}, AI={status_ai}, DE={db_load}")

def main(args=None):
    rclpy.init(args=args)
    node = DemoSidangNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()