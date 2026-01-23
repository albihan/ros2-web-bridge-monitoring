import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import random
import time
from datetime import datetime

class DataEngineerMockNode(Node):
    def __init__(self):
        super().__init__('mock_sensor_publisher')
        
        # Publisher Dashboard & AI
        self.publisher_ = self.create_publisher(String, 'dashboard/telemetry', 10)
        self.publisher_ai = self.create_publisher(String, '/ai_data', 10)

        # Timer 1 detik
        self.timer = self.create_timer(1.0, self.publish_data)
        self.seq_counter = 0

    def publish_data(self):
        self.seq_counter += 1

        # 1. LOGIC DATA ENGINEER
        # random.choice HANYA butuh satu argumen (list)
        is_sensor_ready = random.choice([True, True, False]) 
            
        # Saya ubah ke round(..., 1) agar ada komanya (mirip sensor asli)
        current_temp = round(random.uniform(25.0, 32.0), 1) if is_sensor_ready else None
        current_humid = round(random.uniform(60.0, 90.0), 1) if is_sensor_ready else None
        current_ldr = round(random.uniform(400.0, 800.0), 1)
        
        timestamp_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        payload = {
            "humid": current_humid,
            "ldr": current_ldr,
            "node": 1,
            "seq": self.seq_counter,
            "temp": current_temp,
            "timestamp": timestamp_str
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Mock DE: {msg.data}')

        # 2. LOGIC AI (SISTEM CERDAS)
        # Tambahkan "None" agar kadang-kadang tidak mendeteksi apa-apa
        objects = ["Tumbuhan Sedang Sakit", "Tumbuhan Sehat", "None"]
        detected = random.choice(objects)

        ai_payload = {}
        
        # Cek String "None" bukan tipe data None
        if detected != "None":
            confidence = round(random.uniform(0.79, 0.99), 2)
            ai_payload = {
                "class": detected,
                "confidence": confidence,
                "count": random.randint(1, 3),
                "timestamp": datetime.now().strftime("%H:%M:%S")
            }
        else:
            ai_payload = { "class": "No Object", "count": 0 }

        msg_ai = String()
        msg_ai.data = json.dumps(ai_payload)
        self.publisher_ai.publish(msg_ai)
        self.get_logger().info(f'Mock AI: {msg_ai.data}')

# --- PERBAIKAN INDENTASI (HARUS DI LUAR CLASS) ---
def main(args=None):
    rclpy.init(args=args)
    node = DataEngineerMockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()