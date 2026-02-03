import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float32, Int32

import json
from datetime import datetime


class IoTDataPublisher(Node):
    def __init__(self):
        super().__init__('iot_data_publisher')

        # === Publisher (Untuk dikonsumsi Dashboard via Rosbridge)
        self.pub_suhu = self.create_publisher(String, '/suhu', 10)
        self.pub_kelembapan = self.create_publisher(String, '/kelembapan', 10)
        self.pub_cahaya = self.create_publisher(String, '/ldr', 10)

        # === Subscriber (Menerima data dari micro-ROS/Gateway)
        self.sub_suhu = self.create_subscription(
            Float32, '/gateaway/temperature', self.cb_suhu, 10)
        
        # PERBAIKAN: Sebelumnya ini mengarah ke /temperature
        self.sub_kelembapan = self.create_subscription(
            Float32, '/gateaway/humidity', self.cb_kelembapan, 10) 

        self.sub_ldr = self.create_subscription(
            Int32, '/gateaway/ldr', self.cb_ldr, 10)

    def cb_suhu(self, msg):
        self.publish_dashboard(self.pub_suhu, 'suhu', msg.data, 'C')

    def cb_kelembapan(self, msg):
        self.publish_dashboard(self.pub_kelembapan, 'kelembapan', msg.data, '%')

    def cb_ldr(self, msg):
        # Konversi data LDR jika perlu, atau langsung kirim
        self.publish_dashboard(self.pub_cahaya, 'ldr', msg.data, 'lux')

    def publish_dashboard(self, publisher, name, value, unit):
        payload = {
            'name': name,
            'value': round(float(value), 2), # Memastikan format angka rapi
            'unit': unit,
            'timestamp': datetime.now().isoformat(),
        }
        msg = String()
        msg.data = json.dumps(payload)
        publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IoTDataPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()