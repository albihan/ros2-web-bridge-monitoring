import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


class EventBridge(Node):
    def __init__(self):
        super().__init__('event_bridge')

        # Publisher ke dashboard
        self.pub_dashboard = self.create_publisher(String, '/dashboard_events', 10)

        # Subscribe ke 3 topik sumber
        self.create_subscription(String, '/iot_data', self.wrap('iot'), 10)
        self.create_subscription(String, '/ai_data', self.wrap('ai'), 10)
        self.create_subscription(String, '/data_engineer_data', self.wrap('dataeng'), 10)

    def wrap(self, source):
        def callback(msg):
            try:
                payload = {
                    "source": source,
                    "data": json.loads(msg.data)
                }
            except json.JSONDecodeError:
                self.get_logger().error(f"Invalid JSON from {source}: {msg.data}")
                return

            out = String()
            out.data = json.dumps(payload)
            self.pub_dashboard.publish(out)

            self.get_logger().info(f"Forwarded: {payload}")

        return callback


def main(args=None):
    rclpy.init(args=args)
    node = EventBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
