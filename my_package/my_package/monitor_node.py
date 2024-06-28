import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MonitorNode(Node):
    def __init__(self):
        super().__init__('monitor_node')
        self.sensor_subscription = self.create_subscription(
            String,
            'sensor_data',
            self.sensor_callback,
            10)
        self.status_subscription = self.create_subscription(
            String,
            'motor_status',
            self.motor_status_callback,
            10)
        self.publisher_ = self.create_publisher(String, 'system_status', 10)
        self.sensor_data = "No motion detected"
        self.motor_status = "No Motor moved"

    def sensor_callback(self, msg):
        self.sensor_data = msg.data
        self.publish_system_status()

    def motor_status_callback(self, msg):
        self.motor_status = msg.data
        self.publish_system_status()

    def publish_system_status(self):
        if self.sensor_data and self.motor_status:
            status_msg = String()
            status_msg.data = f'Sensor: {self.sensor_data}, Motor: {self.motor_status}'
            self.publisher_.publish(status_msg)
            self.get_logger().info(f'Publishing system status: {status_msg.data}')

def main(args=None):
    rclpy.init(args=args)
    monitor_node = MonitorNode()
    rclpy.spin(monitor_node)
    monitor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
