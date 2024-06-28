import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rclpy.qos import QoSProfile

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.publisher_ = self.create_publisher(String, 'sensor_data', QoSProfile(depth=10))
        self.subscription = self.create_subscription(
            LaserScan,
            '/laser_scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # 타이머를 설정하여 1초마다 모션 감지 메시지를 발행합니다. (레이저 센서가 없기 미구현)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.motion_state = True  # 초기 상태는 모션 감지로 설정
        
    def listener_callback(self, msg):
        if self.detect_motion(msg):
            self.publish_motion_detected()
        else:
            self.publish_no_motion_detected()

    def detect_motion(self, scan_data):
        for distance in scan_data.ranges:
            if distance < 1.0:
                return True
        return False

    def timer_callback(self):
        if self.motion_state:
            self.publish_motion_detected()
        else:
            self.publish_no_motion_detected()
        self.motion_state = not self.motion_state  # 상태를 번갈아 가면서 변경

    def publish_motion_detected(self):
        signal_msg = String()
        signal_msg.data = 'Motion detected'
        self.publisher_.publish(signal_msg)
        self.get_logger().info('Publishing: "%s"' % signal_msg.data)

    def publish_no_motion_detected(self):
        signal_msg = String()
        signal_msg.data = 'No motion detected'
        self.publisher_.publish(signal_msg)
        self.get_logger().info('Publishing: "%s"' % signal_msg.data)

def main(args=None):
    rclpy.init(args=args)
    sensor_node = SensorNode()
    rclpy.spin(sensor_node)
    sensor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
