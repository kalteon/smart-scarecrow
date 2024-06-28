import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.left_arm_subscription = self.create_subscription(
            Float64,
            'move_left_arm',
            self.left_arm_callback,
            10)
        self.right_arm_subscription = self.create_subscription(
            Float64,
            'move_right_arm',
            self.right_arm_callback,
            10)
        self.left_arm_publisher = self.create_publisher(Float64, '/left_arm_joint_controller/command', 10)
        self.right_arm_publisher = self.create_publisher(Float64, '/right_arm_joint_controller/command', 10)
        self.status_publisher = self.create_publisher(String, 'motor_status', 10)
        
        # 초기 팔 위치 설정
        self.left_arm_position = 0.0
        self.right_arm_position = 0.0

    def left_arm_callback(self, msg):
        self.get_logger().info(f'Left arm move request received: {msg.data}')
        self.left_arm_position = self.wrap_position(self.left_arm_position + msg.data)
        position_msg = Float64()
        position_msg.data = self.left_arm_position
        self.left_arm_publisher.publish(position_msg)
        self.publish_status(f'Left arm moved to position {position_msg.data}')

    def right_arm_callback(self, msg):
        self.get_logger().info(f'Right arm move request received: {msg.data}')
        self.right_arm_position = self.wrap_position(self.right_arm_position + msg.data)
        position_msg = Float64()
        position_msg.data = self.right_arm_position
        self.right_arm_publisher.publish(position_msg)
        self.publish_status(f'Right arm moved to position {position_msg.data}')

    def wrap_position(self, position, min_pos=-1.0, max_pos=1.0):
        range_size = max_pos - min_pos
        position = (position - min_pos) % range_size + min_pos
        return position

    def publish_status(self, status):
        status_msg = String()
        status_msg.data = status
        self.status_publisher.publish(status_msg)
        self.get_logger().info(f'Publishing status: {status}')

def main(args=None):
    rclpy.init(args=args)
    motor_node = MotorNode()
    rclpy.spin(motor_node)
    motor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
