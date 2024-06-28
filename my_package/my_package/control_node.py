import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.system_subscription = self.create_subscription(
            String,
            'system_status',
            self.system_status_callback,
            10)
        self.command_subscription = self.create_subscription(
            String,
            'control_command',
            self.command_callback,
            10)
        self.left_arm_publisher = self.create_publisher(Float64, 'move_left_arm', 10)
        self.right_arm_publisher = self.create_publisher(Float64, 'move_right_arm', 10)
        self.error_publisher = self.create_publisher(String, 'error_notification', 10)
        self.motion_detected = False
        self.system_on = False
        self.left_arm_position = 0.0
        self.right_arm_position = 0.0

        self.timer = self.create_timer(1.0, self.timer_callback)

    def system_status_callback(self, msg):
        data = msg.data.split(',')
        sensor_status = data[0].split(':')[1].strip()
        motor_status = data[1].split(':')[1].strip()
        
        if sensor_status == 'Motion detected':
            self.motion_detected = True
        else:
            self.motion_detected = False
        
        self.make_decision()

    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received control command: {command}')
        if command == 'raise_error':
            self.publish_error('Example error raised by command')
        elif command == 'system_on':
            self.system_on = True
            self.publish_error('System turned on')
        elif command == 'system_off':
            self.system_on = False
            self.publish_error('System turned off')
        else:
            self.make_decision(command)

    def publish_error(self, error_message):
        error_msg = String()
        error_msg.data = error_message
        self.error_publisher.publish(error_msg)
        self.get_logger().info(f'Publishing error: {error_msg.data}')

    def make_decision(self, command=None):
        if not self.system_on:
            self.get_logger().info('System is off, not sending commands to motor nodes')
            return

        if self.motion_detected or command == 'move_arms':
            self.get_logger().info('Motion detected or move_arms command received')
            self.left_arm_position += 0.1
            self.right_arm_position -= 0.1
            self.publish_arm_positions()
        else:
            self.get_logger().info('No motion detected and no move_arms command, not sending commands to motor nodes')

    def timer_callback(self):
        self.make_decision()

    def publish_arm_positions(self):
        left_arm_msg = Float64()
        right_arm_msg = Float64()

        left_arm_msg.data = self.left_arm_position
        right_arm_msg.data = self.right_arm_position

        self.left_arm_publisher.publish(left_arm_msg)
        self.right_arm_publisher.publish(right_arm_msg)
        self.get_logger().info(f'Publishing left arm position: {left_arm_msg.data}, right arm position: {right_arm_msg.data}')

def main(args=None):
    rclpy.init(args=args)
    control_node = ControlNode()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
