import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
from flask import Flask, send_from_directory, request, jsonify
import threading
import os

class FlaskNode(Node):
    def __init__(self):
        super().__init__('flask_node')
        self.error_subscription = self.create_subscription(
            String,
            'error_notification',
            self.error_notification_callback,
            10)
        self.command_publisher = self.create_publisher(String, 'control_command', 10)
        self.errors = []
        self.system_on = False

        # Flask 웹 서버 초기화
        self.app = Flask(__name__)

        current_dir = os.path.dirname(__file__)
        self.get_logger().info(f'Current directory: {current_dir}')

        # index.html 반환하는 엔드포인트 설정
        @self.app.route('/')
        def index():
            templates_dir = os.path.join(current_dir, 'templates')
            self.get_logger().info(f'Templates directory: {templates_dir}')
            return send_from_directory(templates_dir, 'index.html')

        # control_command 엔드포인트 설정
        @self.app.route('/control_command', methods=['POST'])
        def control_command():
            command = request.form['command']
            command_msg = String()
            command_msg.data = command
            self.command_publisher.publish(command_msg)
            self.get_logger().info(f'Publishing control command: {command_msg.data}')

            if command == 'system_on':
                self.system_on = True
                self.errors.append('System turned on')
            elif command == 'system_off':
                self.system_on = False
                self.errors.append('System turned off')
            return 'Control command sent', 200

        # error_notification 엔드포인트 설정
        @self.app.route('/error_notification', methods=['POST'])
        def error_notification():
            error_msg = request.form['error']
            self.errors.append(error_msg)
            self.get_logger().info(f'Received error notification: {error_msg}')
            return 'Error notification received', 200

        # errors 엔드포인트 설정 (에러 알림 조회)
        @self.app.route('/errors', methods=['GET'])
        def get_errors():
            return jsonify(self.errors)

        # Flask 서버를 별도의 스레드에서 실행
        self.flask_thread = threading.Thread(target=self.app.run, kwargs={'host': '0.0.0.0', 'port': 5000})
        self.flask_thread.start()

    def error_notification_callback(self, msg):
        self.get_logger().info(f'Received error notification: {msg.data}')
        resp = requests.post('http://localhost:5000/error_notification', data={'error': msg.data})
        if resp.status_code == 200:
            self.get_logger().info('Error notification sent successfully')
        else:
            self.get_logger().error('Failed to send error notification')

def main(args=None):
    rclpy.init(args=args)
    flask_node = FlaskNode()
    rclpy.spin(flask_node)
    flask_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
