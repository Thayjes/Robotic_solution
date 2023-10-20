import socket

import numpy as np
import rclpy
from rclpy.node import Node

from robot_sensor_interfaces.srv import ArmJointState


class ArmSensorService(Node):
    def __init__(self) -> None:
        super().__init__('arm_sensor_service')
        self.srv = self.create_service(ArmJointState, 'get_sensor_data',
                                       self.sensor_callback)
        # Create a TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Connect the socket to the port where the server is listening
        server_address = ('127.0.0.3', 10000)
        print(f'connecting to {server_address[0]} port {server_address[1]}')
        while True:
            try:
                self.sock.connect(server_address)
                break
            except ConnectionRefusedError as e:
                print(f'Waiting for connection: {e}')
                pass

    def sensor_callback(self, request, response):
        num_samples = request.num_samples
        message_string = str(num_samples)
        message = message_string.encode()
        self.sock.sendall(message)
        byte_data = self.sock.recv(10000)
        data =  np.frombuffer(byte_data)
        response.joint_state.data = [float(elem) for elem in data]
        self.get_logger().info('Incoming request\nnum_samples: %d' % (request.num_samples))
        return response


def main(args=None):
    rclpy.init(args=args)
    sensor_service = ArmSensorService()
    rclpy.spin(sensor_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()