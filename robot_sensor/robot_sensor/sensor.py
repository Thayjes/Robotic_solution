""" Simple simulator to generate random samples.
"""

#!/usr/bin/env python3
import socket
import random
import numpy as np
from threading import Thread
import time
from rclpy.node import Node
import rclpy
from robot_sensor_interfaces.srv import SensorData
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class Sensor(Thread):
    def __init__(self, address: str, port: int, sampling_rate: int, _delay: float) -> None:
        # Create a TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Define the server address and port
        self.server_address = (address, port)

        # Bind the server arg to the socket
        self.sock.bind(self.server_address)

        # Listen for incoming connections
        self.sock.listen(1)

        # This is an artificial delay we add as the over head
        self.overhead_delay = _delay

        self.client_address = None
        self.sampling_rate = 0
        self.sensor_running = False
        self.connected = False
        self.DOF = 3
        self.sampling_rate = sampling_rate

    def connect(self) -> bool:
        # Wait for a connection
        print("waiting for a connection")
        while self.client_address is None:
            self.connection, self.client_address = self.sock.accept()
        self.connected = True
        print("connection from", self.client_address)
        return True

    def recive(self, buffer_size: int) -> int:
        # Read a buffer size from the socket
        recived_msg = self.connection.recv(buffer_size)
        msg = recived_msg.decode()
        if msg.isnumeric():
            return int(msg)
        else:
            print("recived a not numeric msg")

    def set_overhead(self, _delay: float) -> None:
        self.overhead_delay = _delay

    def set_sampling_rate(self, sampleing_rate: int) -> None:
        self.sampling_rate = sampleing_rate

    def send(self, data: np.ndarray) -> bool:
        # Send the data to the client
        if self.connected:
            try:
                self.connection.sendall(data.tobytes())
                return True
            except:
                print("Something went wrong in sending samples to: ", self.client_address)
                return False

    def run(self):
        if self.connect():
            try:
                while True:
                    # Recive the request for the number of samples
                    sample_length = self.recive(500)

                    # Lets assume we have access to the latest data before
                    # the time taken to collect samples
                    self.latest_sensor_data = np.random.rand(self.DOF, int(sample_length))

                    # Let's pretend we are really collecting samples
                    time.sleep(
                        int(sample_length) / self.sampling_rate
                        + self.overhead_delay
                        + random.randint(0, 100) / 100000
                    )

                    # Send the samples to the client
                    self.send(self.latest_sensor_data)

            finally:
                # Clean up the connection
                self.connection.close()


class SensorServer(Node):
    def __init__(self, num_samples) -> None:
        super().__init__("sensor_server")
        srv_cb_group = MutuallyExclusiveCallbackGroup()
        self.srv = self.create_service(
            SensorData, "sensor_data_continuous", self.service_cb, callback_group=srv_cb_group
        )
        self.DOF = 3
        self.num_samples = num_samples
        # Create a TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Connect the socket to the port where the server is listening
        # We connect the second service to the second sensor
        server_address = ("127.0.0.1", 10000)
        print(f"connecting to {server_address[0]} port {server_address[1]}")
        self.sock.connect(server_address)

        timer_freq = 100  # Hz
        timer_period = 1 / timer_freq  # seconds
        timer1_cb_group = MutuallyExclusiveCallbackGroup()
        self.pub = self.create_timer(
            timer_period, self.sensor_callback, callback_group=timer1_cb_group
        )
        self.data = np.zeros(self.DOF)
        self.num_calls = 0

    def service_cb(self, request, response):
        response.joint_state.data = [float(elem) for elem in self.data]
        self.get_logger().info("Incoming request sensor server node")
        return response

    def sensor_callback(self):
        num_samples = self.num_samples
        message_string = str(num_samples)
        message = message_string.encode()
        self.sock.sendall(message)
        byte_data = self.sock.recv(10000)
        self.data = np.frombuffer(byte_data)
        self.num_calls += 1


def main(args=None):
    # Launch the first 3DOF sensor for the custom arm service
    sensor1 = Sensor(
        "127.0.0.3", 10000, 2000, 0.001
    )  # Define a sensor with 2000Hz sampling rate and 1ms delay
    t1 = Thread(target=sensor1.run)
    t1.daemon = True

    # Launch the second 3DOF sensor for the sensor server node
    sensor2 = Sensor(
        "127.0.0.1", 10000, 2000, 0.001
    )  # Define a sensor with 2000Hz sampling rate and 1ms delay
    t2 = Thread(target=sensor2.run)
    t2.daemon = True

    t1.start()
    t2.start()
    rclpy.init()
    sensor_server = SensorServer(num_samples=100)
    executor = MultiThreadedExecutor()
    executor.add_node(sensor_server)
    executor.spin()


if __name__ == "__main__":
    main()
