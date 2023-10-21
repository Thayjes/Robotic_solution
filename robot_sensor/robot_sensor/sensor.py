""" Simple simulator to generate random samples.
"""

#!/usr/bin/env python3
from array import array
from logging import exception
import socket
import random
from sqlite3 import connect
import sys
from tokenize import Double
import numpy as np
from threading import Thread, Timer
import time
import rclpy
from rclpy.executors import MultiThreadedExecutor
from robot_sensor_interfaces.srv import ArmJointState


class Sensor(Thread):
    def __init__(
        self, address: str, port: int, sampling_rate: int, _delay: float, init_server: bool
    ) -> None:
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
        self.DOF = 6
        self.sampling_rate = sampling_rate

        self.latest_sensor_data = np.zeros(self.DOF)

        if init_server:
            rclpy.init()
            self.server_node = rclpy.create_node(node_name="sensor_server")
            self.srv = self.server_node.create_service(
                ArmJointState, "sensor_data_continuous", self.sensor_callback
            )

    def sensor_callback(self, request, response):
        # Read the latest available sensor data from the thread
        data = self.latest_sensor_data
        response.joint_state.data = [float(elem) for elem in data]
        return response

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

                    # Let's pretend we are really collecting samples
                    time.sleep(
                        int(sample_length) / self.sampling_rate
                        + self.overhead_delay
                        + random.randint(0, 100) / 100000
                    )

                    self.latest_sensor_data = np.random.rand(self.DOF, int(sample_length))

                    # Send the samples to the client
                    self.send(self.latest_sensor_data)

            finally:
                # Clean up the connection
                self.connection.close()


def main(args=None):
    # Launch the first sensor for the custom arm service
    sensor1 = Sensor(
        "127.0.0.3", 10000, 100, 0.001, False
    )  # Define a sensor with 100Hz sampling rate and 1ms delay
    t1 = Thread(target=sensor1.run)
    t1.daemon = True

    # Launch the second sensor for the server node
    sensor2 = Sensor(
        "127.0.0.1", 10000, 100, 0.003, True
    )  # Define a sensor with 100Hz sampling rate and 3ms delay
    t2 = Thread(target=sensor2.run)
    t2.daemon = True

    t1.start()
    t2.start()
    executor = MultiThreadedExecutor()
    executor.add_node(sensor2.server_node)
    executor.spin()

    while True:
        pass


if __name__ == "__main__":
    main()
