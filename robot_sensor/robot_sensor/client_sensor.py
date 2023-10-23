from robot_sensor_interfaces.srv import SensorData
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSProfile


class SensorClient(Node):
    def __init__(self, num_samples_custom, num_samples_latest):
        super().__init__("sensor_client")
        # Create the callback groups for service clients and timers
        client_cb_group = MutuallyExclusiveCallbackGroup()
        client_cb_group_continuous = MutuallyExclusiveCallbackGroup()
        timer1_cb_group = MutuallyExclusiveCallbackGroup()

        # Create the clients and wait for service to come online
        self.cli = self.create_client(SensorData, "get_sensor_data", callback_group=client_cb_group)
        self.cli_continuous = self.create_client(
            SensorData, "sensor_data_continuous", callback_group=client_cb_group_continuous
        )
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("discrete sensor service not available, waiting again...")
        while not self.cli_continuous.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("continuous sensor service not available, waiting again...")

        # Define the publishers
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
        self.sensor_pub_discrete = self.create_publisher(
            msg_type=Float64MultiArray, topic="sensor_data_discrete", qos_profile=qos_profile
        )

        self.sensor_pub_continuous = self.create_publisher(
            msg_type=Float64MultiArray, topic="sensor_data_continuous", qos_profile=qos_profile
        )
        # Define the timers
        pub_freq = 500.0
        timer_period = 1 / pub_freq  # seconds
        self.pub = self.create_timer(
            timer_period, self.pub_callback, callback_group=timer1_cb_group
        )

        self.DOF = 3
        self.num_samples_custom = num_samples_custom
        self.num_samples_latest = num_samples_latest
        self.num_calls = 0

    def format_msg_data(self, msg, num_samples):
        # This is almost always zero there is no empty padding at the start of your data
        msg.layout.data_offset = 0
        # create two dimensions in the dim array
        msg.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
        # dim[0] is the vertical dimension of your matrix (degrees of freedom)
        msg.layout.dim[0].label = "DOF"
        msg.layout.dim[0].size = self.DOF
        msg.layout.dim[0].stride = int(self.DOF * num_samples)
        # dim[1] is the horizontal dimension of your matrix (number of samples)
        msg.layout.dim[1].label = "samples"
        msg.layout.dim[1].size = num_samples
        msg.layout.dim[1].stride = num_samples
        return msg

    def pub_callback(self):
        msg = Float64MultiArray()
        req = SensorData.Request()
        req.num_samples = self.num_samples_custom
        # Call the custom service sensor first
        result_discrete = self.cli.call(req)
        msg.data = result_discrete.joint_state.data
        msg = self.format_msg_data(msg=msg, num_samples=req.num_samples)
        self.sensor_pub_discrete.publish(msg)
        # Then call the service with latest sensor feed
        result_continuous = self.cli_continuous.call(req)
        msg.data = result_continuous.joint_state.data
        msg = self.format_msg_data(msg=msg, num_samples=self.num_samples_latest)
        self.sensor_pub_continuous.publish(msg)
        self.num_calls += 1


def main(args=None):
    rclpy.init()
    sensor_client = SensorClient(num_samples_custom=1, num_samples_latest=100)
    executor = MultiThreadedExecutor()
    executor.add_node(sensor_client)
    try:
        sensor_client.get_logger().info("Beginning client, shut down with CTRL-C")
        executor.spin()
    except KeyboardInterrupt:
        sensor_client.get_logger().info("Keyboard interrupt, shutting down.\n")
    sensor_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
