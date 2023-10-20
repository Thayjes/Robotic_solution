
from robot_sensor_interfaces.srv import ArmJointState
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSProfile


class SensorClient(Node):

    def __init__(self):
        super().__init__('sensor_client')
        client_cb_group = MutuallyExclusiveCallbackGroup()
        timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.cli = self.create_client(ArmJointState, 'get_sensor_data',
                                      callback_group=client_cb_group)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ArmJointState.Request()
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
        self.sensor_pub_discrete = self.create_publisher(msg_type=Float64MultiArray,
                                                         topic='sensor_data_discrete',
                                                         qos_profile=qos_profile)
        pub_freq = 500.0
        timer_period =  1 / pub_freq  # seconds
        self.pub1 = self.create_timer(timer_period, self.pub1_callback,
                                      callback_group=timer_cb_group)
        self.futures = []

    def send_request(self, num_samples):
        self.req.num_samples = num_samples
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_request_continuous():
        pass

    def pub1_callback(self):
        msg = Float64MultiArray()
        req = ArmJointState.Request()
        req.num_samples = 1
        self.get_logger().info('Sending request for pub1')
        result = self.cli.call(req)
        self.get_logger().info('Received response for pub1')
        print(f'Result = {result.joint_state}')
        msg.data = result.joint_state.data
        self.sensor_pub_discrete.publish(msg)



def main(args=None):
    rclpy.init()
    sensor_client = SensorClient()
    executor = MultiThreadedExecutor()
    executor.add_node(sensor_client)

    try:
        sensor_client.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        sensor_client.get_logger().info('Keyboard interrupt, shutting down.\n')
    sensor_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()