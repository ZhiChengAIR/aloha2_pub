import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float64MultiArray
import time

# from data_pub.utils import MasterRobot

from utils import MasterRobot


class ArmPublisher(Node):

    def __init__(self):
        super().__init__("arm_publisher")
        self.arm1_publisher = self.create_publisher(
            Float64MultiArray, "master_left/joint_states", 10
        )
        self.arm2_publisher = self.create_publisher(
            Float64MultiArray, "master_right/joint_states", 10
        )
        publish_frequency = 210
        self.publish_time = 1.0 / publish_frequency
        read_frequency = 70
        self.read_time = 1.0 / read_frequency

        self.arm1 = MasterRobot(
            "master_left", self.get_logger().info, self.read_time, self.publish_time
        )
        self.get_logger().info(
            f"Initializing the left master bot, it will take about a minute.........."
        )
        self.arm1.initialize_servos()
        self.get_logger().info(f"Initialization complete!")
        self.arm1.set_damping_mode()
        self.arm1.start_read_robot_data()

        self.arm2 = MasterRobot(
            "master_right", self.get_logger().info, self.read_time, self.publish_time
        )
        self.get_logger().info(
            f"Initializing the right master bot, it will take about a minute..........!"
        )
        self.arm2.initialize_servos()
        self.get_logger().info(f"Initialization complete!")
        self.arm2.set_damping_mode()
        self.arm2.start_read_robot_data()

        self.timer1 = self.create_timer(self.publish_time, self.publish_arm1_info)
        self.timer2 = self.create_timer(self.publish_time, self.publish_arm2_info)

    def __del__(self):
        self.arm1.stop_read_robot_data()
        self.arm2.stop_read_robot_data()

    def publish_arm1_info(self):
        # now = self.get_clock().now()
        # timestamp_ = now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] * 1e-9
        timestamp_ = time.time()
        arm1_info = self.arm1.get_robot_data()
        # self.get_logger().info(f'Publishing arm1: {timestamp_}')
        # self.get_logger().info(f'arm1_info: {arm1_info[0]+[arm1_info[1]]} type: {type(arm1_info)}')
        if not (
            isinstance(arm1_info, tuple)
            and len(arm1_info) == 2
            and isinstance(arm1_info[0], list)
            and isinstance(arm1_info[1], float)
        ):
            self.get_logger().error("arm1_info is not in the expected format")
            return
        arm1_data = arm1_info[0] + [arm1_info[1]]
        arm1_data.append(timestamp_)
        if not all(isinstance(i, float) for i in arm1_data):
            self.get_logger().error(
                "Not all elements in the first part of arm1_info are floats"
            )
            return

        msg = Float64MultiArray()
        msg.data = arm1_data
        self.arm1_publisher.publish(msg)
        # self.get_logger().info(f'Publishing arm1: {msg.data}')

    def publish_arm2_info(self):
        # now = self.get_clock().now()
        # _timestamp = now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] * 1e-9
        _timestamp = time.time()
        arm2_info = self.arm2.get_robot_data()
        if not (
            isinstance(arm2_info, tuple)
            and len(arm2_info) == 2
            and isinstance(arm2_info[0], list)
            and isinstance(arm2_info[1], float)
        ):
            self.get_logger().error("arm2_info is not in the expected format")
            return
        if not all(isinstance(i, float) for i in arm2_info[0]):
            self.get_logger().error(
                "Not all elements in the first part of arm2_info are floats"
            )
            return
        arm2_data = arm2_info[0] + [arm2_info[1]]
        arm2_data.append(_timestamp)
        if not all(isinstance(i, float) for i in arm2_data):
            self.get_logger().error("Not all elements in arm2_info are floats")
            return
        msg = Float64MultiArray()
        msg.data = arm2_data
        self.arm2_publisher.publish(msg)
        # self.get_logger().info(f'Publishing arm2: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = ArmPublisher()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        # node.arm1.plot()
        # node.arm2.plot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
