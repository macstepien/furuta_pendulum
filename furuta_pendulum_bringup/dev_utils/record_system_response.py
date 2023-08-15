#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import rosbag2_py

from datetime import datetime


class ImpulseResponseRecorder(Node):
    def __init__(self):
        super().__init__("impulse_response_recorder")
        self.writer = rosbag2_py.SequentialWriter()

        dt_string = datetime.now().strftime("%H-%M-%S-%d-%m-%Y")

        # gazebo or de
        # simulation_type = "gazebo"
        simulation_type = "de"

        # Step or impulse
        self.response_type = "step"
        # self.response_type = "impulse"

        storage_options = rosbag2_py._storage.StorageOptions(
            uri=self.response_type + "_response_" + simulation_type,
            storage_id="sqlite3",
        )
        converter_options = rosbag2_py._storage.ConverterOptions("", "")
        self.writer.open(storage_options, converter_options)

        topic_info = rosbag2_py._storage.TopicMetadata(
            name="joint_states",
            type="sensor_msgs/msg/JointState",
            serialization_format="cdr",
        )
        self.writer.create_topic(topic_info)

        self.subscription = self.create_subscription(
            JointState, "joint_states", self.topic_callback, 10
        )

        self.impulse_pub = self.create_publisher(
            Float64MultiArray, "effort_control", 10
        )

        self.impulse_published = False
        self.impulse_timer = self.create_timer(0.002, self.impulse_timer_cb)

        self.max_num_of_data = 5000
        self.current_num_of_data = 0

        self.torque_value_first = 0.43
        if self.response_type == "impulse":
            self.torque_value_later = 0.0
        else:
            self.torque_value_later = 0.43

    def impulse_timer_cb(self):
        impulse_msg = Float64MultiArray()
        if not self.impulse_published:
            impulse_msg.data.append(self.torque_value_first)
        else:
            impulse_msg.data.append(self.torque_value_later)
        self.impulse_published = True
        self.impulse_pub.publish(impulse_msg)

    def topic_callback(self, msg):
        self.writer.write(
            "joint_states", serialize_message(msg), self.get_clock().now().nanoseconds
        )
        self.current_num_of_data += 1
        if self.current_num_of_data > self.max_num_of_data:
            self.get_logger().info("Finished recording, shutting down")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    sbr = ImpulseResponseRecorder()
    rclpy.spin(sbr)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
