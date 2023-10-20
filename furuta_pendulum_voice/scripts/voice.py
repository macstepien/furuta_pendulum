#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import vlc

from random import randint

from rclpy.duration import Duration
import math

class PendulumVoice(Node):
    def __init__(self):
        super().__init__("pendulum_voice")

        # self.declare_parameter("retraction_height", 0.01)
        # self.retraction_height = self.get_parameter("retraction_height").value

        self.sub = self.create_subscription(JointState, "/joint_states", self.cb, 10)
        self.sub_cmd = self.create_subscription(
            Float64MultiArray, "/effort_control", self.cmd_cb, 10
        )

        self.sounds_path = "file:///home/pendulum/ros2_ws/src/sounds/"
        # playsound(self.sounds_path + "start.mp3")
        p = vlc.MediaPlayer(self.sounds_path + "start.mp3")
        p.play()

        self.last_cmd = 0
        self.current_state = None
        self.last_played_stamp = self.get_clock().now()
        self.min_play_time = Duration(seconds=0.1)

        self.stabilizing = False
        self.stabilized = False

    # def play(self, sound):
    #     pass


    def cmd_cb(self, msg: Float64MultiArray):
        if not self.current_state:
            return 

        if self.get_clock().now() - self.last_played_stamp < self.min_play_time:
            return
        
        angle = math.fabs(self.current_state.position[1])

        if (
            self.last_cmd * msg.data[0] < 0
            and angle > 0.7
        ):
            self.get_logger().info("playing")

            p = vlc.MediaPlayer(
                self.sounds_path + "swing" + str(randint(1, 4)) + ".mp3"
            )
            p.play()
            self.last_played_stamp = self.get_clock().now()
        
        if angle < 0.7 and not self.stabilizing:
            self.stabilizing = True
            p = vlc.MediaPlayer(
                self.sounds_path + "stabilizing.mp3"
            )
            p.play()
        
        if angle < 0.01 and not self.stabilized:
            self.stabilized = True
            p = vlc.MediaPlayer(
                self.sounds_path + "stabilized" + str(randint(1, 3)) + ".mp3"
            )
            p.play()
        
        if angle > 0.1 and self.stabilized:
            self.stabilized = False
            p = vlc.MediaPlayer(
                self.sounds_path + "scream.mp3"
            )
            p.play()

        self.last_cmd = msg.data[0]

    def limit_minus_pi_pi(self, angle):
        angle = math.fmod(angle, 2 * math.pi)
        if (angle > math.pi):
            angle = 2 * math.pi - angle
        elif (angle < -math.pi):
            angle = 2 * math.pi + angle
        return angle

    def cb(self, msg: JointState):
        self.current_state = msg
        self.current_state.position[1] = self.limit_minus_pi_pi(self.current_state.position[1] - math.pi)


def main(args=None):
    rclpy.init(args=args)
    pendulum_voice = PendulumVoice()
    rclpy.spin(pendulum_voice)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

# pulseaudio --kill
# pulseaudio --start
