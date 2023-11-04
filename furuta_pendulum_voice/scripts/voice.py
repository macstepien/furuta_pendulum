#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import vlc

from random import randint, uniform

from rclpy.duration import Duration
import math

from enum import Enum


class PendulumState(Enum):
    SWING_UP = 1
    STABILIZING = 2
    STABILIZED = 3
    MOVED = 4


class PendulumVoice(Node):
    def __init__(self):
        super().__init__("pendulum_voice")

        # self.declare_parameter("retraction_height", 0.01)
        # self.retraction_height = self.get_parameter("retraction_height").value
        # self.get_logger().info("playing")

        self.sub = self.create_subscription(JointState, "/joint_states", self.cb, 10)
        self.sub_cmd = self.create_subscription(
            Float64MultiArray, "/effort_control", self.cmd_cb, 10
        )

        self.sounds_path = "file:///home/pendulum/ros2_ws/src/sounds/"

        self.last_played_sound = None
        
        self.current_sound = None
        self.play("start")

        self.last_cmd = 0
        self.current_state = None
        self.last_played_stamp = self.get_clock().now()
        self.min_play_time = Duration(seconds=0.1)

        self.pendulum_state = PendulumState.SWING_UP

        self.cmd_direction_changed = False
        self.stabilized_play_time = Duration(seconds=1.0)

    def play(self, sound):
        if self.current_sound:
            self.current_sound.stop()
            
        self.last_played_stamp = self.get_clock().now()
        self.last_played_sound = sound
        self.current_sound = vlc.MediaPlayer(self.sounds_path + sound + ".mp3")
        self.current_sound.play()

    def cmd_cb(self, msg: Float64MultiArray):
        self.cmd_direction_changed = self.last_cmd * msg.data[0] < 0
        self.last_cmd = msg.data[0]

        self.voice()

    def voice(self):
        if not self.current_state:
            return

        if self.get_clock().now() - self.last_played_stamp < self.min_play_time:
            return

        if self.cmd_direction_changed and self.pendulum_state == PendulumState.SWING_UP:
            self.play("swing" + str(randint(1, 4)))
        elif self.pendulum_state == PendulumState.STABILIZING and not (
            "stabilizing" in self.last_played_sound
        ):
            self.play("stabilizing")
        elif (
            self.pendulum_state == PendulumState.STABILIZED
            and self.get_clock().now() - self.last_played_stamp
            > self.stabilized_play_time
        ):
            self.play("stabilized" + str(randint(1, 4)))
            self.stabilized_play_time = Duration(seconds=uniform(5, 10))
        elif self.pendulum_state == PendulumState.MOVED and not (
            "scream" in self.last_played_sound
        ):
            self.play("scream")

    def limit_minus_pi_pi(self, angle):
        angle = math.fmod(angle, 2 * math.pi)
        if angle > math.pi:
            angle = 2 * math.pi - angle
        elif angle < -math.pi:
            angle = 2 * math.pi + angle
        return angle

    def cb(self, msg: JointState):
        self.current_state = msg
        self.current_state.position[1] = self.limit_minus_pi_pi(
            self.current_state.position[1] - math.pi
        )

        angle = math.fabs(self.current_state.position[1])
        vel = math.fabs(self.current_state.velocity[1])

        if angle > 0.7:
            self.pendulum_state = PendulumState.SWING_UP
        elif angle < 0.7 and self.pendulum_state == PendulumState.SWING_UP:
            self.pendulum_state = PendulumState.STABILIZING
        elif (
            angle < 0.01
            and vel < 0.01
            and (
                self.pendulum_state == PendulumState.STABILIZING
                or self.pendulum_state == PendulumState.MOVED
            )
        ):
            self.pendulum_state = PendulumState.STABILIZED
        elif angle > 0.01 and self.pendulum_state == PendulumState.STABILIZED:
            self.pendulum_state = PendulumState.MOVED


def main(args=None):
    rclpy.init(args=args)
    pendulum_voice = PendulumVoice()
    rclpy.spin(pendulum_voice)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
