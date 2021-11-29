#!/usr/bin/env python2
from __future__ import print_function

import json
import threading
import numpy as np

# Stretch Imports
import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv
import stretch_gestures as sg

# ROS Stuff
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String, UInt8, Float32


class StretchWithStretch(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        self.gestures = sg.StretchGestures(self.move_to_pose)
        self.move_base = nv.MoveBase(self)
        self.rate = 20

        # subscribers
        # fmt: off
        self.joint_states_subscriber = rospy.Subscriber(
            "/stretch/joint_states", JointState, self.joint_state_callback
        )
        self.nod_head_subscriber = rospy.Subscriber(
            "/sws_nod_head", UInt8, self.nod_head_callback
        )
        self.select_exercise_subscriber = rospy.Subscriber(
            "/sws_select_exercise", String, self.select_exercise_callback
        )
        # fmt: on

        # publishers
        # fmt: off
        self.notify_publisher = rospy.Publisher(
            "/sws_notify", Bool, queue_size=1
        )
        self.sws_ready_publisher = rospy.Publisher(
            "/sws_ready", Bool, queue_size=1
        )
        self.wrist_contact_publisher = rospy.Publisher(
            "/sws_contact_detected", Bool, queue_size=1
        )
        self.sws_start_exercise_publisher = rospy.Publisher(
            "/sws_start_exercise", String, queue_size=1
        )
        self.sws_stop_exercise_publisher = rospy.Publisher(
            "/sws_stop_exercise", Bool, queue_size=1
        )
        self.speech_recognition_publisher = rospy.Publisher(
            "/speech/start_recording", Float32, queue_size=1
        )
        # fmt: on

        # joint state
        self.joint_states_lock = threading.Lock()
        self.joint_states = None

        self.wrist_position = None
        self.lift_position = None
        self.lift_effort = None

        self.wrist_extension = None
        self.wrist_extension_effort = None

        self.wrist_yaw = None
        self.wrist_yaw_effort = None

        # contact detection thresholds
        self.lift_effort_max = 50  # N?
        self.lift_effort_min = 35  # N?
        self.wrist_extension_max = 10  # N?
        self.wrist_extension_min = -10  # N?
        self.wrist_yaw_effort_contact_threshold = 0.35  # N

        # calibration
        self.pre_calibration_pose = {
            "joint_lift": 0.7,  # m
            "wrist_extension": 0.05,  # m
            "joint_wrist_yaw": 0.0,  # rad
            "joint_gripper_finger_left": 0.5,
            "joint_head_pan": np.deg2rad(-90.0),  # rad
            "joint_head_tilt": -0.2,  # rad
        }
        self.post_calibration_pose = self.pre_calibration_pose.copy()
        self.post_calibration_pose["joint_gripper_finger_left"] = 0.05
        self.calibration_xya = None
        self.exercise_radius = 0.635  # m (average human arm length)

        # exercise state
        self.current_exercise = None

    # --------- #
    # callbacks #
    # --------- #

    def joint_state_callback(self, joint_states):
        with self.joint_states_lock:
            self.joint_states = joint_states

        # unpack joint state
        self.lift_position, _, _ = hm.get_lift_state(joint_states)
        self.wrist_position, _, _ = hm.get_wrist_state(joint_states)

        # fmt: off
        self.lift_effort = joint_states.effort[
            joint_states.name.index("joint_lift")
        ]
        self.wrist_extension_effort = joint_states.effort[
            joint_states.name.index("wrist_extension")
        ]
        self.wrist_yaw_effort = joint_states.effort[
            joint_states.name.index("joint_wrist_yaw")
        ]
        # fmt: on

    def nod_head_callback(self, data):
        self.gestures.nod(data.data)

    def select_exercise_callback(self, data):
        self.current_exercise = json.loads(data.data)

    # ------- #
    # helpers #
    # ------- #

    @staticmethod
    def _position_to_xya(pos):
        return [pos["x"], pos["y"], pos["a"]]

    def _goto_position(self, xya):
        if xya is None:
            return

        rospy.loginfo("moving to ({:0.2f}, {:0.2f}, {:0.2f})...".format(*xya))  # debug

        robot_xya, _ = self.get_robot_floor_pose_xya()
        current_xya = np.array(robot_xya) - np.array(self.calibration_xya)
        delta_x = xya[0] - current_xya[0]
        delta_y = xya[1] - current_xya[1]

        rospy.loginfo("delta x: {:0.2f}".format(delta_x))  # debug
        rospy.loginfo("delta y: {:0.2f}".format(delta_y))  # debug

        # rotate towards the target
        movement_a = np.arctan2(delta_y, delta_x)
        delta_a = hm.angle_diff_rad(movement_a, current_xya[2])
        rospy.loginfo("delta a: {:0.2f}".format(delta_a))  # debug
        _ = self.move_base.turn(delta_a, publish_visualizations=False)

        # move forward towards the target
        delta = np.sqrt(np.power(delta_x, 2) + np.power(delta_y, 2))
        rospy.loginfo("delta: {:0.2f}".format(delta))  # debug
        _ = self.move_base.forward(delta, detect_obstacles=False)

        # rotate towards the exercise angle
        delta_a = hm.angle_diff_rad(xya[2], movement_a)
        rospy.loginfo("delta a: {:0.2f}".format(delta_a))  # debug
        _ = self.move_base.turn(delta_a, publish_visualizations=False)

        rospy.loginfo(
            "moving to ({:0.2f}, {:0.2f}, {:0.2f})... done!".format(*xya)
        )  # debug

    def _change_pose(self, src, dst, step):
        if src is None:
            return
        if dst is None:
            dst = src
        h = src["arm_height"] + step * (dst["arm_height"] - src["arm_height"])
        e = src["arm_extension"] + step * (dst["arm_extension"] - src["arm_extension"])
        y = src["wrist_yaw"] + step * (dst["wrist_yaw"] - src["wrist_yaw"])
        self.move_to_pose(
            {
                "joint_lift": h,
                "wrist_extension": e,
                "joint_wrist_yaw": y,
            }
        )

    def _check_for_wrist_contact(self, publish=False):
        bool_wrist_contact = False

        if self.wrist_yaw_effort is not None:
            if abs(self.wrist_yaw_effort) > self.wrist_yaw_effort_contact_threshold:
                bool_wrist_contact = True

        # if self.lift_effort is not None:
        #     if (
        #         self.lift_effort < self.lift_effort_min
        #         or self.lift_effort > self.lift_effort_max
        #     ):
        #         bool_wrist_contact = True

        # if self.wrist_extension_effort is not None:
        #     if (
        #         self.wrist_extension_effort < self.wrist_extension_min
        #         or self.wrist_extension_effort > self.wrist_extension_max
        #     ):
        #         bool_wrist_contact = True

        if publish:
            self.wrist_contact_publisher.publish(bool_wrist_contact)

        return bool_wrist_contact

    # ------ #
    # states #
    # ------ #

    def wait_for_initialization(self):
        # wait 30 sec for joint states to be ready
        stop_wait_time = rospy.Time.now() + rospy.Duration.from_sec(30.0)
        while not rospy.is_shutdown() and self.joint_states is None:
            self.sws_ready_publisher.publish(False)
            if rospy.Time.now() > stop_wait_time:
                break
            rospy.sleep(1)

        if rospy.is_shutdown():
            return

        # move to the calibration pose
        self.move_to_pose(self.pre_calibration_pose)

        # wait 10 sec for wrist contact to stabalize (hack)
        stop_wait_time = rospy.Time.now() + rospy.Duration.from_sec(10.0)
        while not rospy.is_shutdown() and self._check_for_wrist_contact():
            self.sws_ready_publisher.publish(False)
            if rospy.Time.now() > stop_wait_time:
                break
            rospy.sleep(1)

        if rospy.is_shutdown():
            return

        # notify
        rospy.loginfo("Initialization completed.")
        self.notify_publisher.publish(True)

    def wait_for_calibration_handshake(self):
        rate = rospy.Rate(self.rate)

        if rospy.is_shutdown():
            return

        # move to calibration pose
        self.move_to_pose(self.pre_calibration_pose)

        rospy.loginfo("*" * 40)
        rospy.loginfo("Give the robot the ball to start the game!")
        rospy.loginfo("*" * 40)

        while not rospy.is_shutdown():
            self.sws_ready_publisher.publish(False)
            if self._check_for_wrist_contact(publish=False):
                break
            rate.sleep()

        if rospy.is_shutdown():
            return

        self.move_to_pose(self.post_calibration_pose)
        rospy.sleep(2)  # give robot time to settle

        # set x, y, a here in case we moved the robot before calibration
        self.calibration_xya, _ = self.get_robot_floor_pose_xya()

        # notify
        rospy.loginfo("Calibration completed.")
        self.notify_publisher.publish(True)

    def wait_for_exercise(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.sws_ready_publisher.publish(True)
            if self.current_exercise is not None:
                break
            rate.sleep()

        self.sws_ready_publisher.publish(False)

    def execute_exercise(self):
        if self.current_exercise is None:
            return

        rate = rospy.Rate(self.rate)

        # extract exercise info
        name = self.current_exercise["name"]
        movement = self.current_exercise["movement"]
        first_pose = movement["poses"][0]["start"]
        total_duration = sum(item["duration"] for item in movement["poses"])
        has_cognitive = self.current_exercise["audio"]["active"]

        # move to exercise position
        self._goto_position(self._position_to_xya(movement["position"]))
        if rospy.is_shutdown():
            return

        # move to exercise pose
        self._change_pose(first_pose, first_pose, 0)
        if rospy.is_shutdown():
            return

        # notify
        rospy.loginfo("Starting exercise {}".format(name))
        self.sws_ready_publisher.publish(False)
        self.sws_start_exercise_publisher.publish(name)

        # wait for startup chime
        stop_time = rospy.Time.now() + rospy.Duration.from_sec(2.0)
        while not rospy.is_shutdown():
            if rospy.Time.now() > stop_time:
                break
            rate.sleep()
        if rospy.is_shutdown():
            return

        # notify speech recognition
        if has_cognitive:
            self.speech_recognition_publisher.publish(total_duration)

        # execute exercise
        for pose in movement["poses"]:
            duration = pose["duration"]
            start_time = rospy.Time.now().to_sec()
            while not rospy.is_shutdown():
                self.sws_ready_publisher.publish(False)
                self._check_for_wrist_contact(publish=True)
                delta = rospy.Time.now().to_sec() - start_time
                if delta > duration:
                    break
                self._change_pose(pose["start"], pose["stop"], delta / duration)
                rate.sleep()

        # reset
        self.current_exercise = None

        # notify
        rospy.loginfo("Stoping exercise {}".format(name))
        self.sws_ready_publisher.publish(False)
        self.sws_stop_exercise_publisher.publish(True)

        # wait to announce score
        stop_time = rospy.Time.now() + rospy.Duration.from_sec(2.0)
        while not rospy.is_shutdown():
            if rospy.Time.now() > stop_time:
                break
            rate.sleep()

    def main(self):
        hm.HelloNode.main(
            self,
            "stretch_with_stretch_node",
            "node_namespace",
            wait_for_first_pointcloud=False,
        )
        self.wait_for_initialization()
        self.wait_for_calibration_handshake()
        while not rospy.is_shutdown():
            self.wait_for_exercise()
            self.execute_exercise()


if __name__ == "__main__":
    StretchWithStretch().main()
