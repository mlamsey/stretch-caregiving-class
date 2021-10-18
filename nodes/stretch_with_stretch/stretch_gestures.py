from __future__ import print_function

# Python Imports
import threading
import numpy as np
import math

# ROS Imports
import rospy
from sensor_msgs.msg import JointState

# Stretch Imports
import hello_helpers.hello_misc as hm

# Helper functions
def get_head_state(joint_states):
    head_pan_index = joint_states.name.index('joint_head_pan')
    head_tilt_index = joint_states.name.index('joint_head_tilt')
    head_pan = joint_states.position[head_pan_index]
    head_tilt = joint_states.position[head_tilt_index]
    return head_pan, head_tilt

# Main Class
class StretchGestures():
    def __init__(self, move_to_pose_function):
        self.move_to_pose = move_to_pose_function
        
        # ROS Config
        self.rate = 10

        # Threading for callbacks
        self.joint_states_lock = threading.Lock()

        # Subscriber!
        self.joint_states_subscriber = rospy.Subscriber('/stretch/joint_states', JointState, self.joint_state_callback)

        # Joint State Inits
        self.joint_states = None
        self.wrist_position = None
        self.head_pan = None
        self.head_tilt = None
        
        # Motion config
        self.nod_magnitude = 0.3 # rad
        self.shake_head_magnitude = 0.2 # rad
        self.wiggle_wrist_magnitude = 0.2 # rad

    def joint_state_callback(self, joint_states):
        # Update Joint State
        with self.joint_states_lock:
            self.joint_states = joint_states
        
        # Unpack Joint State
        wrist_position, wrist_velocity, wrist_effort = hm.get_wrist_state(joint_states)
        head_pan, head_tilt = get_head_state(joint_states)

        # Store necessary items
        self.wrist_position = wrist_position
        self.head_pan = head_pan
        self.head_tilt = head_tilt

    # ACTIONS!
    def nod(self, n_nods):
        if any([self.joint_states]):
            original_tilt = self.head_tilt
            nod_up_angle = original_tilt + self.nod_magnitude
            nod_down_angle = original_tilt - self.nod_magnitude

            for i in range(n_nods):
                self.move_to_pose({'joint_head_tilt': nod_up_angle})
                rospy.sleep(0.2)
                self.move_to_pose({'joint_head_tilt': nod_down_angle})
                rospy.sleep(0.2)
            
            self.move_to_pose({'joint_head_tilt': original_tilt})

    def shake_head(self, n_shakes):
        if any([self.joint_states]):
            original_pan = self.head_pan
            pan_left_angle = original_pan - self.shake_head_magnitude
            pan_right_angle = original_pan + self.shake_head_magnitude

            for i in range(n_shakes):
                self.move_to_pose({'joint_head_pan': pan_left_angle})
                rospy.sleep(0.1)
                self.move_to_pose({'joint_head_pan': pan_right_angle})
                rospy.sleep(0.1)

            self.move_to_pose({'joint_head_pan': original_pan})

    def wiggle_wrist_yaw(self, n_wiggles):
        if any([self.joint_states]):
            original_yaw = self.wrist_position
            wiggle_left_angle = original_yaw + self.wiggle_wrist_magnitude
            wiggle_right_angle = original_yaw - self.wiggle_wrist_magnitude

            for i in range(n_wiggles):
                self.move_to_pose({'joint_wrist_yaw': wiggle_left_angle})
                self.move_to_pose({'joint_wrist_yaw': wiggle_right_angle})

            self.move_to_pose({'joint_wrist_yaw': original_yaw})

    def wrist_extension_oscillation(self, n_oscillations, amplitude_meters):
        original_extension = self.wrist_position
        oscillation_in_position = original_extension - amplitude_meters
        oscillation_out_position = original_extension + amplitude_meters

        for i in range(n_oscillations):
            self.move_to_pose({"wrist_extension": oscillation_in_position})
            self.move_to_pose({"wrist_extension": oscillation_out_position})

        self.move_to_pose({"wrist_extension": original_extension})

    def base_forward_oscillation(self, n_oscillations, amplitude_meters):
        self.move_to_pose({"translate_mobile_base": 0.5 * amplitude_meters})
        for i in range(n_oscillations - 1):
            self.move_to_pose({"translate_mobile_base": -amplitude_meters})
            self.move_to_pose({"translate_mobile_base": amplitude_meters})

        self.move_to_pose({"translate_mobile_base": -0.5 * amplitude_meters})

    def gripper_circles(self, n_circles, radius_meters):
        n_points = 25
        x = [radius_meters * math.sin(i) for i in np.linspace(0,2*math.pi,n_points)]
        y = [radius_meters * math.cos(i) for i in np.linspace(0,2*math.pi,n_points)]
        
        original_extension = self.wrist_position
        for i in range(math.floor(n_circles)):
            for j in range(n_points):
                if j == 0:
                    x_target = x[j]
                else:
                    x_target = x[j] - x[j-1]

                y_target = original_extension + y[j]
                self.move_to_pose({"translate_mobile_base": x_target, "wrist_extension": y_target})
        
        self.move_to_pose({"wrist_extension": original_extension})

    # TESTS MOTIONS!
    def test_motions(self):
        rate = rospy.Rate(self.rate)

        while self.head_tilt is None:
            rospy.sleep(1)

        self.nod(2)
        self.shake_head(2)