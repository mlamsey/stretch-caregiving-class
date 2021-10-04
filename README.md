# BMED 8813 Fall 2021 - Robotic Caregiving with Stretch
Repository for Team Blue: Stretch with Stretch

## Stretch + ROS
This section contains some useful information for programming a Stretch using ROS.

### Basic Node Setup
It is suggested to create an object ROS node for controlling the Stretch. This node should inherit `hello_helpers.hello_misc.HelloNode`. Also, the Stretch node should contain a main method which calls `hm.HelloNode.main()`. Basic setup for the Stretch node is shown below:

```
#!/usr/bin/env python3
# change the above to python2 if running ROS melodic

import hello_helpers.hello_misc as hm

class stretch_node(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        # other init things
        
    def main(self):
        hm.HelloNode.main(self, 'stretch_test_node', 'node_namespace', wait_for_first_pointcloud=False)
        # main routine for Stretch
        
if __name__ == '__main__':
    node = stretch_node()
    node.main()
```

### Moving the Robot: Joint Commands
Basic motion of the joints of the Stretch can be achieved using the method `self.move_to_pose({"joint_name" : value})`. The dictionary of joint names and target positions can contain multiple joint names, and will move to all of the target values in the dictionary at the same time. The list of possible `joint_name` values is given below. `value` is in SI units: meters for joint motion and base translation, and radians for revolute joints and base rotation.

__Head Joints__:
* `joint_head_pan`
* `joint_head_tilt`

__Arm + Lift__:
* `joint_lift` (arm height)
* `wrist_extension` (arm extension)

__Wrist (non-dexterous)__:
* `joint_wrist_yaw`
* `joint_gripper_finger_left` (opens and closes gripper - value is linear actuation of gripper (m))

__Mobile Base__:
* `translate_mobile_base` (forward and backward, no splines)
* `rotate_mobile_base` (rotate about center of drive wheelbase)

An example of commanding joint motion is given below.

```
def move_gripper_in_a_rectangle(self):
    # Define target positions
    arm_extensions = [0.2, 0.4, 0.4, 0.2, 0.2]
    lift_heights = [0.4, 0.4, 0.6, 0.6, 0.4]
    
    # Execute motions
    for i in range(len(arm_extensions)):
        target_pose = {"joint_lift": lift_heights[i], "wrist_extension": arm_extensions[i]}
        self.move_to_pose(target_pose)
```
