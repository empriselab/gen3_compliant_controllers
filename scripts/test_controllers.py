#! /usr/bin/env python

import sys

import numpy as np
import rospy
import tf2_ros
from controller_manager_msgs.srv import SwitchController
from kortex_hardware.srv import ModeService
from moveit_msgs.msg import CartesianTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

if __name__ == "__main__":
    rospy.init_node('test_controller_py')
    # get command line arguments for control mode
    controller_space = sys.argv[1]
    mode = "effort"
    topic = ""
    if controller_space == "task":
        topic = "/task_space_compliant_controller/command"
    elif controller_space == "joint":
        topic = "/joint_space_compliant_controller/command"
    else:
        print("Invalid control mode")
        sys.exit(0)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    print("Switching to", controller_space, "space controller. The robot might jerk a bit as it is switching to effort mode.")
    rospy.wait_for_service("controller_manager/switch_controller")
    rospy.wait_for_service("set_control_mode")
    switch_controller = rospy.ServiceProxy(
        "controller_manager/switch_controller", SwitchController
    )
    mode_change = rospy.ServiceProxy("set_control_mode", ModeService)

    try:
        if controller_space == "task":
            resp = switch_controller(
                ["task_space_compliant_controller"],
                ["velocity_controller", "joint_space_compliant_controller"],
                1,
                0,
                5,
            )
        else:
            resp = switch_controller(
                ["joint_space_compliant_controller"],
                ["velocity_controller", "task_space_compliant_controller"],
                1,
                0,
                5,
            )
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

    try:
        resp1 = mode_change(mode)
        print("Mode change response: ", resp1)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

    if mode == "stop":
        sys.exit(0)

    print("Switch complete. You can now send commands to the robot.")
    try:
        cmd_pub = None
        if controller_space == "task":
            cmd_pub = rospy.Publisher(topic, CartesianTrajectoryPoint, queue_size=1)
        else:
            cmd_pub = rospy.Publisher(topic, JointTrajectoryPoint, queue_size=1)
        rate = rospy.Rate(100.0)

        while not rospy.is_shutdown():
            # wait for message from joint_states
            cmd = None
            if controller_space == "task":
                cmd = CartesianTrajectoryPoint()
                # get current end effector position using tf
                transform = None
                while True:
                    try:
                        transform = tfBuffer.lookup_transform('base_link', 'end_effector_link', rospy.Time())
                        break
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        rate.sleep()
                        continue
                print(
                    "Current end effector position: ", transform.transform.translation
                )
                # take input from user
                cmd.point.pose.position.x = float(input("Enter x: "))
                cmd.point.pose.position.y = float(input("Enter y: "))
                cmd.point.pose.position.z = float(input("Enter z: "))
                cmd.point.pose.orientation.x = transform.transform.rotation.x
                cmd.point.pose.orientation.y = transform.transform.rotation.y
                cmd.point.pose.orientation.z = transform.transform.rotation.z
                cmd.point.pose.orientation.w = transform.transform.rotation.w
            else:
                # get current joint state from joint_states
                joint_states = rospy.wait_for_message(
                    "/joint_states", JointState, timeout=None
                )
                tmp = np.array(joint_states.position)[1:]
                print("Current joint state: ", tmp)
                cmd = JointTrajectoryPoint()
                # take input from user
                dof = int(input("Enter dof (1 - NDOF): "))
                value = float(input("Enter value: "))
                tmp[dof - 1] = value
                cmd.positions = tmp.tolist()
                cmd.velocities = [0] * len(tmp)
            cmd_pub.publish(cmd)
            input("Press Enter to continue...")

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
