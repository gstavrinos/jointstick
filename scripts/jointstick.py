#!/usr/bin/env python
import os
import yaml
import time
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, JointState
from trajectory_msgs.msg import JointTrajectory
from controller_manager_msgs.srv import ListControllers

# Imports from custom files of this package
from controllers_info import *
from helper import Controller, JoyAction

controllers = []
joint_states = None
publishers = dict()

# Just the joint state callback
def jointStatesCallback(msg):
    global joint_states
    joint_states = msg

# Just the joy callback
def joyCallback(msg):
    print msg

# Initialize publisher objects for all controller with joy bindings
def fixPublishers():
    global publishers
    for controller in controllers:
        publishers[controller.name] = rospy.Publisher(
                        controller.name+topic_extension[controller.type],
                        controller_type_correspondence[controller.type],
                        queue_size=1)

# Translate a dict record to a JoyAction object
def joyActionFromDict(dict_record):
    ax = dict_record["axis"]
    b = dict_record["button"]
    j = dict_record["joint"]
    mf = dict_record["msg_field"]
    v = dict_record["value"]
    return JoyAction(b, ax, v, j, mf)

# Translate the read YAML dict to a list of Controller objects
def controllersFromYAML(yaml_mess):
    global controllers
    for k in yaml_mess:
        actions = []
        for controller in controllers:
            if controller.name == k:
                for j in yaml_mess[k]["actions"]:
                    controller.joyActions.append(joyActionFromDict(yaml_mess[k]["actions"][j]))
                break

# Initialisations and other boring stuff
def main():
    global controllers
    rospy.init_node("jointstick_setup")
    config_file = rospy.get_param("config_file", "-")
    config_file = "/home/gstavrinos/config_22-04-2019_15-52-40.yaml"
    if config_file == "-":
        print("No config file provided. Exiting...")
        return
    elif not os.path.isfile(config_file):
        print("Config file {} not found. Exiting...").format(config_file)
        return
    controllers_srv = rospy.ServiceProxy("controller_manager/list_controllers", ListControllers)

    print("Reading available controllers....")

    rospy.wait_for_service("controller_manager/list_controllers")

    c = controllers_srv().controller

    for controller in c:
        controllers.append(Controller(controller.name, controller.type, [], controller.claimed_resources[0].resources))

    print("Done!")

    controllersFromYAML(yaml.safe_load(open(config_file, "r")))

    # Delete controllers that do not have any joy bindings
    controllers = [x for x in controllers if x.joyActions]

    fixPublishers()

    rospy.Subscriber("/joy", Joy, joyCallback)
    rospy.Subscriber("/joint_states", JointState, jointStatesCallback)
    while not rospy.is_shutdown():
        rospy.spin()

main()
