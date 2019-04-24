#!/usr/bin/env python
import os
import yaml
import time
import rospy
import threading
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, JointState
from trajectory_msgs.msg import JointTrajectory
from controller_manager_msgs.srv import ListControllers

# Imports from custom files of this package
from controllers_info import *
from helper import Controller, JoyAction

joint_states = None
joy_msg = None

controllers = []
publishers = dict()
actions_to_exec = []
rate = 10# Hz

# Thread flag
kill = False

# Just the joint state callback
def jointStatesCallback(msg):
    global joint_states
    joint_states = msg

# Just the joy callback
def joyCallback(msg):
    global joy_msg
    joy_msg = msg
    bindingsOfJoy(msg)

# Initialize publisher objects for all controller with joy bindings
def initPublishers():
    global publishers
    for controller in controllers:
        publishers[controller.name] = rospy.Publisher(
                        controller.name+topic_extension[controller.type],
                        controller_type_correspondence[controller.type],
                        queue_size=1)

# With the action_to_exec at hand, send some commands to the robot! (in a new thread)
def execCommands():
    while(not kill):
        actions = dict()
        for action, controller in actions_to_exec:
            if controller.type == "diff_drive_controller/DiffDriveController":
                msg = Twist()
                if action.msg_field == "linear.x":
                    msg.linear.x = action.value if action.axis == -1 else joy_msg.axes[action.axis] * action.value
                elif action.msg_field == "linear.y":
                    msg.linear.y = action.value if action.axis == -1 else joy_msg.axes[action.axis] * action.value
                elif action.msg_field == "linear.z":
                    msg.linear.z = action.value if action.axis == -1 else joy_msg.axes[action.axis] * action.value
                elif action.msg_field == "angular.x":
                    msg.angular.x = action.value if action.axis == -1 else joy_msg.axes[action.axis] * action.value
                elif action.msg_field == "angular.y":
                    msg.angular.y = action.value if action.axis == -1 else joy_msg.axes[action.axis] * action.value
                elif action.msg_field == "angular.z":
                    msg.angular.z = action.value if action.axis == -1 else joy_msg.axes[action.axis] * action.value
                if controller not in actions:
                    actions[controller] = [msg]
                else:
                    actions[controller].append(msg)
            elif controller.type == "effort_controllers/JointPositionController":
                v = action.value if action.axis == -1 else joy_msg.axes[action.axis] * action.value
                stepIt = joint_states.position[joint_states.name.index(action.joint)] + v
                publishers[controller.name].publish(stepIt)
            elif controller.type == "position_controllers/JointTrajectoryController":
                # TODO
                pass
        for controller in actions:
            msg = None
            if controller.type == "diff_drive_controller/DiffDriveController":
                msg = Twist()
                for action in actions[controller]:
                    msg.linear.x = action.linear.x if abs(action.linear.x) > abs(msg.linear.x) else msg.linear.x
                    msg.linear.y = action.linear.y if abs(action.linear.y) > abs(msg.linear.y) else msg.linear.y
                    msg.linear.z = action.linear.z if abs(action.linear.z) > abs(msg.linear.z) else msg.linear.z
                    msg.angular.x = action.angular.x if abs(action.angular.x) > abs(msg.angular.x) else msg.angular.x
                    msg.angular.y = action.angular.y if abs(action.angular.y) > abs(msg.angular.y) else msg.angular.y
                    msg.angular.z = action.angular.z if abs(action.angular.z) > abs(msg.angular.z) else msg.angular.z
            elif controller.type == "position_controllers/JointTrajectoryController":
                # TODO
                pass
            publishers[controller.name].publish(msg)
        time.sleep(1/rate)

# Save in actions_to_exec the joy bindings that should be used to send a command to a joint
def bindingsOfJoy(msg):
    global actions_to_exec
    actions_to_exec = []
    pressed_buttons = [x for x, e in enumerate(msg.buttons) if e != 0]
    pressed_axes = [x for x, e in enumerate(msg.axes) if e != 0]
    for controller in controllers:
        for ja in controller.joyActions:
            if ((ja.axis in pressed_axes) or (not pressed_axes and ja.axis == -1)) and\
            ((ja.button in pressed_buttons) or (not pressed_buttons and ja.button == -1)):
                actions_to_exec.append((ja, controller))

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
    global controllers, kill, rate
    rospy.init_node("jointstick_setup")
    config_file = rospy.get_param("config_file", "-")
    config_file = "/home/gstavrinos/config_22-04-2019_15-52-40.yaml"
    if config_file == "-":
        print("No config file provided. Exiting...")
        return
    elif not os.path.isfile(config_file):
        print("Config file {} not found. Exiting...").format(config_file)
        return
    rate = rospy.get_param("rate", 10)
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

    initPublishers()

    rospy.Subscriber("/joy", Joy, joyCallback)
    rospy.Subscriber("/joint_states", JointState, jointStatesCallback)

    t = threading.Thread(target=execCommands)
    t.deamon = True
    t.start()

    while not rospy.is_shutdown():
        rospy.spin()

    kill = True

main()
