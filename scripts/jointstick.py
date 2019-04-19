#!/usr/bin/env python
import yaml
import rospy
from sensor_msgs.msg import Joy, JointState

def jointStatesCallback(msg):
    #TODO
    pass

def joyCallback(msg):
    #TODO
    pass

def main():
    rospy.init_node("jointstick_setup")
    config_file = rospy.get_param("config_file", "-")
    #config_file = "/home/gstavrinos/config_18-04-2019_16-48-52.yaml"
    if config_file == "-":
        print("No config file provided. Exiting...")
        return
    cf = yaml.load_all(open(config_file, "r"))
    #print(cf)
    rospy.Subscriber("/joy", Joy, joyCallback)
    rospy.Subscriber("/joint_states", JointState, jointStatesCallback)
    while not rospy.is_shutdown():
        rospy.spin()

main()
