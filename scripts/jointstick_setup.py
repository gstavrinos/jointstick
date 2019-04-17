#!/usr/bin/env python
import os
import rospy
from sensor_msgs.msg import Joy
from controller_manager_msgs.srv import ListControllers

terminal_text = ""
controllers = []
curr_controller = None
saved_axi = -1
saved_buti = -1

read_joy = False
keepItUp = True

YES = ["YES", "yes", "Y", "y"]
NO = ["NO", "no", "N", "n"]
QUIT = ["QUIT", "quit", "Q", "q"]

class JoyAction():
    buttons = []
    axes = []
    step = 0
    limit = 0

    def __init__(self, b, a, s, t):
        self.buttons = b
        self.axes = a
        self.step = s
        self.limit = l

    def __repr__(self):
        return "Buttons: {},\nAxes: {},\nStep: {},\nLimit: {}".format(self.buttons, self.axes, self.step, self.limit)

class Controller():
    name = ""
    type = ""
    joyActions = []

    def __init__(self, n, t, ja):
        self.name = n
        self.type = t
        self.joyActions = ja

    def __repr__(self):
        return "\nName: {},\nType: {},\nActions:{}\n".format(self.name, self.type, self.joyActions)

def youTalkinToMe(prompt, accepted_ans):
    while(True):
        print(prompt)
        if accepted_ans:
            print("(Accepted answers: {})".format(accepted_ans))
        inp = ""
        try:
            # Python 2
            inp = raw_input()
        except:
            # Python 3
            inp = input()
        if not accepted_ans or inp in accepted_ans:
            return inp

def joyInLife(msg):
    global saved_axi, saved_buti
    max_ax = max(map(abs, msg.axes))
    axis_i = -1
    button_i = -1
    if min(msg.axes) == -max_ax and max_ax !=0:
        axis_i = -1 if max_ax == 0 else msg.axes.index(-max_ax)
    elif max_ax != 0:
        axis_i = -1 if max_ax == 0 else msg.axes.index(max_ax)
    if 1 in msg.buttons:
        button_i = msg.buttons.index(1)
    combo = None
    # TODO clear saved when a new button/axis is used
    #if button_i != -1 and button_i != saved_buti:
    saved_buti = button_i if button_i != -1 else saved_buti
    saved_axi = axis_i if axis_i != -1 else saved_axi
    if saved_axi >= 0:
        combo = "[Axis{}]".format(saved_axi)
    if saved_buti >= 0:
        if combo is not None:
            combo += "+[Button{}]".format(saved_buti)
        else:
            combo = "[Button{}]".format(saved_buti)
    return combo

def joyCallback(msg):
    global read_joy, curr_controller
    if read_joy:
        clear()
        print("Controls for {}".format(curr_controller.name))
        print(joyInLife(msg))

def clear():
    os.system("cls" if os.name == "nt" else "clear")

def configureJoyActions():
    global read_joy, keepItUp, curr_controller
    ans = youTalkinToMe("Do you want to see all the discovered controllers?", YES+NO+QUIT)
    if ans in YES:
        print(controllers)
        print("---")
    elif ans in QUIT:
        keepItUp = False
        return
    print("Initiating joystick configuration...")
    youTalkinToMe("Press any enter to continue...", [])
    clear()
    for controller in controllers:
        ans = youTalkinToMe("Do you want to assign a joystick command to {}?".format(controller.name), YES+NO+QUIT)
        if ans in QUIT:
            keepItUp = False
            return
        elif ans in YES:
            curr_controller = controller
            read_joy = True
            while read_joy and keepItUp and not rospy.is_shutdown():
                pass

def main():
    rospy.init_node("jointstick_setup")
    rospy.Subscriber("/joy", Joy, joyCallback)
    controllers_srv = rospy.ServiceProxy("controller_manager/list_controllers", ListControllers)

    print("Reading available controllers....")

    rospy.wait_for_service("controller_manager/list_controllers")

    c = controllers_srv().controller

    for controller in c:
        controllers.append(Controller(controller.name, controller.type, []))

    print("Done!")

    configureJoyActions()

    while not rospy.is_shutdown() and keepItUp:
        rospy.spin()
    print("Byeee!")

main()
