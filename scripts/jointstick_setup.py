#!/usr/bin/env python
import os
import yaml
import rospy
import datetime
import threading
from sensor_msgs.msg import Joy
from controller_manager_msgs.srv import ListControllers
from helper import JoyAction, Controller

ui = [None] # mutable hack. Thanks python!

terminal_text = ""
controllers = []
curr_controller = None
saved_axi = -1
saved_buti = -1

read_joy = False
keepItUp = True

config_file_location = ""

YES = ["YES", "yes", "Y", "y"]
NO = ["NO", "no", "N", "n"]
QUIT = ["QUIT", "quit", "Q", "q"]
SAVE = ["SAVE", "save", "S", "s"]

def joyActionsToDict(actions):
    d = dict()
    for i, action in enumerate(actions):
        d[i] = {"button": action.button,
                "axis": action.axis,
                "step": action.step
                }
    return d

def controllersToDict():
    d = dict()
    for controller in controllers:
        if controller.joyActions:
            d[controller.name] = {"type": controller.type,
                                    "actions": joyActionsToDict(controller.joyActions)
                                }
    return d

def save():
    print("Saving...")
    with open(config_file_location+ os.sep+"config_"+datetime.datetime.now().strftime("%d-%m-%Y_%H-%M-%S")+".yaml", "w") as outfile:
        yaml.dump(controllersToDict(), outfile, default_flow_style=False)
    print("Success!")

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

def youTalkinToMeAboutFloats(prompt):
    while(True):
        print(prompt)
        inp = ""
        try:
            # Python 2
            inp = raw_input()
        except:
            # Python 3
            inp = input()
        try:
            return float(inp)
        except:
            print("Please provide a number!")

def youTalkinToMeAboutThreads(prompt):
    global ui
    print(prompt)
    try:
        # Python 2
        ui[0] = raw_input()
    except:
        # Python 3
        ui[0] = input()

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
    if button_i != -1 and button_i != saved_buti:
        saved_axi = -1
    if axis_i != -1 and axis_i != saved_axi:
        saved_buti = -1
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
    if read_joy:
        clear()
        print("Controls for {}".format(curr_controller.name))
        print(joyInLife(msg))
        if saved_axi != -1 or saved_buti != -1:
            print("Press enter on your keyboard to continue with the current configuration...")

def clear():
    os.system("cls" if os.name == "nt" else "clear")

def configureJoyActions():
    global read_joy, keepItUp, curr_controller, saved_buti, saved_axi, ui
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
        cnt = 0
        while(True):
            ans = ""
            if cnt == 0:
                ans = youTalkinToMe("Do you want to assign a joystick command to {}?".format(controller.name), YES+NO+QUIT)
            else:
                ans = youTalkinToMe("Do you want to assign another joystick command to {}?".format(controller.name), YES+NO+QUIT)
            if ans in QUIT:
                keepItUp = False
                return
            elif ans in YES:
                curr_controller = controller
                read_joy = True
                print("Now use your controller to set a new configuration...")
                t = threading.Thread(target=youTalkinToMeAboutThreads, args=("Don't touch your keyboard now!",))
                t.deamon = True
                t.start()
                while read_joy and keepItUp and not rospy.is_shutdown():
                    if ui[0] is not None and (saved_axi != -1 or saved_buti != -1):
                        read_joy = False
                # TODO prompt user to select which joint to move
                step = youTalkinToMeAboutFloats("Please provide a joint step value:")

                ans = youTalkinToMe("Do you want to save the current configuration?", YES+NO+QUIT)
                if ans in YES:
                    controller.joyActions.append(JoyAction(saved_buti, saved_axi, step))
                    print("Saved! (Not on disk yet!)")
                    print("---")
                    cnt += 1
                elif ans in QUIT:
                    keepItUp = False
                    return
                ui = [None]
                saved_buti = -1
                saved_axi = -1
            elif ans in NO:
                break
    keepItUp = False

    while(True):
        ans = youTalkinToMe("Do you want to save your configuration?", YES+NO+QUIT)
        if ans in YES:
            save()
        elif ans in QUIT:
            keepItUp = False
            break
        elif ans in NO:
            ans = youTalkinToMe("Are you sure?", YES+NO+QUIT)
            if ans in YES or ans in QUIT:
                keepItUp = False
                break

def main():
    global config_file_location
    rospy.init_node("jointstick_setup")
    config_file_location = rospy.get_param("config_file_location", os.path.expanduser("~"))
    print("Config file save location was set as: {}".format(config_file_location))
    rospy.Subscriber("/joy", Joy, joyCallback)
    controllers_srv = rospy.ServiceProxy("controller_manager/list_controllers", ListControllers)

    print("Reading available controllers....")

    rospy.wait_for_service("controller_manager/list_controllers")

    c = controllers_srv().controller

    for controller in c:
        controllers.append(Controller(controller.name, controller.type, [], controller.claimed_resources[0].resources))

    print("Done!")

    configureJoyActions()

    while not rospy.is_shutdown() and keepItUp:
        rospy.spin()
    print("Byeee!")

main()
