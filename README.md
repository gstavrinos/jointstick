<img src="https://raw.githubusercontent.com/gstavrinos/jointstick/master/images/jointstick_logo.png" width=40%>
<sup><sup>Logo was created by combining two icons by https://thenounproject.com/prosymbols/</sup></sup>
A ROS node that allows you to move any joint with any controller!

# How to use it

## Setup
The first step before teleoperating your robot, is to create some joystick bindings for it. For this, the `joystick_setup.py` script will help you. Assumptions that the setup script makes that you need to keep in mind:
* Your robot controllers are already runnning (simulated or real drivers). This is important because the setup script uses the `controller_manager` services to discover and analyse your robot's controllers.
* The `joy_node` of the `joy` package is running. This is important because the setup script subscribes to the `/joy` topic to save your bindings.

During setup the script will guide you through the process via messages in the console. When using the setup script for the first time(s) pay attention to the console messages to avoid wrong bindings.

TODO add screnshots here

The setup script uses the `jointstick_setup/config_file` param to know where to save the file that contains your joystick bindings. If not such parameter is found, the config file is saved in your home directory. The configuration file name always contains the date and time of creation.

### Tip 1
If you have made a wrong binding but you have already done too much work for your controllers, just save the configurations and then manually remove the wrong binding from the file. Since it's a .yaml, it's very human readable.

### Tip 2
Based on the controller you are binding controls for, you will be asked by the setup script to provide some information. This should be straight forward, but keep in mind that you don't need to provide both directions for an axis. For example, if `Axis1` is bound for the `x` direction, you don't have to provide two different bindings for the positive and negative direction, since the joystick axes provide both positive and negative values. The same does not apply for buttons. If Button1 on your keyboard is used to move your robot forward on the x axis, another button with negative value should be used to move your robot backwards, since joystick buttons always have a zero or positive value.

## Teleoperation
TODO
