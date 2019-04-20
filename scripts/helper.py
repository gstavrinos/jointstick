
class JoyAction():
    button = -1
    axis = -1
    step = 0
    joint = ""

    def __init__(self, b, a, s, j):
        self.button = b
        self.axis = a
        self.step = s
        self.joint = j

    def __repr__(self):
        return "\nButton: {},\nAxis: {},\nStep: {},\nJoint: {}\n".format(self.button, self.axis, self.step, self.joint)

class Controller():
    name = ""
    type = ""
    joints = []
    joyActions = []

    def __init__(self, n, t, ja, j):
        self.name = n
        self.type = t
        self.joyActions = ja
        self.joints = j

    def __repr__(self):
        return "\nName: {},\nType: {},\nActions: {},\nJoints: {}\n".format(self.name, self.type, self.joyActions, self.joints)
