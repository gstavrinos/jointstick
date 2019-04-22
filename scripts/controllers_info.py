# A file with lists of supported controllers and other
# categorisations on those controllers

# A list of controllers that this script can handle
supported_controllers = ["effort_controllers/JointPositionController",
                        "position_controllers/JointTrajectoryController",
                        "diff_drive_controller/DiffDriveController"]

# A list of controllers that do not apply on specific joints
ignore_controller_joints = ["effort_controllers/JointPositionController",
                            "diff_drive_controller/DiffDriveController"]

# A list of controllers that do not need a step value, but an explicit one (like velocity)
non_step_controllers = ["diff_drive_controller/DiffDriveController"]

# A list of controllers that need a user specified message field (e.g. "linear.x")
message_field_controllers = ["position_controllers/JointTrajectoryController",
                            "diff_drive_controller/DiffDriveController"]

# A list of controllers that their command is based on a Twist message
twist_controllers = ["diff_drive_controller/DiffDriveController"]

# A list of controllers that their commands is based on a JointTrajectory message
joint_traj_controllers = ["position_controllers/JointTrajectoryController"]

# A list of controllers that their commands is based on a Float64 message
float_controllers = ["effort_controllers/JointPositionController"]

topic_extension = {
                    "effort_controllers/JointPositionController": "/command",
                    "position_controllers/JointTrajectoryController": "/command",
                    "diff_drive_controller/DiffDriveController": "/cmd_vel"
                }


