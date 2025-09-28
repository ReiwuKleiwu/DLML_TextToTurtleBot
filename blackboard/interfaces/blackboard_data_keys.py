from enum import Enum


class BlackboardDataKey(str, Enum):
    LIDAR_OBSTACLE_PRESENT = "lidar-obstacle-present"

    CAMERA_RESOLUTION = 'camera-resolution'

    DETECTED_OBJECTS = 'detected-objects'
    DETECTED_OBJECTS_WITH_COORDINATES = 'detected-objects-with-coordinates'
    TARGET_OBJECT_CLASS = 'target-object-class'
    SELECTED_TARGET_OBJECT = 'selected-target-object'

    ROBOT_POSITION = 'robot-position'
    ROBOT_ORIENTATION = 'robot-orientation'
    ROBOT_IS_TURNING = 'robot-is-turning'
    ROBOT_TRAIL = 'robot-trail'

    ROBOT_MAP = "robot-map"

    COMMAND_QUEUE = 'command-queue'
    ACTIVE_COMMAND = 'active-command'

    DRIVE_TARGET_DISTANCE = 'drive-target-distance'
    DRIVE_START_POSE = 'drive-start-pose'
    DRIVE_DISTANCE_TRAVELLED = 'drive-distance-travelled'
    DRIVE_DIRECTION_SIGN = 'drive-direction-sign'

    ROTATE_TARGET_ANGLE = 'rotate-target-angle'
    ROTATE_START_YAW = 'rotate-start-yaw'
    ROTATE_ANGLE_TRAVELLED = 'rotate-angle-travelled'
    ROTATE_DIRECTION_SIGN = 'rotate-direction-sign'

    NAVIGATION_CURRENT_GOAL = 'navigation-current-goal'
    NAVIGATION_STATUS = 'navigation-status'
    NAVIGATION_FEEDBACK = 'navigation-feedback'

    BEHAVIOUR_TREE_PAUSED = 'behaviour-tree-paused'
    LLM_CHAT_LOG = 'llm-chat-log'
