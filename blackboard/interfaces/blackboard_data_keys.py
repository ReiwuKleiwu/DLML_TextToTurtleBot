from enum import Enum


class BlackboardDataKey(str, Enum):
    LIDAR_OBSTACLE_PRESENT = "lidar-obstacle-present"

    DETECTED_OBJECTS = 'detected-objects'
    DETECTED_OBJECTS_WITH_COORDINATES = 'detected-objects-with-coordinates'
    TARGET_OBJECT_CLASS = 'target-object-class'
    SELECTED_TARGET_OBJECT = 'selected-target-object'

    ROBOT_POSITION = 'robot-position'
    ROBOT_ORIENTATION = 'robot-orientation'
    ROBOT_IS_TURNING = 'robot-is-turning'

    ROBOT_MAP = "robot-map"

    COMMAND_QUEUE = 'command-queue'
    ACTIVE_COMMAND_ID = 'active-command-id'
