import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import torch
from transformers import AutoTokenizer, AutoModelForCausalLM, pipeline
import random
from irobot_create_msgs.msg import IrIntensityVector, IrIntensity
from enum import Enum
import os
import time
import math
import threading

import ollama

from classes.controllers.StateMachine import StateMachine, TurtleBotState, TurtleBotStateSource
from classes.behaviors.obstacle_avoidance.SimpleObstacleAvoidance import SimpleObstacleAvoidance
from classes.behaviors.exploration.RandomExploration import RandomExploration

class ExploreAndDetectNode(Node):
    def __init__(self):
        super().__init__('TextToTurtlebotNode')

        # Initialize TurtleBot state machine
        self.state_machine = StateMachine()

        # Initialize Twist and CMD-Publisher
        self.twist = Twist()
        self.cmd_publisher = self.create_publisher(Twist, '/robot_1/cmd_vel', 10)

        # Initialize Obstacle-Avoider as well as Explorer
        self.obstacle_avoider = SimpleObstacleAvoidance(self.twist, self.cmd_publisher)
        self.explorer = RandomExploration(self.twist, self.cmd_publisher)



    def handle_state(self):
        current_state = self.state_machine.get_current_state()

        if current_state is None:
            raise Exception('[ERROR]: State cannot be None.')

        match current_state.value:
            case TurtleBotState.EXPLORE:
                print('[INFO]: Exploring...')
            case TurtleBotState.AVOID_OBSTACLE:
                print('[INFO]: Avoiding obstacle...')
            case TurtleBotState.OBJECT_FOUND:
                print('[INFO]: Target Object found...')
            case TurtleBotState.OBJECT_FOUND:
                print('[INFO]: Target Object reached...')
            case _:
                print(f'[INFO]: Unknown state: {current_state.value}')