from irobot_create_msgs.msg import IrIntensityVector, IrIntensity

from classes.controllers.StateMachine import StateMachine, TurtleBotState, TurtleBotStateSource

class IRHandler:
    def __init__(self, state_machine: StateMachine, min_distance = 175):
        self.state_machine = state_machine
        self.min_distance = min_distance

    def handle(self, ir_intensity_vector: IrIntensityVector):
        if any(dist.value > self.min_distance for dist in ir_intensity_vector.readings):
            self.state_machine.push_state(TurtleBotState.AVOID_OBSTACLE, TurtleBotStateSource.IR)
        else:
            self.state_machine.pop_state(TurtleBotState.AVOID_OBSTACLE, TurtleBotStateSource.IR)