from irobot_create_msgs.msg import IrIntensityVector, IrIntensity

from classes.controllers.StateMachine import StateMachine, TurtleBotState, TurtleBotStateSource
from classes.events import EventQueue, EventType, Event

class IRHandler:
    def __init__(self, state_machine: StateMachine, min_distance = 175):
        self.state_machine = state_machine
        self.min_distance = min_distance

        # Event queue for decoupled communication
        self.event_queue = EventQueue()

        # Track obstacle state for event publishing
        self._obstacle_detected = False

    def handle(self, ir_intensity_vector: IrIntensityVector):
        obstacle_detected = any(dist.value > self.min_distance for dist in ir_intensity_vector.readings)

        if obstacle_detected:
            if not self._obstacle_detected:
                self.state_machine.push_state(TurtleBotState.AVOID_OBSTACLE, TurtleBotStateSource.IR)

                # Publish IR obstacle detected event
                max_intensity = max(dist.value for dist in ir_intensity_vector.readings)
                self.event_queue.publish_event(
                    EventType.IR_OBSTACLE_DETECTED,
                    source="IRHandler",
                    data={
                        'max_intensity': max_intensity,
                        'min_distance': self.min_distance,
                        'sensor_type': 'ir',
                        'readings': [dist.value for dist in ir_intensity_vector.readings]
                    }
                )
                self._obstacle_detected = True
        else:
            if self._obstacle_detected:
                self.state_machine.pop_state(TurtleBotState.AVOID_OBSTACLE, TurtleBotStateSource.IR)

                # Publish IR obstacle cleared event
                self.event_queue.publish_event(
                    EventType.IR_OBSTACLE_CLEARED,
                    source="IRHandler",
                    data={
                        'sensor_type': 'ir'
                    }
                )
                self._obstacle_detected = False