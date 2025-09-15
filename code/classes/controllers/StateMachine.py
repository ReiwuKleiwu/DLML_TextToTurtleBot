from enum import Enum
from classes.events import EventQueue, EventType, Event

class TurtleBotState(Enum):
    IDLE = 0,
    EXPLORE = 1,
    AVOID_OBSTACLE = 2
    OBJECT_FOUND = 3
    OBJECT_REACHED = 4
    EXIT = 5

class TurtleBotStateSource(Enum):
    USER = 0
    LIDAR = 1
    IR = 2
    BUMPER = 3
    CAMERA = 4

class State:
    def __init__(
            self,
            state: TurtleBotState,
            source: TurtleBotStateSource,
            data: dict = None):
        self.value = state
        self.source = source
        self.data = data or {}

    def set_data(self, data: dict):
        self.data = data

class StateMachine:
    def __init__(self):
        self.states = [State(
            TurtleBotState.IDLE,
            TurtleBotStateSource.USER,
        )]

        # Event queue for decoupled communication
        self.event_queue = EventQueue()

        self.log_state_stack()

    def push_state(self, new_state: TurtleBotState, new_state_source: TurtleBotStateSource, data: dict = None):
        # Check if the most recent state matches the new state
        if self.get_current_state().value == new_state and self.get_current_state().source != TurtleBotStateSource.USER: return

        # print(f"[STATE CHANGE]: Pushed state: {new_state} |  Source: {new_state_source} | Data: {data}")
        old_state = self.get_current_state()
        self.states.append(State(new_state, new_state_source, data))

        # Publish state change events
        self.event_queue.publish_event(
            EventType.STATE_PUSHED,
            source="StateMachine",
            data={
                'new_state': new_state.name,
                'new_state_source': new_state_source.name,
                'old_state': old_state.value.name if old_state else None,
                'old_state_source': old_state.source.name if old_state else None,
                'state_data': data
            }
        )

        self.event_queue.publish_event(
            EventType.STATE_CHANGED,
            source="StateMachine",
            data={
                'current_state': new_state.name,
                'current_state_source': new_state_source.name,
                'previous_state': old_state.value.name if old_state else None,
                'state_data': data
            }
        )

        self.log_state_stack()

    def pop_state(self, state, source):
        # Don't pop if the current state does not match
        if self.get_current_state().value != state: return
        # Don't pop if the source does not match
        if self.get_current_state().source != source: return

        # print(f"[STATE CHANGE]: Popped state: {state} |  Source: {source}")
        old_state = self.get_current_state()
        self.states.pop()
        new_current_state = self.get_current_state()

        # Publish state change events
        self.event_queue.publish_event(
            EventType.STATE_POPPED,
            source="StateMachine",
            data={
                'popped_state': state.name,
                'popped_state_source': source.name,
                'new_current_state': new_current_state.value.name if new_current_state else None,
                'new_current_state_source': new_current_state.source.name if new_current_state else None
            }
        )

        self.event_queue.publish_event(
            EventType.STATE_CHANGED,
            source="StateMachine",
            data={
                'current_state': new_current_state.value.name if new_current_state else None,
                'current_state_source': new_current_state.source.name if new_current_state else None,
                'previous_state': old_state.value.name if old_state else None,
                'state_data': new_current_state.data if new_current_state else None
            }
        )

        self.log_state_stack()

    def get_current_state(self) -> State | None:
        if not self.states:
            # This case should never actually happen, but just in case
            return None
        return self.states[-1]
    
    def log_state_stack(self):   
        header = "[STATE STACK]"
      
        cols = ["Idx", "State", "Source", "Data"]

        rows = [
            (
                str(idx),
                state.value.name,
                state.source.name,
                repr(state.data)
            )
            for idx, state in enumerate(self.states)
        ]
        widths = [
            max(len(cols[i]), max(len(row[i]) for row in rows))
            for i in range(4)
        ]
       
        sep = "┼".join("─" * w for w in widths)
       
        def fmt(row):
            return " │ ".join(row[i].ljust(widths[i]) for i in range(4))


        print(header)
        print(sep)
        print(fmt(cols))
        print(sep)
        for row in rows:
            print(fmt(row))
        print(sep)