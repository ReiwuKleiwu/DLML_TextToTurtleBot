from enum import Enum

class TurtleBotState(Enum):
    EXPLORE = 0
    AVOID_OBSTACLE = 1
    OBJECT_FOUND = 2
    OBJECT_REACHED = 3
    EXIT = 4

class TurtleBotStateSource(Enum):
    GOAL = 0
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

class StateMachine:
    def __init__(self):
        self.states = [State(
            TurtleBotState.EXPLORE,
            TurtleBotStateSource.GOAL,
        )]

    def push_state(self, new_state: TurtleBotState, new_state_source: TurtleBotStateSource, data: dict = None):
        # Check if the most recent state matches the new state
        if self.get_current_state().value == new_state: return

        print(f"[STATE CHANGE]: Pushed state: {new_state} |  Source: {new_state_source} | Data: {data}")
        self.states.append(State(new_state, new_state_source, data))

    def pop_state(self, state, source):
        # Don't pop if the current state does not match
        if self.get_current_state().value != state: return
        # Don't pop if the source does not match
        if self.get_current_state().source != source: return

        print(f"[STATE CHANGE]: Popped state: {state} |  Source: {source}")
        self.states.pop()

    def get_current_state(self) -> State | None:
        if not self.states:
            # This case should never actually happen, but just in case
            return None
        return self.states[-1]