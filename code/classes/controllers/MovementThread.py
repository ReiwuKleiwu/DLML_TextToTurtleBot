import threading
from classes.nodes.TextToTurtlebotNode import TextToTurtlebotNode

class MovementThread(threading.Thread):
    def __init__(self, rate, node: TextToTurtlebotNode):
        super(MovementThread, self).__init__()
        self.condition = threading.Condition()
        self.done = False

        self.node = node
        self.timeout = 1.0 / rate if rate != 0.0 else None

        self.start()

    def run(self):
        while not self.done:
            self.node.handle_state()
            self.condition.acquire()
            self.condition.wait(self.timeout)
            self.condition.release()

    def stop(self):
        self.done = True
