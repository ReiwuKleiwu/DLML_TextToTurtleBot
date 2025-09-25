from py_trees.behaviour import Behaviour
from py_trees.common import Status
from time import sleep

class ABehaviour(Behaviour):
    def __init__(self, name: str) -> None:
        super(ABehaviour, self).__init__(name)

    def setup(self, **kwargs: any) -> None:
        self.logger.debug('Setup ABehaviour')

    def initialise(self):
        self.logger.debug('Initialized ABheaviour')

    def update(self) -> Status:
        self.logger.debug('Updating ABehaviour')
        sleep(2)
        return Status.SUCCESS
    
    def terminate(self, new_status: Status) -> None:
        self.logger.debug('Terminating ABehaviour')

    