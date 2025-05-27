from abc import abstractmethod, ABC

class ObstacleAvoidanceStrategy(ABC):
    @abstractmethod
    def execute(self):
        pass