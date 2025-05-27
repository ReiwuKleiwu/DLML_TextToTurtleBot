from abc import abstractmethod, ABC

class ExplorationStrategy(ABC):
    @abstractmethod
    def execute(self):
        pass