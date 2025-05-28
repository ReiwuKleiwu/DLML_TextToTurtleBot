from abc import abstractmethod, ABC

class TargetNavigationStrategy(ABC):
    @abstractmethod
    def execute(self):
        pass