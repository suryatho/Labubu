
class Task():
    def __init__(self, priority: int = 1, name = None):
        self.priority = priority
        self.resources = set()
        # Name for display; defaults to the class name when not provided
        self.name = name or self.__class__.__name__

    def __repr__(self) -> str:
        # show the name and object memory address in hex
        return f"<{self.name if self.name is not None else ('')} at 0x{id(self):x}>"

    def __str__(self) -> str:
        return self.__repr__()

    def start(self, timestamp: float):
        pass

    def execute(self, timestamp: float):
        pass

    def isFinished(self, timestamp: float) -> bool:
        return False

    def end(self, canceled: bool, timestamp: float):
        pass

    def add_resources(self, *resources):
        self.resources.update(resources)