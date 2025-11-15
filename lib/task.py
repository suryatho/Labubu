
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

class RunnableTask(Task):
    def __init__(self, executefunc, priority: int = 1, end: bool = True, name = None):
        super().__init__(priority, name)
        self.does_end = end
        self.executefunc = executefunc

    def execute(self, timestamp: float):
        self.executefunc(timestamp)

    def isFinished(self, timestamp: float) -> bool:
        return self.does_end

class WaitTask(Task):
    def __init__(self, wait_time_s: float, priority: int = 1, name = None):
        super().__init__(priority, name)
        self.wait_time_s = wait_time_s
        self.start_time = None

    def start(self, timestamp: float):
        self.start_time = timestamp

    def isFinished(self, timestamp: float) -> bool:
        if self.start_time is None:
            return False
        return (self.start_time + self.wait_time_s) <= timestamp

class SequentialTask(Task):
    def __init__(self, tasks, priority: int = 1, name = None):
        super().__init__(priority, name)
        self.subtasks = []
        self.current_task_index = 0
        for task in tasks:
            self.subtasks.append(task)
            self.resources.update(task.resources)

    def start(self, timestamp: float):
        if self.subtasks:
            self.subtasks[0].start(timestamp)

    def execute(self, timestamp: float):
        if self.current_task_index < len(self.subtasks):
            current_task = self.subtasks[self.current_task_index]
            current_task.execute(timestamp)
            if current_task.isFinished(timestamp):
                current_task.end(False, timestamp)
                self.current_task_index += 1
                if self.current_task_index < len(self.subtasks):
                    self.subtasks[self.current_task_index].start(timestamp)

    def isFinished(self, timestamp) -> bool:
        return self.current_task_index >= len(self.subtasks)

    def end(self, canceled: bool, timestamp: float):
        if canceled and self.current_task_index < len(self.subtasks):
            current_task = self.subtasks[self.current_task_index]
            current_task.end(True, timestamp)

class ParallelTask(Task):
    def __init__(self, tasks, priority: int = 1, name = None):
        super().__init__(priority, name)
        self.subtasks = set()
        self.running_tasks = set()
        for task in tasks:
            self.running_tasks.add(task)
            self.subtasks.add(task)
            if task.resources in self.resources:
                raise ValueError("Resource conflict in ParallelTask")
            self.resources.update(task.resources)

    def start(self, timestamp: float):
        for task in self.running_tasks:
            task.start(timestamp)

    def execute(self, timestamp: float):
        for task in self.running_tasks:
            if not task.isFinished(timestamp):
                task.execute(timestamp)
            
            if task.isFinished(timestamp):
                self.running_tasks.remove(task)

        for task in self.subtasks - self.running_tasks:
            task.end(False, timestamp)

    def isFinished(self, timestamp: float) -> bool:
        return all(task.isFinished(timestamp) for task in self.subtasks)

    def end(self, canceled: bool, timestamp: float):
        for task in self.running_tasks:
            if not task.isFinished(timestamp):
                task.end(canceled, timestamp)
            else:
                task.end(False, timestamp)