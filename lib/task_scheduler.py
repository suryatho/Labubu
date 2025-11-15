import heapq

from lib.task import Task
from lib.sorted_list import SortedList

class _Resource():
    def __init__(self, name):
        self.name = name
    
class TaskScheduler():
    def __init__(self):
        self.resources = set()
        self.scheduled_resources = {}

        self.scheduled_tasks = SortedList()
        self.tasks_to_start = []
        self.tasks_to_cancel = []
        self.tasks_to_end = []
        self.timestamp = 0
        
    def create_resource(self, name):
        resource = _Resource(name)
        self.resources.add(resource)

    def run(self, timestamp: float):
        self.timestamp = timestamp

        # Cancel tasks
        while self.tasks_to_cancel:
            _, task = heapq.heappop(self.tasks_to_cancel)
            task.end(True, timestamp)
            self.scheduled_tasks.remove(task, task.priority)
            for resource in task.resources:
                self.scheduled_resources.pop(resource)

        # Start tasks
        while self.tasks_to_start:
            _, task = heapq.heappop(self.tasks_to_start)
            task.start(timestamp)

        # Run scheduled tasks
        for task in self.scheduled_tasks:
            task.execute(timestamp)
            if task.isFinished(timestamp):
                heapq.heappush(self.tasks_to_end, (task.priority, task))
            
        # End tasks
        while self.tasks_to_end:
            _, task = heapq.heappop(self.tasks_to_end)
            task.end(False, timestamp)
            self.scheduled_tasks.remove(task, task.priority)
            for resource in task.resources:
                self.scheduled_resources.pop(resource)

        pass

    def schedule(self, task):
        # Alraedy scheduled
        if self.scheduled_tasks.contains_item(task, task.priority):
            return
        
        conflicted_resources = task.resources & set(self.scheduled_resources.keys())
        if conflicted_resources:
            for resource in conflicted_resources:
                # Check if resource is scheduled
                if resource not in self.scheduled_resources:
                    continue
            
                # If there is a lower priority task scheduled, return
                scheduled_task = self.scheduled_resources[resource]
                if scheduled_task.priority < task.priority:
                    return
                
        for resource in conflicted_resources:
            print("Cancelling task due to resource conflict:", scheduled_task)
            self.cancel(scheduled_task)
        
        # Schedule new task
        self.scheduled_tasks.insert(task, task.priority)
        heapq.heappush(self.tasks_to_start, (task.priority, task))
        for resource in task.resources:
            self.scheduled_resources[resource] = task
            
    def cancel(self, task):
        heapq.heappush(self.tasks_to_cancel, (task.priority, task))
            
