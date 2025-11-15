class SortedList:
    def __init__(self):
        self._data = []
    
    def _find_insertion_index(self, item):
        data = self._data
        lo, hi = 0, len(data)
        while lo < hi:
            mid = (lo + hi) // 2
            if item[0] < data[mid][0]:
                hi = mid
            else:
                lo = mid + 1
        return lo
    
    def _find_item_index(self, item):
        data = self._data
        priority = item[0]
        lo, hi = 0, len(data)
        
        while lo < hi:
            mid = (lo + hi) // 2
            if priority <= data[mid][0]:
                hi = mid
            else:
                lo = mid + 1
        
        start = lo
        while start < len(data) and data[start][0] == priority:
            if data[start] == item:
                return start
            start += 1
        
        return None
    
    def insert(self, value, priority):
        item = (priority, value)
        index = self._find_insertion_index(item)
        self._data.insert(index, item)
    
    def remove(self, value, priority):
        item = (priority, value)
        index = self._find_item_index(item)
        
        if index is not None:
            del self._data[index]
            return True
        else:
            print("Error removing item: ({}, {}) not in list".format(value, priority))
            return False
    
    def contains_item(self, value, priority):
        target_item = (priority, value)
        index = self._find_item_index(target_item)
        return index is not None
    
    def peek(self):
        if self._data:
            return self._data[0][1]
        return None
    
    def pop(self):
        if self._data:
            item = self._data.pop(0)
            return item[1]
        return None
    
    def clear(self):
        self._data = []
    
    def __iter__(self):
        for _, value in self._data:
            yield value
    
    def __getitem__(self, index):
        return self._data[index]
    
    def __len__(self):
        return len(self._data)
    
    def __repr__(self):
        return "SortedList({})".format(self._data)
    
    def __str__(self):
        if not self._data:
            return "SortedList(empty)"
        items = []
        for priority, value in self._data:
            items.append("({}: {})".format(priority, value))
        return "SortedList[{}]".format(", ".join(items))


if __name__ == "__main__":
    values = SortedList()
    
    values.insert("3", 2)
    values.insert("4", 5)
    values.insert("2", 1)
    values.insert("1", 0)
    values.insert("duplicate", 2)
    values.insert("also2", 2)
    
    print("Initial list contents (by value):")
    for value in values:
        print(value)
    
    print("\nFull list representation:")
    print(values)
    
    values.remove("2", 1)
    print("\nAfter removal of ('2', 1):")
    for value in values:
        print(value)
    
    print("\nContains ('3', 2)? {}".format(values.contains_item('3', 2)))
    print("Contains ('2', 1)? {}".format(values.contains_item('2', 1)))
    print("Contains ('duplicate', 2)? {}".format(values.contains_item('duplicate', 2)))
    
    values.remove("nonexistent", 99)
    
    print("\nPeek at lowest priority item: {}".format(values.peek()))
    print("Pop lowest priority item: {}".format(values.pop()))
    print("After pop:")
    for value in values:
        print(value)
    
    print("\nTesting duplicate priorities:")
    dup_list = SortedList()
    dup_list.insert("first", 1)
    dup_list.insert("second", 1)
    dup_list.insert("third", 1)
    print(dup_list)
    
    print("Contains ('second', 1)? {}".format(dup_list.contains_item('second', 1)))
    dup_list.remove("second", 1)
    print("After removing 'second':")
    print(dup_list)