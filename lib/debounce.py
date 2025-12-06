class Debouncer:
    def __init__(self, initial_state, counts=10):
        self._curr = initial_state
        self._required_counts = counts
        self._curr_count = 0
        self._target = initial_state

    def reset(self, new_state):
        self._curr = new_state
        self._curr_count = 0
        self._target = self._curr

    def calculate(self, des_state):
        if des_state == self._curr:
            self._curr_count = 0
            self._target = des_state
        elif des_state == self._target:
            self._curr_count += 1

            if self._curr_count >= self._required_counts:
                self._curr = self._target
                self._curr_count = 0
        else:
            self._target = des_state
            self._curr_count = 1

        return self._curr
