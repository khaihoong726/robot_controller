class Encoder:
    def __init__(self, ticks_per_rev):
        self.delta = 0
        self.increment = 0
        self.prev_count = 0
        self.ticks_per_rev = ticks_per_rev
        self.direction = 0
        self._set_encoder_wrap(-32768, 32768)

    def _set_encoder_wrap(self, low, high):
        self.range = high - low + 1
        self.low_wrap = low + self.range * 0.3
        self.high_wrap = high + self.range * 0.7

    def update(self, current_count):
        if self.prev_count > self.high_wrap and current_count < self.low_wrap:
            self.increment = current_count + self.range - self.prev_count
        elif self.prev_count < self.low_wrap and current_count > self.high_wrap:
            self.increment = current_count - self.range - self.prev_count
        else:
            self.increment = current_count - self.prev_count

        self.delta += self.increment
        self.prev_count = current_count
