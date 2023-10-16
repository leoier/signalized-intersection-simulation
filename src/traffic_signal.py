class TrafficSignal:

    def __init__(self, green_time, red_time, state='green'):
        self.green_time = green_time
        self.red_time = red_time
        self.buffer_time = 2
        self.time = 0
        self.state = 'red'
        if state == 'green':
            self.time = self.red_time + self.buffer_time

    def update(self):
        self.time += 1
        if self.state == 'green' and self.time > self.green_time:
            self.state = 'red'
            self.time = 1
        elif self.state == 'red' and self.time > self.red_time + self.buffer_time * 2:
            self.state = 'green'
            self.time = 1
    

if __name__ == '__main__':
    # unit test
    signal1 = TrafficSignal(30, 30, 'green')
    signal2 = TrafficSignal(30, 30, 'red')
    for i in range(100):
        signal1.update()
        signal2.update()
        print(signal1.state, signal2.state)