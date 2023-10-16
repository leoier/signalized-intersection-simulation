class Car:

    p_brake = 0.1
    p_left = 0.15
    p_right = 0.15

    def __init__(self, velocity, position, turn):
        self.velocity = velocity
        self.position = position    # position on the lane
        self.turn = turn    # 0 = straight, -1 = left, 1 = right

    
    def __repr__(self):
        return '(Car: velocity = {}, position = {}, turn = {})'.format(self.velocity, self.position, self.turn)
