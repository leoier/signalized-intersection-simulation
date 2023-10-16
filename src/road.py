import numpy as np
from car import Car
from traffic_signal import TrafficSignal

class Lane:
    def __init__(self, length, speed_limit, density, signal, 
                 lane_ahead, lane_left, lane_right, rng):
        self.length = length
        self.speed_limit = speed_limit
        self.density = density
        self.signal = signal
        self.lane_ahead = lane_ahead
        self.lane_left = lane_left
        self.lane_right = lane_right
        self.rng = rng
        self.cells = [None] * length    # the cells on the lane
        self.cars = set()               # set of cars on the lane
        self.car_exit = None   # the car that have exited the lane
        self.car_enter_queue = {}   # {car: remaining_time}

    '''
        generate a new car at the beginning of the lane
    '''
    def car_generator(self):
        if self.cells[0] is None \
           and self.rng.random() < self.rng.poisson(self.density):
            turn = 0
            p_turn = Car.p_left + Car.p_right
            if self.rng.random() < p_turn:
                turn = -1 if self.rng.random() < (Car.p_left / p_turn) else 1
            new_car = Car(1 + int(self.rng.random() * self.speed_limit), 0, turn)
            self.cells[0] = new_car
            self.cars.add(new_car)


    '''
        update the velocity of each car on the lane
    '''
    def update_velocity(self):
        for car in self.cars:
            # acceleration
            if car.velocity < self.speed_limit:
                car.velocity += 1

            # deceleration
            car.velocity = min(car.velocity, self.distance_ahead(car.position))

            if self.lane_ahead is not None:
                # also consider the distance on the lane ahead
                car.velocity = min(car.velocity, 
                                   self.length - 1 - car.position +
                                       self.lane_ahead.distance_ahead(-1))
                # cars stop at the intersection when red light or turning
                if self.signal.state == "red" or car.turn != 0:
                    car.velocity = min(car.velocity, self.length - car.position - 1)

            
            # randomization
            if car.position != 0 and self.rng.random() < Car.p_brake:
                car.velocity = max(car.velocity - 1, 0)



    '''
        update the position of each car on the lane
    '''
    def update_position(self):
        new_cells = [None] * self.length
        for car in self.cars.copy():
            new_position = car.position + car.velocity
            if new_position >= self.length:
                # if self.car_exit is not None:
                #     raise Exception("multiple cars exit the lane")
                self.car_exit = car
                self.cars.remove(car)
            else:
                new_cells[new_position] = car
                car.position = new_position
        self.cells = new_cells


    def update_enter_queue(self):
        for car in list(self.car_enter_queue.keys()):
            if self.car_enter_queue[car] == 0:
                # if self.cells[0] is not None:
                #     raise Exception("multiple cars enter the lane")
                self.car_enter_queue.pop(car)
                self.cells[0] = car
                self.cars.add(car)
                car.position = 0
                car.velocity = max(car.velocity, 1)
            else:
                self.car_enter_queue[car] -= 1
            


    '''
        return the distance to the car ahead of the car at car_index, and speed_limit
        if there is no car ahead
    '''
    def distance_ahead(self, position):
        for v in range(1, self.speed_limit + 1):
            if position + v >= self.length:
                return self.speed_limit
            elif self.cells[position + v] is not None:
                return v - 1
        return self.speed_limit




    '''
        determine if the position is safe to enter
    '''
    def position_safe_to_enter(self, position):
        # the cars behind position do not need to decelerate if a car enters
        for v in range(self.speed_limit + 1):
            if position - v >= 0 and self.cells[position - v] is not None \
               and self.cells[position - v].velocity >= v:
                return False

        # the cars ahead position are at least 1 cells away
        if position < self.length - 1 and self.cells[position + 1] is not None:
            return False

        return True



    '''
        return a copy of the lane with cells and cars copied
    '''
    def copy(self):
        new_lane = Lane(self.length, self.speed_limit, self.density, self.signal,
                        self.lane_ahead, self.lane_left, self.lane_right, self.rng)
        new_lane.cells = self.cells.copy()
        new_lane.cars = self.cars.copy()
        new_lane.car_exit = self.car_exit
        return new_lane


    def __repr__(self):
        return ' '.join([str(self.cells[i].velocity) 
                         if self.cells[i] is not None 
                         else '·' 
                         for i in range(self.length)])




class Road:
    '''
        Call init_lanes() after setting road_ahead
    '''
    def __init__(self, length, max_speed, density, signal,
                 road_ahead=None, road_left=None, road_right=None, 
                 road_opposite=None, name=None, start=False, 
                 rng=np.random):
        self.length = length
        self.max_speed = max_speed
        self.density = density
        self.signal = signal
        self.road_ahead = road_ahead
        self.road_left = road_left
        self.road_right = road_right
        self.road_opposite = road_opposite
        self.name = name
        self.start = start
        self.rng = rng
        self.lanes = []

    def init_lanes(self):
        self.lanes = [Lane(self.length, self.max_speed, self.density, self.signal,
                           self.road_ahead.lanes[i] 
                               if self.road_ahead is not None else None,
                           self.road_left.lanes[i] 
                               if self.road_left is not None else None,
                           self.road_right.lanes[i] 
                               if self.road_right is not None else None,
                           self.rng
                           )
                      for i in range(2)]
        

    def update_enter_queue(self):
        for lane in self.lanes:
            lane.update_enter_queue() 


    def update(self):
        if len(self.lanes) == 0:
            self.init_lanes()

        # update car velocity and position on each lane
        for lane in self.lanes:
            lane.update_velocity()
            lane.update_position()

        # move the cars to the enter queue of the road ahead
        for lane in self.lanes:
            if lane.car_exit is not None:
                if lane.lane_ahead is not None:
                    # print(f"{lane.car_exit} go straight")
                    lane.lane_ahead.car_enter_queue[lane.car_exit] = 0
                    lane.car_exit.position = -1
                    lane.car_exit = None
                else:
                    # print(f"{lane.car_exit} exit")
                    lane.car_exit = None


        # lane change
        # create a snapshot of the lanes to read from
        lanes_old = [lane.copy() for lane in self.lanes]

        for i, lane in enumerate(self.lanes):
            lane_old = lanes_old[i]
            adjacent_lane_old = lanes_old[1 - i]
            adjacent_lane = self.lanes[1 - i]
            for car in lane_old.cars:
                intend_to_change = (i == 1 and car.turn == -1) or \
                                   (i == 0 and car.turn == 1) or \
                                   (car.turn == 0 and 
                                    lane_old.distance_ahead(car.position) < self.max_speed and
                                    lane_old.distance_ahead(car.position) 
                                        < adjacent_lane_old.distance_ahead(car.position))
                # change the lane only if the opposite lane is safe to enter
                if intend_to_change and car.velocity > 0 and\
                   adjacent_lane_old.position_safe_to_enter(car.position):
                    # print(f"{car} {i} -> {1 - i}")
                    lane.cars.remove(car)
                    lane.cells[car.position] = None
                    adjacent_lane.cars.add(car)
                    adjacent_lane.cells[car.position] = car


        # turning
        for i, lane in enumerate(self.lanes):
            car_at_end = lane.cells[lane.length - 1]
            if car_at_end is not None and self.signal.state == 'green':
                if i == 0 and car_at_end.turn == -1 and \
                   self.safe_to_turn_left() and \
                   (lane.lane_left is None or (lane.lane_left.cells[0] is None 
                                               and len(lane.lane_left.car_enter_queue) < 2)):
                    # print(f"{car_at_end} turn left")
                    if lane.lane_left is not None:
                        lane.lane_left.car_enter_queue[car_at_end] = 2
                    lane.cells[lane.length - 1] = None
                    lane.cars.remove(car_at_end)
                    car_at_end.velocity = 1
                    car_at_end.turn = 0
                    car_at_end.position = -1
                elif i == 1 and car_at_end.turn == 1 and \
                     (lane.lane_right is None or lane.lane_right.cells[0] is None):
                    # print(f"{car_at_end} turn right")
                    if lane.lane_right is not None:
                        lane.lane_right.car_enter_queue[car_at_end] = 1
                    lane.cells[lane.length - 1] = None
                    lane.cars.remove(car_at_end)
                    car_at_end.velocity = 1
                    car_at_end.turn = 0
                    car_at_end.position = -1
                elif ((i == 0 and car_at_end.turn == 1) or (i == 1 and car_at_end.turn == -1)) and \
                     (lane.lane_ahead is None or lane.lane_ahead.cells[0] is None):
                    # print(f"{car_at_end} have to go straight")
                    if lane.lane_right is not None:
                        lane.lane_ahead.car_enter_queue[car_at_end] = 0
                    lane.cars.remove(car_at_end)
                    car_at_end.velocity = 1
                    car_at_end.turn = 0
                    car_at_end.position = -1



        # generate new cars
        if self.start:
            for lane in self.lanes:
                lane.car_generator()


    '''
        Return True if it is safe to turn left
        it is safe to turn only if no car on the opposite road will 
        arrive at the intersection in 2 time steps
    '''
    def safe_to_turn_left(self):
        if self.road_opposite is None:
            return True
        for lane in self.road_opposite.lanes:
            for v in range(self.max_speed * 2):
                if lane.cells[lane.length - 1 - v] is not None and \
                    lane.cells[lane.length - 1 - v].turn == 0 and \
                    lane.cells[lane.length - 1 - v].velocity * 2 >= v:
                    return False
        return True

    

if __name__ == '__main__':
    # unit test
    rng = np.random.default_rng(0)
    signal = TrafficSignal(30, 30, 'green')

    leng = 40
    vmax = 5
    dens = 0.5

    road_ahead = Road(leng, vmax, dens, signal, rng=rng)
    road_ahead.init_lanes()
    road_left = Road(leng, vmax, dens, signal, rng=rng)
    road_left.init_lanes()
    road_right = Road(leng, vmax, dens, signal, rng=rng)
    road_right.init_lanes()
    road_opposite = Road(leng, vmax, dens, signal, start=True, rng=rng)
    road_opposite.init_lanes()

    road = Road(leng, vmax, dens, signal, road_ahead=road_ahead, 
                road_left=road_left, road_right=road_right, 
                road_opposite=road_opposite, start=True, rng=rng)
    road.init_lanes()

    for i in range(100):
        road.update()
        road_ahead.update()
        road_opposite.update()
        road.update_enter_queue()
        road_ahead.update_enter_queue()
        road_left.update()
        road_left.update_enter_queue()
        road_right.update()
        road_right.update_enter_queue()
        signal.update()
        
        print(f"{road.lanes[0]} + {road_ahead.lanes[0]}")
        print(f"{road.lanes[1]} + {road_ahead.lanes[1]}")
        print(road_opposite.lanes[0])
        print(road_opposite.lanes[1])
        print(road_left.lanes[0])
        print(road_right.lanes[1])
        print(road.signal.state)
        print()