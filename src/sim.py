from road import Road
from traffic_signal import TrafficSignal
import numpy as np
import os
import time as tm
from statistics import mean 


def simulate(length, speed_limit, irwin_east_density, irwin_west_density,
             blvd_south_density, blvd_north_density, blvd_green_time, 
             init_time, simulation_time, rng):
    
    '''
        Helper Functions
    '''
    def sim_update():
        for road in all_roads:
            road.update()
        for road in all_roads:
            road.update_enter_queue()
        signal_blvd.update()
        signal_irwin.update()
        # if signal_irwin.state == 'green' and signal_blvd.state == 'green':
        #     raise Exception('Error: Both signals are green')


    def way_string(way):
        ret = [""] * len(way[0].lanes)
        for i in range(len(way[0].lanes)):
            ret[i] += f"{way[0].lanes[i]} **** {way[1].lanes[i]}\n"
        return ret
    

    def way_vector(way):
        ret = [[] for _ in range(len(way[0].lanes))]
        for i in range(len(way[0].lanes)):
            ret.append(way[0].lanes[i].cells[i].velocity if
                       way[0].lanes[i].cells[i] is not None else -1  
                       for i in range(way[0].lanes[i].length))
            ret += [-2] * 4
            ret.append(way[1].lanes[i].cells[i].velocity if
                       way[1].lanes[i].cells[i] is not None else -1  
                       for i in range(way[1].lanes[i].length))
        return ret


    def flow_way(way, before_intersection=True):
        cnt = []
        dist = []
        way_idx = [0] if before_intersection else [0, 1]
        for i in way_idx:
            for lane in way[i].lanes:
                cnt_cur = 0
                dist_cur = 0
                for i in way_idx:
                    cnt_cur += len(lane.cars)
                    dist_cur += lane.length
                cnt.append(cnt_cur)
                dist.append(dist_cur)
        return cnt, dist


    def density_way(way, before_intersection=True, all=False):
        cnt, dist = flow_way(way, before_intersection)
        if all:
            return sum(cnt) / (dist)
        return [cnt[i] / dist[i] for i in range(len(cnt))]
    

    def density_intersect(ways, before_intersection=True):
        cnt = 0
        dist = 0
        for way in ways:
            way_cnt, way_dist = flow_way(way, before_intersection)
            cnt += sum(way_cnt)
            dist += sum(way_dist)
        return cnt / dist


    def queue_length(way):
        cnt = [0, 0]
        for i in range(2):
            lane = way[0].lanes[i]
            for d in range(1, lane.length):
                if lane.cells[lane.length - d - 1] is None or \
                   lane.cells[lane.length - d - 1].velocity > 0:
                    break
                cnt[i] += 1
        return cnt
    

    def avg_speed_way(way, before_intersection=True, all=False):
        spd = [0, 0]
        cnt = [0, 0]
        way_idx = [0] if before_intersection else [0, 1]
        for i in way_idx:
            for l, lane in enumerate(way[i].lanes):
                curr_spd = 0
                curr_cnt = 0
                for car in lane.cars:
                    curr_spd += car.velocity
                    curr_cnt += 1
                spd[l] += curr_spd
                cnt[l] += curr_cnt
        if all:
            return sum(spd) / sum(cnt)
        return [(spd[i] / cnt[i]) if cnt[i] > 0 else 0 for i in range(len(spd))]


    def avg_speed_intersect(ways, before_intersection=True):
        sum = 0
        cnt = 0
        way_idx = [0] if before_intersection else [0, 1]
        for way in ways:
            for i in way_idx:
                for lane in way[i].lanes:
                    for car in lane.cars:
                        sum += car.velocity
                        cnt += 1
        return sum / cnt
    

    def delay_way(way, all=False):
        cnt = [0, 0]
        for i in range(2):
            lane = way[0].lanes[i]
            for car in lane.cars:
                if car.velocity == 0:
                    cnt[i] += 1
        if all:
            return sum(cnt)
        return cnt
    
    

    '''
        Simulation
    '''
    # create the traffic signals
    signal_blvd = TrafficSignal(blvd_green_time, blvd_red_time, 'green')
    signal_irwin = TrafficSignal(irwin_green_time, irwin_red_time, 'red')

    # create and connect the roads for the intersection
    irwin_west = [Road(length, speed_limit, irwin_west_density, signal_irwin, 
                       name="Irwin St NE - Westbound", rng=rng) 
                  for _ in range(2)]
    irwin_east = [Road(length, speed_limit, irwin_east_density, signal_irwin, 
                       name="Irwin St NE - Eastbound", rng=rng) 
                  for _ in range(2)]
    blvd_north = [Road(length, speed_limit, blvd_north_density, signal_blvd,
                       name="Boulevard NE - Northbound", rng=rng) 
                  for _ in range(2)]
    blvd_south = [Road(length, speed_limit, blvd_south_density, signal_blvd,
                       name="Boulevard NE - Southbound", rng=rng) 
                  for _ in range(2)]
    irwin_west[0].start = True
    irwin_east[0].start = True
    blvd_north[0].start = True
    blvd_south[0].start = True

    irwin_west[0].road_ahead = irwin_west[1]
    irwin_east[0].road_ahead = irwin_east[1]
    blvd_north[0].road_ahead = blvd_north[1]
    blvd_south[0].road_ahead = blvd_south[1]

    irwin_east[0].road_left = blvd_north[1]
    irwin_east[0].road_right = blvd_south[1]
    irwin_west[0].road_left = blvd_south[1]
    irwin_west[0].road_right = blvd_north[1]
    blvd_south[0].road_left = irwin_east[1]
    blvd_south[0].road_right = irwin_west[1]
    blvd_north[0].road_left = irwin_west[1]
    blvd_north[0].road_right = irwin_east[1]

    irwin_east[0].road_opposite = irwin_west[0]
    irwin_west[0].road_opposite = irwin_east[0]
    blvd_south[0].road_opposite = blvd_north[0]
    blvd_north[0].road_opposite = blvd_south[0]

    all_ways = {"irwin_west": irwin_west, "irwin_east": irwin_east, 
                "blvd_north": blvd_north, "blvd_south": blvd_south}
    all_roads = sum(all_ways.values(), [])

    # Initialize the lanes
    for i in range(1, -1, -1):
        for way in all_ways.values():
            way[i].init_lanes()

    output = {}

    output["signal"] = {way: [] for way in all_ways}
    output["space_time_mat"] = {way: [[] for _ in range(2)] for way in all_ways}
    output["space_time_str"] = {way: [[] for _ in range(2)] for way in all_ways}
    output["density"] = {"all": []}
    output["avg_speed"] = {"all": []}
    output["queue_length"] = {"all": []}
    output["delay"] = {"all": []}
    for way in all_ways:
        output["density"][way] = [[] for _ in range(2)]
        output["queue_length"][way] = [[] for _ in range(2)]
        output["avg_speed"][way] = [[] for _ in range(2)]
        output["delay"][way] = [[] for _ in range(2)]

    # Initialize the empty roads
    for t in range(init_time):
        sim_update()

    # Run the simulation
    for t in range(simulation_time):
        sim_update()

        # write the output
        for way in all_ways:
            way_vec = way_vector(all_ways[way])
            way_str = way_string(all_ways[way])
            output["signal"][way].append(all_ways[way][0].signal.state)
            for l in range(2):
                output["space_time_mat"][way][l].append(way_vec[l])
                output["space_time_str"][way][l].append(way_str[l])
        
        max_queue_length = [0, 0]
        sum_delay = [0, 0]
        
        for way in all_ways:
            curr_density = density_way(all_ways[way])
            curr_avg_speed = avg_speed_way(all_ways[way])
            curr_queue_length = queue_length(all_ways[way])
            curr_delay = delay_way(all_ways[way])
            for l in range(2):
                output["density"][way][l].append(curr_density[l])
                output["avg_speed"][way][l].append(curr_avg_speed[l])
                max_queue_length[l] = max(max_queue_length[l], curr_queue_length[l])
                sum_delay[l] += curr_delay[l]
                output["queue_length"][way][l].append(curr_queue_length[l])
                output["delay"][way][l].append(curr_delay[l])
                

        output["density"]["all"].append(density_intersect(all_ways.values()))
        output["avg_speed"]["all"].append(avg_speed_intersect(all_ways.values()))
        output["queue_length"]["all"].append(max(max_queue_length))
        output["delay"]["all"].append(sum(sum_delay))

    return output


'''
    Simulation of the intersection at Irwin St NE and Boulevard NE
'''
if __name__ == "__main__":
    rng = np.random.default_rng(32)

    # parameters
    length = 70
    speed_limit = 3
    # data source: https://gdottrafficdata.drakewell.com/tfdayreport.asp?node=GDOT_PORTABLES&cosit=0000121_6220&reportdate=2020-10-28&enddate=2020-10-28
    # data source: https://gdottrafficdata.drakewell.com/tfdayreport.asp?node=GDOT_PORTABLES&cosit=0000121_5594&reportdate=2021-02-03&enddate=2021-02-03
    flow_real = {"8am": {"irwin_east": 0.1025, "irwin_west": 0.0181, "blvd_south": 0.1225, "blvd_north": 0.17083}, 
                 "noon": {"irwin_east": 0.0617, "irwin_west": 0.0375, "blvd_south": 0.1825, "blvd_north": 0.1733},
                 "5pm": {"irwin_east": 0.0812, "irwin_west": 0.0533, "blvd_south": 0.3258, "blvd_north": 0.2139}
                }

    if not os.path.exists("output"):
        os.makedirs("output")
    os.chdir("output")

    for blvd_green_time in range(30, 56):

        start_time = tm.time()

        blvd_red_time = 60 - blvd_green_time
        irwin_green_time = blvd_red_time
        irwin_red_time = blvd_green_time

        # simulation parameters
        init_time = 600
        simulation_time = 3600

        output = {}
        
        for time in flow_real.keys():
            output[time] = simulate(length, speed_limit, 
                                    flow_real[time]["irwin_east"], 
                                    flow_real[time]["irwin_west"],
                                    flow_real[time]["blvd_south"], 
                                    flow_real[time]["blvd_north"], 
                                    blvd_green_time,
                                    init_time, simulation_time,
                                    rng)
        
        

        output_dir = "blvd_green_" + str(blvd_green_time)
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        os.chdir(output_dir)

        f = open("stats.csv", "w")
        f.write("time, location, density, avg_speed, max_queue_length, delay\n")
        for time in flow_real.keys():
            for way in output[time]["density"].keys():
                if way == "all":
                    continue
                density_lst = []
                avg_speed_lst = []
                queue_length_lst = []
                delay_lst = []
                for lane in range(2):
                    location = way + "_" + ("left" if lane == 0 else "right")
                    curr_density = mean(output[time]["density"][way][lane])
                    curr_avg_speed = mean(output[time]["avg_speed"][way][lane])
                    curr_queue_length = max(output[time]["queue_length"][way][lane])
                    curr_delay = mean(output[time]["delay"][way][lane])
                    density_lst.append(curr_density)
                    avg_speed_lst.append(curr_avg_speed)
                    queue_length_lst.append(curr_queue_length)
                    delay_lst.append(curr_delay)
                    f.write(f"{time}, {location}, {curr_density: .4f}, {curr_avg_speed: .4f}, {curr_queue_length}, {curr_delay}\n")
                f.write(f"{time}, {way}, {mean(density_lst): .4f}, {mean(avg_speed_lst): .4f}, {max(queue_length_lst)}, {mean(delay_lst)}\n")
        f.close()

        f = open("stats_overall.csv", "w")
        f.write("time, density, avg_speed, max_queue_length, delay\n")
        for time in flow_real.keys():
            f.write(f"{time}, {mean(output[time]['density']['all']): .4f}, {mean(output[time]['avg_speed']['all']): .4f}, {max(output[time]['queue_length']['all'])}, {mean(output[time]['delay']['all'])}\n")

        if not os.path.exists("space_time"):
            os.makedirs("space_time")
        os.chdir("space_time")
        for time in flow_real.keys():
            for way in output[time]["space_time_str"].keys():
                for lane in range(2):
                    with open(f"space_time_{way}_{lane}_{time}.txt", "w") as f:
                        f.write("".join(output[time]["space_time_str"][way][lane]))
                        f.close()

        os.chdir("../../")
        
        print(f"Simulation done for blvd_green_time = {str(blvd_green_time)}, elapsed time = {tm.time() - start_time: .4f} seconds")
