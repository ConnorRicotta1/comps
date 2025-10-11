import math
import traci
from itertools import permutations
from collections import defaultdict

INTERSECTION_POS = (200, 200)  # coordinates of the junction
CONTROL_RADIUS = 100.0
MAX_N = 8  # maximum number of vehicles to control
SLOT_DURATION = 2 
S = 11.2 # junction length
V = 13.9 # assumed speed for crossing
A = 2.6 #accel of vehicle 
Sm = 750 # saturation flow


def distance(pos1, pos2):
    return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)


def get_controlled_vehicles():
    controlled = []
    for veh_id in traci.vehicle.getIDList():
        pos = traci.vehicle.getPosition(veh_id)
        speed = traci.vehicle.getSpeed(veh_id)
        lane = traci.vehicle.getLaneID(veh_id)
        dist = ((pos[0] - INTERSECTION_POS[0])**2 + (pos[1] - INTERSECTION_POS[1])**2)**0.5

        if lane == "north_in_0":
            lane_length = traci.lane.getLength("north_in_0")
            traci.vehicle.setStop(veh_id, edgeID="north_in", pos=lane_length - 0.1, duration=999.0)
        if lane == "east_in_0":
            # print(lane)
            lane_length = traci.lane.getLength("east_in_0")
            traci.vehicle.setStop(veh_id, edgeID="east_in", pos=lane_length - 0.1, duration=999.0)

        #print(f"{veh_id}: dist={dist:.2f}, speed={speed:.2f}")  # Debug line

        if (dist < CONTROL_RADIUS and (lane == "north_in_0" or lane == "east_in_0")):
            controlled.append((veh_id, speed, dist, lane))
        

    # Sort by distance to prioritize closest vehicles
    controlled.sort(key=lambda x: x[2])

    # Limit the number of vehicles returned
    controlled = controlled[:MAX_N]

    scheduled = []
    queued = []

    q_pos_n = 1
    q_pos_e = 1

    for i, (veh_id, speed, dist, lane) in enumerate(controlled):
        if lane ==  "north_in_0":
            q_pos = q_pos_n
            queued.append((veh_id, speed, dist, lane, q_pos))
            q_pos_n +=1
        else:
            q_pos = q_pos_e
            queued.append((veh_id, speed, dist, lane, q_pos))
            q_pos_e +=1


    for i, (veh_id, speed, dist, lane, q_pos) in enumerate(queued):

        eta = estimate_arrival_time(speed, dist, q_pos)
        scheduled.append((veh_id, eta, dist, lane, q_pos))

    scheduled.sort(key=lambda x: x[1])  # sort by ETA

    return scheduled


def estimate_arrival_time(speed, dist, queue_position):
    
    if speed > 1.5:
        return dist / speed
    else:
        # Estimate based on queue position and slot duration
        return queue_position * SLOT_DURATION
    
    
def getLaneQ(vehicles):
    lane_queues = defaultdict(list)

    for veh_id, eta, dist, lane, q_pos in vehicles:
        lane_queues[lane].append((veh_id, eta, dist, lane, q_pos))

    # Sort each lane's queue by queue position
    for lane in lane_queues:
        lane_queues[lane].sort(key=lambda x: x[4])  # q_pos

    return lane_queues


def interleave(seq1, seq2):
    if not seq1:
        return [seq2]
    if not seq2:
        return [seq1]

    results = []
    for rest in interleave(seq1[1:], seq2):
        results.append([seq1[0]] + rest)
    for rest in interleave(seq1, seq2[1:]):
        results.append([seq2[0]] + rest)
    return results

def interleave_all(sequences):
    if len(sequences) == 1:
        return [sequences[0]]
    if len(sequences) == 2:
        return interleave(sequences[0], sequences[1])

    first = sequences[0]
    rest = interleave_all(sequences[1:])
    results = []
    for r in rest:
        results.extend(interleave(first, r))
    return results

def penalty(vehicle):
    Pck = max(S/V, (-A*(1/Sm)*((vehicle[4] - 1)*5) + math.sqrt((A*(1/Sm)*(vehicle[4] - 1)*5)**2 + 2*A*S) / A))
    return Pck

def delayCost(vehicle, vehicle_index):
    Pck = penalty(vehicle)
    cost = max(vehicle[1], (vehicle_index - 1) + (1/Sm) + Pck)
    return cost

def totalDelay(costs):
    for cost in len(costs):
        total_delay = total_delay + (cost[0] - cost[1]) #if the max dc from previous step is determined as vc, this will equal zero and reduce the cost impact of that vehicle
    return total_delay

def processCombinations(combinations):
    costIndex = []
    
    for combo_index, combination in enumerate(combinations):
        #print(f"\n Combination {combo_index + 1}:")
        total_cost = 0

        for vehicle_index, vehicle in enumerate(combination):
            veh_id, eta, dist, lane, q_pos = vehicle

            # print(f"    Vehicle {vehicle_index + 1}:")
            # print(f"    ID: {veh_id}")
            # print(f"    ETA: {eta}")
            # print(f"    Distance to junction: {dist}")
            # print(f"    Lane ID: {lane}")
            # print(f"    Lane Position: {q_pos}")
            cost = delayCost(vehicle, vehicle_index)
            # print(f"    cost: {cost}")
            total_cost = total_cost + cost

        costIndex.append(total_cost)
    #print(f"    cost index: {costIndex}")
    return costIndex