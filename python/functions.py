import math
import traci
from itertools import permutations
from collections import defaultdict

INTERSECTION_POS = (200, 200)  # coordinates of the junction
CONTROL_RADIUS = 200.0
MAX_N = 8  # maximum number of vehicles to control
SLOT_DURATION = 2 
S = 11.2 # junction length
V = 13.9 # assumed speed for crossing
A = 2.6 #accel of vehicle 
Sm_n = 400 # saturation flow
Sm_e = 1200 # saturation flow
Sm_s = 400 # saturation flow
Sm_w = 1200 # saturation flow

distThresh = 1


vehicle_spacing = 6.0  # meters per vehicle
default_speed = 13.89  # m/s (~50 km/h)

# Define vph for each lane
vph_north = 900
vph_east = 900

# Convert to arrival rate
arrival_rate_north = vph_north / 3600
arrival_rate_east = vph_east / 3600

# Get traffic light state


# Lane index mapping (adjust if needed)
lane_indices = {
    "north_in_0": 0,
    "east_in_0": 1
}

def estimate_non_cvs(lane, arrival_rate, lane_index):
    estimated = []
    signal_state = traci.trafficlight.getRedYellowGreenState("n1")
    phase_duration = traci.trafficlight.getPhaseDuration("n1")
    if signal_state[lane_index] == 'r':
        # RED phase: estimate queued vehicles
        estimated_count = int(arrival_rate * phase_duration)
        for i in range(estimated_count):
            veh_id = f"nonCV_red_{lane}_{i}"
            current_time = traci.simulation.getTime()
            eta = current_time + i * 2  # staggered
            dist = CONTROL_RADIUS - i * vehicle_spacing
            q_pos = i + 1
            estimated.append((veh_id, eta, dist, lane, q_pos))    
    elif signal_state[lane_index] == 'G':
        # GREEN phase: estimate oncoming vehicles
        estimated_count = int(arrival_rate * phase_duration)
        for i in range(estimated_count):
            veh_id = f"nonCV_green_{lane}_{i}"
            dist = CONTROL_RADIUS + i * vehicle_spacing
            current_time = traci.simulation.getTime()
            eta = current_time + dist / default_speed
            q_pos = None 
            estimated.append((veh_id, eta, dist, lane, q_pos))
    return estimated


def get_controlled_vehicles():
    controlled = []
    for veh_id in traci.vehicle.getIDList():
        pos = traci.vehicle.getPosition(veh_id)
        speed = traci.vehicle.getSpeed(veh_id)
        lane = traci.vehicle.getLaneID(veh_id)
        dist = ((pos[0] - INTERSECTION_POS[0])**2 + (pos[1] - INTERSECTION_POS[1])**2)**0.5
        vehType = traci.vehicle.getTypeID(veh_id)
        loss =traci.vehicle.getTimeLoss(veh_id)
        # print(loss)
        #print(f"{veh_id}: dist={dist:.2f}, speed={speed:.2f}")  # Debug line

        if (dist < CONTROL_RADIUS and (lane == "north_in_0" or lane == "east_in_0" or lane == "south_in_0" or lane == "west_in_0") and vehType == "connected"):
            controlled.append((veh_id, speed, dist, lane))
        
    # Sort by distance to prioritize closest vehicles
    controlled.sort(key=lambda x: x[2])

    # Limit the number of vehicles returned
    controlled = controlled[:MAX_N + 4]

    scheduled = []
    queued = []

    q_pos_n = 1
    q_pos_e = 1
    q_pos_s = 1
    q_pos_w = 1

    for i, (veh_id, speed, dist, lane) in enumerate(controlled):
        if lane ==  "north_in_0":
            q_pos = q_pos_n
            queued.append((veh_id, speed, dist, lane, q_pos))
            q_pos_n +=1
        elif lane ==  "east_in_0":
            q_pos = q_pos_e
            queued.append((veh_id, speed, dist, lane, q_pos))
            q_pos_e +=1
        elif lane ==  "south_in_0":
            q_pos = q_pos_s
            queued.append((veh_id, speed, dist, lane, q_pos))
            q_pos_s +=1
        elif lane ==  "west_in_0":
            q_pos = q_pos_w
            queued.append((veh_id, speed, dist, lane, q_pos))
            q_pos_w +=1


    for i, (veh_id, speed, dist, lane, q_pos) in enumerate(queued):

        eta = estimate_arrival_time(speed, dist, q_pos)
        scheduled.append((veh_id, eta, dist, lane, q_pos))

    # if len(scheduled) == 0 and traci.simulation.getTime() > 3:
    #     scheduled += estimate_non_cvs("north_in_0", arrival_rate_north, lane_indices["north_in_0"])
    #     scheduled += estimate_non_cvs("east_in_0", arrival_rate_east, lane_indices["east_in_0"])

    scheduled.sort(key=lambda x: x[1])  # sort by ETA


        



    #!!!!!! after sorting by eta, delete 1 of the vehicle in the same lane that have a similar eta to treat 
    #them as the same vehicle (this should be the closest one so the slightly further is left)

    # print(scheduled)

    # penetration_rate = 0.9
    # non_cv_ratio = (1 - penetration_rate) / penetration_rate
    # non_cv_counter = 0

    # lane_groups = defaultdict(list)
    # for veh_id, eta, dist, lane, q_pos in scheduled:
    #     lane_groups[lane].append((veh_id, eta, dist, q_pos))

    # for lane, vehicles in lane_groups.items():
    #     vehicles.sort(key=lambda x: x[3])  # sort by queue position

    #     # 1. Ahead of first CV
    #     first_pos = vehicles[0][3]
    #     estimated_ahead = int((first_pos - 1) * non_cv_ratio)
    #     for j in range(estimated_ahead):
    #         non_cv_id = f"nonCV_{non_cv_counter}"
    #         non_cv_counter += 1
    #         est_q_pos = j + 1
    #         est_dist = vehicles[0][2] + 6 * (first_pos - est_q_pos)  # assume 6m spacing
    #         est_eta = vehicles[0][1] + 2 + j  # staggered buffer
    #         scheduled.append((non_cv_id, est_eta, est_dist, lane, est_q_pos))

    #     # 2. Between CVs
    #     for i in range(len(vehicles) - 1):
    #         curr_pos = vehicles[i][3]
    #         next_pos = vehicles[i + 1][3]
    #         gap = next_pos - curr_pos - 1
    #         estimated_count = int(gap * non_cv_ratio)

    #         for j in range(estimated_count):
    #             non_cv_id = f"nonCV_{non_cv_counter}"
    #             non_cv_counter += 1
    #             est_q_pos = curr_pos + j + 1
    #             est_dist = (vehicles[i][2] + vehicles[i + 1][2]) / 2
    #             est_eta = (vehicles[i][1] + vehicles[i + 1][1]) / 2 + 2
    #             scheduled.append((non_cv_id, est_eta, est_dist, lane, est_q_pos))

    #     # 3. Behind last CV
    #     last_pos = vehicles[-1][3]
    #     estimated_behind = int(3 * non_cv_ratio)  # assume 3 trailing positions
    #     for j in range(estimated_behind):
    #         non_cv_id = f"nonCV_{non_cv_counter}"
    #         non_cv_counter += 1
    #         est_q_pos = last_pos + j + 1
    #         est_dist = vehicles[-1][2] - 6 * (est_q_pos - last_pos)  # assume 6m spacing
    #         est_eta = vehicles[-1][1] + 2 + j
    #         scheduled.append((non_cv_id, est_eta, est_dist, lane, est_q_pos))

    #     scheduled.sort(key=lambda x: x[1])  # re-sort after adding non-CVs

    #     scheduled = scheduled[:MAX_N]

    # print(scheduled)

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

def deleteSimilar(laneQ):
    
    all_vehicles = []

    if "north_in_0" in laneQ and "south_in_0" in laneQ:

        i = 0
        while i < len(laneQ['north_in_0']):
            j = 0
            while j < len(laneQ['south_in_0']):

                dist_n = laneQ['north_in_0'][i][2]
                dist_s = laneQ['south_in_0'][j][2]

                if abs(dist_n - dist_s) <= distThresh:

                    if dist_n > dist_s:
                        # print(laneQ)
                        # print("north is further!!!!!!")

                        # remove south vehicle
                        laneQ['south_in_0'].pop(j)

                        # adjust q_pos for south lane
                        for k in range(j, len(laneQ['south_in_0'])):
                            veh_id, eta, dist, lane, q_pos = laneQ['south_in_0'][k]
                            laneQ['south_in_0'][k] = (veh_id, eta, dist, lane, q_pos - 1)
                        # print(laneQ)

                        # do NOT increment j here because list shrank
                        continue

                    else:
                        # print(laneQ)
                        # print("south is further!!!!!!")

                        # remove north vehicle
                        laneQ['north_in_0'].pop(i)

                        # adjust q_pos for south lane
                        for k in range(i, len(laneQ['north_in_0'])):
                            veh_id, eta, dist, lane, q_pos = laneQ['north_in_0'][k]
                            laneQ['north_in_0'][k] = (veh_id, eta, dist, lane, q_pos - 1)
                        # print(laneQ)
                        break

                j += 1
            i += 1
    


    if "west_in_0" in laneQ and "east_in_0" in laneQ:

        i = 0
        while i < len(laneQ['west_in_0']):
            j = 0
            while j < len(laneQ['east_in_0']):

                dist_n = laneQ['west_in_0'][i][2]
                dist_s = laneQ['east_in_0'][j][2]

                if abs(dist_n - dist_s) <= distThresh:

                    if dist_n > dist_s:
                        # print(laneQ)
                        # print("west is further!!!!!!")

                        # remove east vehicle
                        laneQ['east_in_0'].pop(j)

                        # adjust q_pos for east lane
                        for k in range(j, len(laneQ['east_in_0'])):
                            veh_id, eta, dist, lane, q_pos = laneQ['east_in_0'][k]
                            laneQ['east_in_0'][k] = (veh_id, eta, dist, lane, q_pos - 1)
                        # print(laneQ)

                        # do NOT increment j here because list shrank
                        continue

                    else:
                        # print(laneQ)
                        # print("east is further!!!!!!")

                        # remove west vehicle
                        laneQ['west_in_0'].pop(i)

                        # adjust q_pos for east lane
                        for k in range(i, len(laneQ['west_in_0'])):
                            veh_id, eta, dist, lane, q_pos = laneQ['west_in_0'][k]
                            laneQ['west_in_0'][k] = (veh_id, eta, dist, lane, q_pos - 1)
                        # print(laneQ)
                        break

                j += 1
            i += 1

    for lane_id in ("north_in_0", "south_in_0", "west_in_0", "east_in_0"):
        if lane_id in laneQ:
            all_vehicles.extend(laneQ[lane_id])
    # print(all_vehicles)
    all_vehicles.sort(key=lambda x: x[1])
    all_vehicles = all_vehicles[:MAX_N]

    finalQ = getLaneQ(all_vehicles)


    return finalQ



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

def penalty(Ock, Sm):
    Pck = max(S/V, ((-A*(1/Sm)*((Ock - 1))) + math.sqrt((A*(1/Sm)*(Ock - 1)**2) + 2*A*S)) / A)
    return Pck

def delayCost(vehicle, vehicle_index, Ock):

    if vehicle[3] == "north_in_0":
        Sm = Sm_n
    elif vehicle[3] == "east_in_0":
        Sm = Sm_e
    elif vehicle[3] == "south_in_0":
        Sm = Sm_s
    elif vehicle[3] == "west_in_0":
        Sm = Sm_w

    ideal_time = vehicle[2] / V 
    estimated_time = vehicle[1] 
    estimated_delay = estimated_time - ideal_time

    Pck = penalty(Ock, Sm)
    cost = max(vehicle[1], (vehicle_index - 1)*SLOT_DURATION + (1/Sm) + Pck - 3*estimated_delay)

    # print("cost")
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
        prevVeh = None
        Ock = 0
        SWc = 0
        for vehicle_index, vehicle in enumerate(combination):
            veh_id, eta, dist, lane, q_pos = vehicle

            if prevVeh == None:
                prevVeh = vehicle

            if vehicle[3] == prevVeh[3]:
                Ock += 1
            else:
                Ock = 1

            if Ock == 1:
                SWc += 1

            # print(f"    Ock: {Ock}")

            # if vehicle_index + 1 < len(combination):
            #     if vehicle[3] == combination[vehicle_index + 1][3]:
            #         SWc -= 1
            # print("here!!")
            # print(SWc)
            # print(f"    Vehicle {vehicle_index + 1}:")
            # print(f"    ID: {veh_id}")
            # print(f"    ETA: {eta}")
            # print(f"    Distance to junction: {dist}")
            # print(f"    Lane ID: {lane}")
            # print(f"    Lane Position: {q_pos}")
            cost = delayCost(vehicle, vehicle_index, Ock)
            # print(f"    cost: {cost}")
            total_cost = total_cost + cost

        costIndex.append(total_cost)
    #print(f"    cost index: {costIndex}")
    return costIndex