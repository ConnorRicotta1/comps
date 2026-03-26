import traci
import xml.etree.ElementTree as ET

import re

from createRoute import createRoute
from functions import *
from statistics import mean


minGreen = 2
timeStep = 0.5
iterations = 5

# createRoute()

# post processing statistics array


avgStats = []




for i in range(iterations):

    with open("stats.log", "w"):
        pass    

    stats = {}
    # Dictionary to track stops per vehicle
    stop_counts = {}
    # Dictionary to track whether a vehicle was previously stopped
    was_stopped = {}
    # , "--end", "900"

    traci.start(["sumo", "-c", "../twoWay/intersection.sumocfg", "--seed", "-1", "--random"])

    END = 700

    while traci.simulation.getTime() < END:
        
        for veh_id in traci.vehicle.getIDList():
            speed = traci.vehicle.getSpeed(veh_id)

            # Initialize tracking if vehicle is new
            if veh_id not in stop_counts:
                stop_counts[veh_id] = 0
                was_stopped[veh_id] = False

            # Detect a new stop (transition from moving to stopped), need to remove this for the logic used by paper
            if speed <= 0.1 and not was_stopped[veh_id]:
                stop_counts[veh_id] += 1
                was_stopped[veh_id] = True
            elif speed > 0:
                was_stopped[veh_id] = False

        scheduled = get_controlled_vehicles()

        # print(len(scheduled))
        firstQueued = getLaneQ(scheduled)

        # queued = deleteSimilar(firstQueued)

        # print(scheduled)

        lane_sequences = list(firstQueued.values())  # list of lane-wise vehicle lists
        if not lane_sequences:
            feasible_orders = []
        elif len(lane_sequences) == 1:
            feasible_orders = [lane_sequences[0]]
        else:
            feasible_orders = interleave_all(lane_sequences)

        #print(feasible_orders)
        #print(traci.trafficlight.getControlledLanes("n1"))

        costIndex = []
        costIndex = processCombinations(feasible_orders)
        # print(costIndex)
        if costIndex:
            min_index = costIndex.index(min(costIndex))
            # print(min_index)
            #print(f"min index: {min_index}")
            controlCombination = feasible_orders[min_index]
            #print(f"controlCombination: {[c[3] for c in controlCombination]}")
            lane_length = traci.lane.getLength(controlCombination[0][3])

            
            # print(controlCombination)
            for i in range(len(controlCombination)): 
                if controlCombination[i][3] == "east_in_0" or controlCombination[i][3] == "west_in_0":
                    traci.trafficlight.setRedYellowGreenState("n1", "rGrG")

                elif controlCombination[i][3] == "north_in_0" or controlCombination[i][3] == "south_in_0":
                    traci.trafficlight.setRedYellowGreenState("n1", "GrGr")
                    # print(controlCombination[i])
                # print("here?")
                green_start = traci.simulation.getTime()
                while traci.simulation.getTime() - green_start < minGreen:
                    traci.simulationStep()

                try:
                    while min(traci.vehicle.getPosition(controlCombination[i][0])) > 195:
                        # if traci.vehicle.isStopped(controlCombination[i][0]):
                        #     traci.vehicle.resume(controlCombination[i][0])
                        traci.simulationStep()
                # print('made it!')
                except:
                    pass
            if len(controlCombination) < 1:
                traci.trafficlight.setRedYellowGreenState("n1", "GrGr")
            # print("not oops?")


        traci.simulationStep()

    traci.close()

    # Load tripinfo.xml
    tree = ET.parse("../twoWay/tripinfo.xml")
    root = tree.getroot()

    # Extract timeLoss for each vehicle
    delays = [float(trip.attrib["timeLoss"]) for trip in root.findall("tripinfo")]

    # Compute average delay
    if delays:
        avg_delay = sum(delays) / len(delays)
        print(f"Average delay per car: {round(avg_delay, 2)}", flush=True)
    else:
        print("No vehicles recorded in tripinfo.xml")

    # Compute average stops per car
    if stop_counts:
        avg_stops = sum(stop_counts.values()) / len(stop_counts)
        print(f"Average stops per car: {round(avg_stops, 2)}", flush=True)
    else:
        print("No stop data recorded.")



            #print(lane_length)
            # traci.vehicle.setStop(controlCombination[0][0], edgeID=controlCombination[0][3][:-2], pos=lane_length - 0.1, duration=0)



            



    with open("stats.log") as f:
        for line in f:
            line = line.strip()

            if line.startswith("Real time factor:"):
                stats["rtf"] = float(line.split(":")[1])
            if line.startswith("Speed:"):
                stats["avg_speed"] = float(line.split(":")[1])
            if line.startswith("WaitingTime:"):
                stats["avg_waiting"] = float(line.split(":")[1])
            if line.startswith("TimeLoss:"):
                stats["avg_timeloss"] = float(line.split(":")[1])
            if line.startswith("DepartDelay:"):
                stats["avg_depart_delay"] = float(line.split(":")[1])
            if line.startswith("Average stops per car:"):
                stats["stops"] = float(line.split(":")[1])

    avgStats.append(stats)

averaged = {}

for key in avgStats[0].keys():
    values = [run[key] for run in avgStats]
    averaged[key] = mean(values)

# print(avgStats)
print(averaged)
print(len(avgStats))



