import traci
import xml.etree.ElementTree as ET

from createRoute import createRoute
from functions import *
minGreen = 5
timeStep = 0.5

# createRoute()

# Dictionary to track stops per vehicle
stop_counts = {}
# Dictionary to track whether a vehicle was previously stopped
was_stopped = {}

traci.start(["sumo-gui", "-c", "../scenario/intersection.sumocfg"])

while traci.simulation.getMinExpectedNumber() > 0:
    
    for veh_id in traci.vehicle.getIDList():
        speed = traci.vehicle.getSpeed(veh_id)

        # Initialize tracking if vehicle is new
        if veh_id not in stop_counts:
            stop_counts[veh_id] = 0
            was_stopped[veh_id] = False

        # Detect a new stop (transition from moving to stopped), need to remove this for the logic used by paper
        if speed == 0 and not was_stopped[veh_id]:
            stop_counts[veh_id] += 1
            was_stopped[veh_id] = True
        elif speed > 0:
            was_stopped[veh_id] = False


    scheduled = get_controlled_vehicles()

    # print(len(scheduled))
    

    queued = getLaneQ(scheduled)



    lane_sequences = list(queued.values())  # list of lane-wise vehicle lists
    if not lane_sequences:
        feasible_orders = []
    elif len(lane_sequences) == 1:
        feasible_orders = [lane_sequences[0]]
    else:
        feasible_orders = interleave_all(lane_sequences)

    #print(feasible_orders)
    # print(traci.trafficlight.getControlledLanes("n1"))

    costIndex = []
    costIndex = processCombinations(feasible_orders)
    if costIndex:
        min_index = costIndex.index(min(costIndex))
        #print(f"min index: {min_index}")
        controlCombination = feasible_orders[min_index]
        #print(f"controlCombination: {[c[3] for c in controlCombination]}")
        lane_length = traci.lane.getLength(controlCombination[0][3])


        # print(traci.vehicle.__dict__)
        # print(traci.vehicle.getRoute(controlCombination[0][0]))
        # print(controlCombination)

        # platoon = []
        # control_lane = controlCombination[0][3]

        # for i in range(len(controlCombination)):
        #     if controlCombination[i][3] == control_lane:
        #         platoon.append(controlCombination[i])
        #     else: 
        #         break

        # for i in range(len(platoon)):
        #     if platoon[i][3] == "east_in_0":
        #         traci.vehicle.setRoute(platoon[i][0], ["east_in","west_out"])
        #         traci.vehicle.setSpeed(platoon[i][0], -1)
        #         traci.vehicle.setStop(platoon[i][0], edgeID="west_out", pos=190, duration=0)
        #         if traci.vehicle.isStopped(platoon[i][0]):
        #             traci.vehicle.resume(platoon[i][0])

        #     elif platoon[i][3] == "north_in_0":
        #         traci.vehicle.setRoute(platoon[i][0], ["north_in","south_out"])
        #         traci.vehicle.setSpeed(platoon[i][0], -1)
        #         traci.vehicle.setStop(platoon[i][0], edgeID="south_out", pos=190, duration=0)
        #         if traci.vehicle.isStopped(platoon[i][0]):
        #             traci.vehicle.resume(platoon[i][0])
        #         # print(controlCombination[i])
        #     last = platoon[i]

        # while min(traci.vehicle.getPosition(last[0])) > 195:
        #     if traci.vehicle.isStopped(platoon[i][0]):
        #         traci.vehicle.resume(platoon[i][0])
        #     traci.simulationStep()
        # print('made it!')

        # print(controlCombination)
        # for i in range(len(controlCombination)): 
        #     if controlCombination[i][3] == "east_in_0":
        #         traci.trafficlight.setRedYellowGreenState("n1", "rG")

        #     elif controlCombination[i][3] == "north_in_0":
        #         traci.trafficlight.setRedYellowGreenState("n1", "Gr")
        #         # print(controlCombination[i])

        #     # green_start = traci.simulation.getTime()
        #     # while traci.simulation.getTime() - green_start < minGreen:
        #     #     traci.simulationStep()

        #     try:
        #         while min(traci.vehicle.getPosition(controlCombination[i][0])) > 195:
        #             # if traci.vehicle.isStopped(controlCombination[i][0]):
        #             #     traci.vehicle.resume(controlCombination[i][0])
        #             traci.simulationStep()
        #     # print('made it!')
        #     except:
        #         pass
        # if len(controlCombination) < 1:
        #     traci.trafficlight.setRedYellowGreenState("n1", "rG")


    traci.simulationStep()



        #print(lane_length)
        # traci.vehicle.setStop(controlCombination[0][0], edgeID=controlCombination[0][3][:-2], pos=lane_length - 0.1, duration=0)



        

traci.close()

# Load tripinfo.xml
tree = ET.parse("../scenario/tripinfo.xml")
root = tree.getroot()

# Extract timeLoss for each vehicle
delays = [float(trip.attrib["timeLoss"]) for trip in root.findall("tripinfo")]

# Compute average delay
if delays:
    avg_delay = sum(delays) / len(delays)
    print(f"Average delay per car: {round(avg_delay, 2)} seconds")
else:
    print("No vehicles recorded in tripinfo.xml")

# Compute average stops per car
if stop_counts:
    avg_stops = sum(stop_counts.values()) / len(stop_counts)
    print(f"Average stops per car: {round(avg_stops, 2)}")
else:
    print("No stop data recorded.")




