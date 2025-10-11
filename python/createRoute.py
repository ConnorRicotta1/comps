import xml.etree.ElementTree as ET
import xml.dom.minidom
import os


def createRoute():
    total_vehicles_per_hour = 1500
    penetration_rate_connected = 0.3  # 30% connected vehicles
    simulation_duration = 900  # seconds

    # Demand ratio between roads (e.g., 60% Road A, 40% Road B)
    road_a_ratio = 0.25
    road_b_ratio = 1 - road_a_ratio

    # Split total vehicles between roads
    vehicles_a = int(total_vehicles_per_hour * road_a_ratio)
    vehicles_b = total_vehicles_per_hour - vehicles_a

    # Split by vehicle type
    cv_a = int(vehicles_a * penetration_rate_connected)
    nc_a = vehicles_a - cv_a

    cv_b = int(vehicles_b * penetration_rate_connected)
    nc_b = vehicles_b - cv_b

    routes = ET.Element("routes")

    # Vehicle types
    ET.SubElement(routes, "vType", id="non_connected", accel="2.6", decel="4.5", maxSpeed="13.9", length="5")
    ET.SubElement(routes, "vType", id="connected", accel="2.6", decel="4.5", maxSpeed="13.9", length="5")

    # Routes (adjust edge names to match your network)
    ET.SubElement(routes, "route", id="rA", edges="north_in south_out")
    ET.SubElement(routes, "route", id="rB", edges="east_in west_out")

    # Road A flows
    ET.SubElement(routes, "flow", id="nc_flow_A", type="non_connected", route="rA",
                begin="0", end=str(simulation_duration),
                vehsPerHour=str(nc_a), randomDepart="true")
    
    #ET.SubElement(nc_flow_A, "stop", lane="east_in_0", endPos="95.0", duration="10")

    ET.SubElement(routes, "flow", id="cv_flow_A", type="connected", route="rA",
                begin="0", end=str(simulation_duration),
                vehsPerHour=str(cv_a), randomDepart="true")

    # Road B flows
    ET.SubElement(routes, "flow", id="nc_flow_B", type="non_connected", route="rB",
                begin="0", end=str(simulation_duration),
                vehsPerHour=str(nc_b), randomDepart="true")

    ET.SubElement(routes, "flow", id="cv_flow_B", type="connected", route="rB",
                begin="0", end=str(simulation_duration),
                vehsPerHour=str(cv_b), randomDepart="true")
                

    tree = ET.ElementTree(routes)
    ET.indent(tree, space="\t", level=0)
    tree.write("/home/connor/ecocar/comps/scenario/routes.rou.xml", encoding="utf-8", xml_declaration=True)





