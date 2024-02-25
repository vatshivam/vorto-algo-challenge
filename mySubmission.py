    # import argparse
    # # parser = argparse.ArgumentParser()
    # # parser.add_argument("path", help="enter path to problem")
    # # args = parser.parse_args()
    # # print(args.path)

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import numpy as np
from scipy.spatial import distance_matrix 
import os

    # os.environ['PYDEVD_DISABLE_FILE_VALIDATION'] = "1"
path = "./Training Problems"
os.listdir(path)
paths_to_problems = []
for file in os.listdir(path):
    paths_to_problems.append(path+'/'+file)
# replace paths_to_problems[0] with the argument value

pickups = []
drops = []
f = open(paths_to_problems[0], "r")
f.readline()
for x in f:
    pickup,drop = (x.split()[1:])
    pickup = list(eval(pickup))
    drop = list(eval(drop))
    pickups.append(pickup)
    drops.append(drop)

all_points = [[0,0]] + pickups + drops    

dist_matrix = distance_matrix(all_points, all_points, p=2).round().astype(int)
dist_matrix = dist_matrix.tolist()
pickup_deliveries = [[i,i+len(pickups)] for i in range(1,len(pickups)+1)]

# print((dist_matrix))
# print(pickup_deliveries)

    # # In each pair of pickup_deliveries, the first entry is the pickup location index and the second is dropoff index
def create_data_model():
    """Stores the data for the problem."""
    data = {}
    # print("pickups_deliveries",len(data['pickups_deliveries']))
    # print("distance_matrix",len(data['distance_matrix']))
    # print("my dist matrix length",len(dist_matrix))
    # print("my pikcup drop list length",len(pickup_deliveries))
    data["distance_matrix"] = dist_matrix
    # print(data['distance_matrix'])
    data["pickups_deliveries"] = pickup_deliveries
    data["num_vehicles"] = len(pickup_deliveries)
    data["depot"] = 0
    return data

def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    # print(f"Objective: {solution.ObjectiveValue()}")
    total_distance = 0
    for vehicle_id in range(data["num_vehicles"]):
        # print("vehivle_id",vehicle_id)
        index = routing.Start(vehicle_id)
        plan_output = f"Route for vehicle {vehicle_id}:\n"
        route_distance = 0
        nodes_out = []
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            if node>0 and node<=10:
                nodes_out.append(node)
            # plan_output += f" {node} -> "
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id
            )
            # print("distance travelled",route_distance)
        if route_distance == 0:
            continue
        route_distance -= 500
        # plan_output += f"{manager.IndexToNode(index)}\n"
        # plan_output += f"Distance of the route: {route_distance}m\n"
        # print(plan_output)
        # total_distance += route_distance
        if len(nodes_out)>1:
            print(nodes_out)
    # print(f"Total Distance of all routes: {total_distance}m")
    
def main():
    """Entry point of the program."""
    # Instantiate the data problem.
    data = create_data_model()
    # print(data)
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data["distance_matrix"]),
        data["num_vehicles"],
        data["depot"]
    )
    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Define cost of each arc.
    def distance_callback(from_index, to_index):
        """Returns the manhattan distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    
    #adding fixed cost of 500 for each driver
    routing.SetFixedCostOfAllVehicles(500)

    # Add Distance constraint.
    dimension_name = "Distance"
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        720,  # vehicle maximum travel distance
        False,  # start cumul to zero
        dimension_name,
    )
    
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(1000)

    # Define Transportation Requests.
    for request in data["pickups_deliveries"]:
        pickup_index = manager.NodeToIndex(request[0])
        delivery_index = manager.NodeToIndex(request[1])
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        routing.solver().Add(
            routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index)
        )
        routing.solver().Add(
            distance_dimension.CumulVar(pickup_index)
            <= distance_dimension.CumulVar(delivery_index)
        )

    #custom objective function
    # routing.SetObjective(500*routing.VehicleVarCount() + routing.GetGlobalSpanCostVar())
    
    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION
    )
    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)
    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)
if __name__ == "__main__":
    main()