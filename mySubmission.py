# Even though this file produces final output as expected
# evaluateShared encouters an error while parsing STDOUT
# This is due the /r expression produced when splitting the STDOUT
# Line number 78 in modified evaluateShared.py replaces /r with empty string and the tests run as expected
import time
import argparse
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from scipy.spatial import distance_matrix 

def extract_pickups_dropoffs(path):
    
    """ Reads Problem txt file """
    
    pickups = []
    dropoffs = []
    f = open(path, "r")
    f.readline()
    for x in f:
        pickup,drop = (x.split()[1:])
        pickup = list(eval(pickup))
        drop = list(eval(drop))
        pickups.append(pickup)
        dropoffs.append(drop)
    f.close()
    return pickups,dropoffs

def create_data_model(pickup_deliveries,distance_matrix):
    
    """ Stores the data for the problem. """
    
    data = {}
    data["distance_matrix"] = distance_matrix
    data["pickups_deliveries"] = pickup_deliveries
    data["depot"] = 0
    
    # initializing number of drivers equal to the number of dropoffs
    data["num_vehicles"] = len(pickup_deliveries)
    
    # Setting demand for pickup locations equal to 2 and -2 for dropoffs.
    # This forces the driver to return to depot after every delivery as the vehicle capacity becomes 0 after a delivery is complete
    # Note: Value "2" could be replaced by an arbitrary number since the value doesn't have any significance
    data["vehicle_capacities"] = [2]*data['num_vehicles']
    data["demands"] = [0]+[2]*(len(pickup_deliveries))+[-2]*(len(pickup_deliveries))
    return data

def print_solution(data, manager, routing, solution):
    
    """Prints solution on console."""

    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        route_distance = 0
        nodes_output = []
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            
            # appending the load_id to the output array when the delivery is complete.
            # Delivery is considered to be complete if the load_id is greater than the number of pickup-dropoff pairs.
            # This is because, pickups and dropoffs are ordered like [depot, pickup1, pickup2, pickup_n, dropoff1, dropoff2, dropoffn]
            if node>len(pickup_deliveries):
                nodes_output.append(node-len(pickup_deliveries))
            
            # moving the pointer to next node
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            
            # adding transit distance to total route distance 
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id
            )
        if route_distance == 0:
            continue
        if len(nodes_output)>=1:
            print(nodes_output)
            
def find_solution(pickup_deliveries,dist_matrix,max_distance):
    
    """Finding solution for the routing problem"""
    
    # Instantiate the data problem.
    data = create_data_model(pickup_deliveries,dist_matrix)
    
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
        
        """ Returns the manhattan distance between the two nodes. """
        
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    def demand_callback(from_index):
        
        """ Returns the demand of the node. """
        
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data["demands"][from_node]
    
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    
    #adding fixed cost of 500 for each driver
    routing.SetFixedCostOfAllVehicles(500)

    # Add capacity constrain to force vehicles to come back to depot
    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data["vehicle_capacities"],  # vehicle maximum capacities
        True,  # start cumul to zero
        "Capacity",
    )
    
    # Add Distance constraint.
    dimension_name = "Distance"
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        max_distance,  # vehicle maximum travel distance
        False,  #  Setting the value of fix_start_cumul_to_zero to true will force the "cumul" variable of the start node of all vehicles to be equal to 0
        dimension_name,
    )
    
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Define Transportation Requests.
    for request in data["pickups_deliveries"]:
        pickup_index = manager.NodeToIndex(request[0])
        delivery_index = manager.NodeToIndex(request[1])
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        
        # Ensuring that same vehicle is used for the pickup-dropoff pair
        routing.solver().Add(
            routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index)
        )
        
        # Cumulative distance at pickup should be smaller that dropoff
        routing.solver().Add(
            distance_dimension.CumulVar(pickup_index)
            <= distance_dimension.CumulVar(delivery_index)
        )
        
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

# Parsing input problem path
parser = argparse.ArgumentParser()
parser.add_argument("path", help="enter path to problem")
args = parser.parse_args()
path = args.path
st = time.time()

# Extract pickup and dropoff coordinates from txt file
pickups,drops = extract_pickups_dropoffs(path)

# List of all points including depot.
all_points = [[0,0]] + pickups + drops

# calculate pairwise distances between all the pickup and dropoff points    
dist_matrix = distance_matrix(all_points, all_points, p=2).round().astype(int).tolist()

# In Each pair of pickup_deliveries, the first entry is the pickup location index and the second is dropoff index with respect to "all_points"
pickup_deliveries = [[i,i+len(pickups)] for i in range(1,len(pickups)+1)]
    
# Maximum dsitance for each driver
max_distance = 12*60 - 1

find_solution(pickup_deliveries,dist_matrix,max_distance)

et = time.time()
print('Execution time:', round((et-st)*1000,2), 'milliseconds')