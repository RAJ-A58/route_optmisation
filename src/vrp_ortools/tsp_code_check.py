import json
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


def load_data():
    # Load data from JSON file
    with open("data/synthetic/vrp_6_nodes.json", "r") as f:
        data = json.load(f)

    return data["distance_matrix"]
    #Return only the distance matrix


def print_solution(manager, routing, solution):
   # Print the solution of the routing problem
    index = routing.Start(0)
    route = []
    total_distance = 0

    while not routing.IsEnd(index):
        node = manager.IndexToNode(index)
        route.append(node)

        previous_index = index
        index = solution.Value(routing.NextVar(index))
        total_distance += routing.GetArcCostForVehicle(
            previous_index, index, 0
        )
        print("Checking arc costs:")  # Find the distance between each node
        print(f"{manager.IndexToNode(previous_index)} -> {manager.IndexToNode(index)} : "
        f"{routing.GetArcCostForVehicle(previous_index, index, 0)}")

    route.append(manager.IndexToNode(index))
    print("Route:")
    print(" -> ".join(map(str, route)))
    print("Total distance:", total_distance)


def solve_vrp():
    """
    Solves a basic VRP with 1 vehicle using OR-Tools.
    """
    distance_matrix = load_data()

    num_locations = len(distance_matrix)
    num_vehicles = 1
    depot = 0

    # Create routing index manager
    manager = pywrapcp.RoutingIndexManager(
        num_locations,
        num_vehicles,
        depot
    )

    # Create routing model
    routing = pywrapcp.RoutingModel(manager)

    # Distance callback
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(distance_matrix[from_node][to_node])

    transit_callback_index = routing.RegisterTransitCallback(
        distance_callback
    )

    # Set cost of travel
    routing.SetArcCostEvaluatorOfAllVehicles(
        transit_callback_index
    )

    # Search parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    # Solve
    solution = routing.SolveWithParameters(search_parameters)

    if solution:
        print_solution(manager, routing, solution)
    else:
        print("No solution found")


if __name__ == "__main__":
    solve_vrp()
