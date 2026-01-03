import json
from ortools.constraint_solver import pywrapcp, routing_enums_pb2


def load_data(path):
    with open(path, "r") as f:
        return json.load(f)


def solve_cvrp(data):
    num_nodes = len(data["distance_matrix"])
    num_vehicles = data["num_vehicles"]
    depot = data["depot"]
    vehicle_capacities = data["vehicle_capacities"]

    manager = pywrapcp.RoutingIndexManager(
        num_nodes, num_vehicles, depot
    )

    routing = pywrapcp.RoutingModel(manager)

    # -------- Distance callback --------
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    transit_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_index)

    # -------- Demand callback --------
    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return data["demands"][from_node]

    demand_index = routing.RegisterUnaryTransitCallback(demand_callback)

    routing.AddDimensionWithVehicleCapacity(
        demand_index,
        0,
        vehicle_capacities,
        True,
        "Capacity"
    )

    # -------- Search parameters --------
    search_params = pywrapcp.DefaultRoutingSearchParameters()
    search_params.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    solution = routing.SolveWithParameters(search_params)

    if solution:
        print_solution(manager, routing, solution, data)
    else:
        print("❌ No feasible solution found")


def print_solution(manager, routing, solution, data):
    total_distance = 0

    print("\n===== CVRP SOLUTION =====")

    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        route_distance = 0
        route_load = 0

        print(f"\nVehicle {vehicle_id}:")

        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            route_load += data["demands"][node]
            print(f"  Node {node} | Load {route_load}")
            prev_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                prev_index, index, vehicle_id
            )

        print(f"  End | Load {route_load}")
        print(f"  Route distance: {route_distance}")

        total_distance += route_distance

    print(f"\nTotal distance: {total_distance}")


if __name__ == "__main__":
    data = load_data("data/synthetic/simple_10_nodes.json")
    solve_cvrp(data)
