import json
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

# =========================================================
# CONFIGURATION (CHANGE ONLY THESE VALUES)
# =========================================================

AVERAGE_SPEED_KMPH = 30      # vehicle speed
SERVICE_TIME_MIN = 10        # service time per stop
MAX_ROUTE_TIME_MIN = 240     # ⬅️ change THIS easily later

# =========================================================


def load_data(path):
    with open(path, "r") as f:
        return json.load(f)


def solve_cvrp_with_max_time(data):
    num_nodes = len(data["distance_matrix"])
    num_vehicles = data["num_vehicles"]
    depot = data["depot"]
    vehicle_capacities = data["vehicle_capacities"]

    manager = pywrapcp.RoutingIndexManager(
        num_nodes, num_vehicles, depot
    )

    routing = pywrapcp.RoutingModel(manager)

    # ---------------------------------------------------------
    # Distance callback (objective)
    # ---------------------------------------------------------
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    transit_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_index)

    # ---------------------------------------------------------
    # Demand callback (capacity)
    # ---------------------------------------------------------
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

    # ---------------------------------------------------------
    # Time callback (route duration)
    # ---------------------------------------------------------
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)

        distance_m = data["distance_matrix"][from_node][to_node]

        # distance -> time
        travel_time = int(
            (distance_m / 1000) / AVERAGE_SPEED_KMPH * 60
        )

        return travel_time + SERVICE_TIME_MIN

    time_index = routing.RegisterTransitCallback(time_callback)

    routing.AddDimension(
        time_index,
        0,                     # no waiting
        MAX_ROUTE_TIME_MIN,    # ⬅️ single control point
        True,
        "Time"
    )

    # ---------------------------------------------------------
    # Search parameters
    # ---------------------------------------------------------
    search_params = pywrapcp.DefaultRoutingSearchParameters()
    search_params.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    solution = routing.SolveWithParameters(search_params)

    if solution:
        print_solution(manager, routing, solution, data)
    else:
        print("❌ No feasible solution under current constraints")


def print_solution(manager, routing, solution, data):
    print("\n===== CVRP WITH MAX ROUTE TIME =====")

    time_dimension = routing.GetDimensionOrDie("Time")

    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        route_load = 0

        print(f"\nVehicle {vehicle_id}:")

        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            route_load += data["demands"][node]
            time = solution.Value(time_dimension.CumulVar(index))
            print(f"  Node {node} | Load {route_load} | Time {time} min")
            index = solution.Value(routing.NextVar(index))

        time = solution.Value(time_dimension.CumulVar(index))
        print(f"  End | Total time {time} min")


if __name__ == "__main__":
    data = load_data("data/synthetic/simple_10_nodes.json")
    solve_cvrp_with_max_time(data)
