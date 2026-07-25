import json
import sys
import time
from pathlib import Path
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

CURRENT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = CURRENT_DIR.parent.parent
INPUT_FILE = PROJECT_ROOT / "data" / "processed" / "global_optimization_data1.json"
OUTPUT_FILE = PROJECT_ROOT / "results" / "algorithmic_benchmark.json"

AVERAGE_SPEED_KMPH = 40.0
SERVICE_TIME_MINS = 5.0
MAX_ROUTE_TIME_HOURS = 4.0
SPEED_METERS_PER_MIN = (AVERAGE_SPEED_KMPH * 1000) / 60.0
MAX_ROUTE_TIME_MINS = int(MAX_ROUTE_TIME_HOURS * 60)

def run_benchmark_solver(data, metaheuristic_name, metaheuristic_enum, time_limit_sec=60):
    num_nodes = len(data["distance_matrix"])
    num_vehicles = data["num_vehicles"]
    starts = data["starts"]
    ends = data["ends"]
    
    capacities = data["vehicle_capacities"]
    fixed_costs = data.get("fixed_costs", [0] * num_vehicles)
    var_costs = data.get("var_costs", [1] * num_vehicles)

    manager = pywrapcp.RoutingIndexManager(num_nodes, num_vehicles, starts, ends)
    routing = pywrapcp.RoutingModel(manager)

    for v in range(num_vehicles):
        def create_cost_callback(vehicle_id):
            def cost_callback(from_index, to_index):
                from_node = manager.IndexToNode(from_index)
                to_node = manager.IndexToNode(to_index)
                distance_m = data["distance_matrix"][from_node][to_node]
                return int((distance_m / 1000.0) * var_costs[vehicle_id])
            return cost_callback
        callback_index = routing.RegisterTransitCallback(create_cost_callback(v))
        routing.SetArcCostEvaluatorOfVehicle(callback_index, v)
        routing.SetFixedCostOfVehicle(fixed_costs[v], v)

    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return data["demands"][from_node]
    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(demand_callback_index, 0, capacities, True, "Capacity")

    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        dist = data["distance_matrix"][from_node][to_node]
        travel_time = dist / SPEED_METERS_PER_MIN
        is_customer = data["demands"][from_node] > 0
        service_time = SERVICE_TIME_MINS if is_customer else 0
        return int(travel_time + service_time)
    time_callback_index = routing.RegisterTransitCallback(time_callback)
    routing.AddDimension(time_callback_index, 0, MAX_ROUTE_TIME_MINS, False, "Time")

    penalty = 100_000
    for node in range(num_nodes):
        if node not in starts and node not in ends:
            routing.AddDisjunction([manager.NodeToIndex(node)], penalty)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_parameters.local_search_metaheuristic = metaheuristic_enum
    search_parameters.time_limit.seconds = time_limit_sec

    start_time = time.time()
    solution = routing.SolveWithParameters(search_parameters)
    end_time = time.time()
    
    compute_time = end_time - start_time

    if not solution:
        return None

    total_cost_inr = 0
    vehicles_used = 0
    for vehicle_id in range(num_vehicles):
        index = routing.Start(vehicle_id)
        if routing.IsEnd(solution.Value(routing.NextVar(index))):
            continue
        vehicles_used += 1
        route_dist = 0
        while not routing.IsEnd(index):
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            from_node = manager.IndexToNode(previous_index)
            to_node = manager.IndexToNode(index)
            route_dist += data["distance_matrix"][from_node][to_node]
        
        dist_km = route_dist / 1000.0
        route_total_cost = fixed_costs[vehicle_id] + (dist_km * var_costs[vehicle_id])
        total_cost_inr += route_total_cost

    return {
        "Algorithm": metaheuristic_name,
        "Compute_Time_sec": compute_time,
        "Total_Cost_INR": total_cost_inr,
        "Vehicles_Used": vehicles_used,
        "Objective_Value": solution.ObjectiveValue()
    }

def main():
    print(f"Loading optimization data from {INPUT_FILE}...")
    with open(INPUT_FILE, "r") as f:
        data = json.load(f)

    algorithms = [
        ("Guided Local Search", routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH),
        ("Tabu Search", routing_enums_pb2.LocalSearchMetaheuristic.TABU_SEARCH),
        ("Simulated Annealing", routing_enums_pb2.LocalSearchMetaheuristic.SIMULATED_ANNEALING)
    ]

    results = []
    print("\n--- Starting Algorithmic Benchmark (60s time limit per algorithm) ---")
    
    for name, enum_val in algorithms:
        print(f"\nEvaluating {name}...")
        res = run_benchmark_solver(data, name, enum_val, time_limit_sec=60)
        if res:
            print(f"  -> Cost: Rs. {res['Total_Cost_INR']:,.2f} | Time: {res['Compute_Time_sec']:.2f}s | Vehicles: {res['Vehicles_Used']}")
            results.append(res)
        else:
            print(f"  -> {name} failed to find a solution.")
            
    print("\nSaving results...")
    OUTPUT_FILE.parent.mkdir(parents=True, exist_ok=True)
    with open(OUTPUT_FILE, "w") as f:
        json.dump(results, f, indent=2)
    print(f"Done! Results written to {OUTPUT_FILE}")

if __name__ == "__main__":
    main()
