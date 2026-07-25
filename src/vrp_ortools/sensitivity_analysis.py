import json
import sys
import copy
from pathlib import Path
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

CURRENT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = CURRENT_DIR.parent.parent
INPUT_FILE = PROJECT_ROOT / "data" / "processed" / "global_optimization_data1.json"
OUTPUT_FILE = PROJECT_ROOT / "results" / "sensitivity_analysis.json"

AVERAGE_SPEED_KMPH = 40.0
SERVICE_TIME_MINS = 5.0
MAX_ROUTE_TIME_HOURS = 4.0
SPEED_METERS_PER_MIN = (AVERAGE_SPEED_KMPH * 1000) / 60.0
MAX_ROUTE_TIME_MINS = int(MAX_ROUTE_TIME_HOURS * 60)

def run_sensitivity_solver(data, scenario_name, time_limit_sec=60):
    num_nodes = len(data["distance_matrix"])
    num_vehicles = data["num_vehicles"]
    starts = data["starts"]
    ends = data["ends"]
    
    capacities = data["vehicle_capacities"]
    fixed_costs = data.get("fixed_costs", [0] * num_vehicles)
    var_costs = data.get("var_costs", [1] * num_vehicles)
    vehicle_labels = data.get("vehicle_labels", ["Unknown"] * num_vehicles)

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
    search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_parameters.time_limit.seconds = time_limit_sec

    solution = routing.SolveWithParameters(search_parameters)
    
    if not solution:
        return None

    total_cost_inr = 0
    vehicles_used = {}
    total_milk_collected = 0
    dropped_nodes = 0
    
    for node in range(num_nodes):
        if routing.IsStart(node) or routing.IsEnd(node):
            continue
        if solution.Value(routing.NextVar(manager.NodeToIndex(node))) == manager.NodeToIndex(node):
            dropped_nodes += 1

    for vehicle_id in range(num_vehicles):
        index = routing.Start(vehicle_id)
        if routing.IsEnd(solution.Value(routing.NextVar(index))):
            continue
            
        v_type = vehicle_labels[vehicle_id]
        vehicles_used[v_type] = vehicles_used.get(v_type, 0) + 1
        
        route_dist = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            total_milk_collected += data["demands"][node_index]
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            from_node = manager.IndexToNode(previous_index)
            to_node = manager.IndexToNode(index)
            route_dist += data["distance_matrix"][from_node][to_node]
        
        dist_km = route_dist / 1000.0
        total_cost_inr += fixed_costs[vehicle_id] + (dist_km * var_costs[vehicle_id])

    return {
        "Scenario": scenario_name,
        "Total_Milk_Collected_L": total_milk_collected,
        "Total_Cost_INR": total_cost_inr,
        "Total_Vehicles": sum(vehicles_used.values()),
        "Vehicle_Breakdown": vehicles_used,
        "Dropped_Nodes": dropped_nodes
    }

def main():
    print(f"Loading optimization data from {INPUT_FILE}...")
    with open(INPUT_FILE, "r") as f:
        base_data = json.load(f)

    results = []
    
    # 1. Base Scenario (1.0x)
    print("\n--- Running Base Scenario (1.0x) ---")
    res_base = run_sensitivity_solver(base_data, "Base Scenario (1.0x)")
    if res_base: results.append(res_base)

    # 2. Flush Season (1.2x)
    print("\n--- Running Flush Season (+20%) ---")
    data_flush = copy.deepcopy(base_data)
    data_flush["demands"] = [int(d * 1.2) for d in base_data["demands"]]
    res_flush = run_sensitivity_solver(data_flush, "Flush Season (+20%)")
    if res_flush: results.append(res_flush)

    # 3. Lean Season (0.8x)
    print("\n--- Running Lean Season (-20%) ---")
    data_lean = copy.deepcopy(base_data)
    data_lean["demands"] = [int(d * 0.8) for d in base_data["demands"]]
    res_lean = run_sensitivity_solver(data_lean, "Lean Season (-20%)")
    if res_lean: results.append(res_lean)

    print("\n" + "="*50)
    print("SENSITIVITY ANALYSIS RESULTS")
    print("="*50)
    for res in results:
        print(f"[{res['Scenario']}]")
        print(f"  - Milk Collected: {res['Total_Milk_Collected_L']:,} L")
        print(f"  - Total Cost:     Rs. {res['Total_Cost_INR']:,.2f}")
        print(f"  - Vehicles Used:  {res['Total_Vehicles']} {res['Vehicle_Breakdown']}")
        print(f"  - Dropped Nodes:  {res['Dropped_Nodes']}")
        print("-" * 50)
        
    OUTPUT_FILE.parent.mkdir(parents=True, exist_ok=True)
    with open(OUTPUT_FILE, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\nResults written to {OUTPUT_FILE}")

if __name__ == "__main__":
    main()
