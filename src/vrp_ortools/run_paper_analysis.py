import json
import sys
import time
from pathlib import Path
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

CURRENT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = CURRENT_DIR.parent.parent
INPUT_FILE = PROJECT_ROOT / "data" / "processed" / "global_optimization_data1.json"

AVERAGE_SPEED_KMPH = 40.0
SERVICE_TIME_MINS = 5.0
MAX_ROUTE_TIME_HOURS = 4.0
SPEED_METERS_PER_MIN = (AVERAGE_SPEED_KMPH * 1000) / 60.0
MAX_ROUTE_TIME_MINS = int(MAX_ROUTE_TIME_HOURS * 60)

def run_solver(data, mode="heterogeneous", time_limit_sec=90):
    num_nodes = len(data["distance_matrix"])
    num_vehicles = data["num_vehicles"]
    starts = data["starts"]
    ends = data["ends"]
    
    if mode == "heterogeneous":
        capacities = data["vehicle_capacities"]
        fixed_costs = data.get("fixed_costs", [0] * num_vehicles)
        var_costs = data.get("var_costs", [1] * num_vehicles)
        vehicle_labels = data.get("vehicle_labels", ["Unknown"] * num_vehicles)
    elif mode == "homogeneous_medium":
        # Simulate traditional homogeneous fleet (all Medium 7000L trucks)
        capacities = [7000] * num_vehicles
        fixed_costs = [15000] * num_vehicles
        var_costs = [28] * num_vehicles
        vehicle_labels = ["Medium"] * num_vehicles
    elif mode == "homogeneous_heavy":
        # Simulate traditional homogeneous fleet (all Heavy 25000L tankers)
        capacities = [25000] * num_vehicles
        fixed_costs = [25000] * num_vehicles
        var_costs = [55] * num_vehicles
        vehicle_labels = ["Heavy"] * num_vehicles

    manager = pywrapcp.RoutingIndexManager(num_nodes, num_vehicles, starts, ends)
    routing = pywrapcp.RoutingModel(manager)

    for v in range(num_vehicles):
        def create_cost_callback(vehicle_id):
            def cost_callback(from_index, to_index):
                from_node = manager.IndexToNode(from_index)
                to_node = manager.IndexToNode(to_index)
                distance_m = data["distance_matrix"][from_node][to_node]
                travel_cost = int((distance_m / 1000.0) * var_costs[vehicle_id])
                return travel_cost
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

    penalty = 100_000 # Using 100,000 as specified in Section 2.3 of the draft
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

    time_dim = routing.GetDimensionOrDie("Time")
    total_dist_m = 0
    total_load_l = 0
    total_time_min = 0
    total_cost_inr = 0
    vehicles_used = {}
    depot_stats = {}
    route_times_list = []
    dropped_nodes = []
    included_nodes_count = 0

    # Initialize depot stats
    unique_starts = set(starts)
    for s_node in unique_starts:
        d_name = data["names"][s_node].replace("DEPOT-", "").strip()
        depot_stats[d_name] = {"vccs": 0, "milk": 0, "vehicles": 0, "heavy": 0, "medium": 0, "light": 0}

    for node in range(num_nodes):
        if routing.IsStart(node) or routing.IsEnd(node):
            continue
        if solution.Value(routing.NextVar(manager.NodeToIndex(node))) == manager.NodeToIndex(node):
            dropped_nodes.append((data["names"][node], data["demands"][node]))
        else:
            included_nodes_count += 1

    for vehicle_id in range(num_vehicles):
        index = routing.Start(vehicle_id)
        if routing.IsEnd(solution.Value(routing.NextVar(index))):
            continue

        start_node = manager.IndexToNode(index)
        d_name = data["names"][start_node].replace("DEPOT-", "").strip()
        v_type = vehicle_labels[vehicle_id]
        fixed_cost = fixed_costs[vehicle_id]
        var_cost = var_costs[vehicle_id]

        vehicles_used[v_type] = vehicles_used.get(v_type, 0) + 1
        depot_stats[d_name]["vehicles"] += 1
        if v_type == "Heavy": depot_stats[d_name]["heavy"] += 1
        elif v_type == "Medium": depot_stats[d_name]["medium"] += 1
        elif v_type == "Light": depot_stats[d_name]["light"] += 1

        route_load = 0
        route_dist = 0
        vccs_in_route = 0

        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            demand = data["demands"][node_index]
            route_load += demand
            if demand > 0:
                vccs_in_route += 1
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            from_node = manager.IndexToNode(previous_index)
            to_node = manager.IndexToNode(index)
            route_dist += data["distance_matrix"][from_node][to_node]

        time_val = solution.Value(time_dim.CumulVar(index))
        route_times_list.append(time_val)
        
        dist_km = route_dist / 1000.0
        route_travel_cost = dist_km * var_cost
        route_total_cost = fixed_cost + route_travel_cost

        total_dist_m += route_dist
        total_load_l += route_load
        total_time_min += time_val
        total_cost_inr += route_total_cost

        depot_stats[d_name]["vccs"] += vccs_in_route
        depot_stats[d_name]["milk"] += route_load

    avg_route_time_min = sum(route_times_list) / len(route_times_list) if route_times_list else 0
    
    return {
        "mode": mode,
        "included_vccs": included_nodes_count,
        "dropped_vccs_count": len(dropped_nodes),
        "dropped_vccs": dropped_nodes,
        "total_dist_km": total_dist_m / 1000.0,
        "total_milk_l": total_load_l,
        "total_cost_inr": total_cost_inr,
        "total_vehicles_used": sum(vehicles_used.values()),
        "vehicles_breakdown": vehicles_used,
        "avg_route_time_hours": int(avg_route_time_min // 60),
        "avg_route_time_mins": int(avg_route_time_min % 60),
        "depot_stats": depot_stats
    }

def main():
    print(f"Loading optimization data from {INPUT_FILE}...")
    with open(INPUT_FILE, "r") as f:
        data = json.load(f)

    print("\n--- Running HETEROGENEOUS FLEET optimization (Target: 90s) ---")
    res_het = run_solver(data, mode="heterogeneous", time_limit_sec=90)

    print("\n--- Running HOMOGENEOUS MEDIUM FLEET optimization (Target: 60s) ---")
    res_med = run_solver(data, mode="homogeneous_medium", time_limit_sec=60)

    print("\n--- Running HOMOGENEOUS HEAVY FLEET optimization (Target: 60s) ---")
    res_hvy = run_solver(data, mode="homogeneous_heavy", time_limit_sec=60)

    print("\n" + "="*80)
    print("FINAL RESULTS SUMMARY FOR RESEARCH PAPER")
    print("="*80)
    print(f"\n1. TABLE 2: OPERATIONAL SUMMARY BY DEPOT HUB (HETEROGENEOUS FLEET)")
    print("-" * 80)
    print(f"{'Depot Name':<15} | {'VCCs Assigned':<15} | {'Total Milk (L)':<15} | {'Vehicles Used (H/M/L)':<25}")
    print("-" * 80)
    
    for d_name, stats in sorted(res_het['depot_stats'].items()):
        v_str = f"{stats['vehicles']} ({stats['heavy']}H / {stats['medium']}M / {stats['light']}L)"
        print(f"{d_name:<15} | {stats['vccs']:<15} | {stats['milk']:<15,d} | {v_str:<25}")
    print("-" * 80)

    print(f"\n2. SECTION 3.2: FLEET UTILIZATION AND LOGISTICS PERFORMANCE")
    print(f"   - Total Vehicles Used: {res_het['total_vehicles_used']} out of {data['num_vehicles']} initial pool")
    print(f"   - Vehicle Breakdown:   {res_het['vehicles_breakdown']}")
    print(f"   - Total Distance:      {res_het['total_dist_km']:.2f} km")
    print(f"   - Total Operational Cost: Rs. {res_het['total_cost_inr']:,.2f}")
    print(f"   - Total Milk Collected:   {res_het['total_milk_l']:,} L (out of {sum(data['demands']):,} L total demand)")
    print(f"   - Average Route Duration: {res_het['avg_route_time_hours']} hours and {res_het['avg_route_time_mins']} minutes")

    print(f"\n3. SECTION 3.3: NODE DISJUNCTIONS (DROPPED LOCATIONS)")
    print(f"   - Dropped VCCs count: {res_het['dropped_vccs_count']}")
    if res_het['dropped_vccs']:
        print(f"   - Dropped Locations (Name, Demand L):")
        for name, demand in res_het['dropped_vccs'][:10]: # Print top 10
            print(f"       * {name}: {demand} L")
        if len(res_het['dropped_vccs']) > 10:
            print(f"       * ... and {len(res_het['dropped_vccs']) - 10} more.")

    print(f"\n4. SECTION 3.4: COST-BENEFIT ANALYSIS vs HOMOGENEOUS FLEET")
    print(f"   - Heterogeneous Total Cost:   Rs. {res_het['total_cost_inr']:,.2f}")
    if res_med:
        saving_med = ((res_med['total_cost_inr'] - res_het['total_cost_inr']) / res_med['total_cost_inr']) * 100
        print(f"   - Homogeneous Medium Cost:    Rs. {res_med['total_cost_inr']:,.2f} (Savings: {saving_med:.2f}%) | Vehicles: {res_med['total_vehicles_used']} | Covered VCCs: {res_med['included_vccs']}")
    if res_hvy:
        saving_hvy = ((res_hvy['total_cost_inr'] - res_het['total_cost_inr']) / res_hvy['total_cost_inr']) * 100
        print(f"   - Homogeneous Heavy Cost:     Rs. {res_hvy['total_cost_inr']:,.2f} (Savings: {saving_hvy:.2f}%) | Vehicles: {res_hvy['total_vehicles_used']} | Covered VCCs: {res_hvy['included_vccs']}")
    print("="*80)

    out_file = PROJECT_ROOT / "results" / "paper_results.json"
    out_file.parent.mkdir(parents=True, exist_ok=True)
    with open(out_file, "w") as f:
        json.dump({"heterogeneous": res_het, "homogeneous_medium": res_med, "homogeneous_heavy": res_hvy}, f, indent=2)
    print(f"\nSaved complete results summary to {out_file}")

if __name__ == "__main__":
    main()

