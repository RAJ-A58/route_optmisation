import pandas as pd
import json
import requests
import sys
import time
from pathlib import Path

# ================= CONFIGURATION =================
CURRENT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = CURRENT_DIR.parent.parent
INPUT_FILE = PROJECT_ROOT / "data" / "raw" / "MPP & BMC Mcc.xlsx"
OUTPUT_FILE = PROJECT_ROOT / "data" / "processed" / "global_optimization_data.json"

OSRM_URL = "http://router.project-osrm.org/table/v1/driving/"
CHUNK_SIZE = 50

# 1. DEPOT COORDINATES
DEPOT_COORDINATES = {
    "MCC BULDHANA":  (20.977988, 76.192343),
    "MCC BUTTIBORI": (20.942681, 78.945645),
    "MCC DHANOLI":   (21.066017, 78.602454),
    "BMC MANAPUR":   (21.382528, 79.322877),
    "BMC DAHID":     (20.498811, 76.062318),
    "BMC DHAD":      (20.380343, 75.993664)
}

# 2. FLEET CONFIGURATION (Swarm Strategy)
FLEET_CONFIG = {
    "MCC Buldhana": [
        2500, 2500, 2500, 2500, 2500, 
        2000, 2000, 2000, 2000, 2000, 
        2000, 1500, 1500, 1500, 1500, 
        1000, 1000, 1000
    ],
    "MCC Buttibori": [2500, 2500, 2000, 2000, 2000, 1500, 1500],
    "MCC Dhanoli":   [2500, 2500, 2000, 2000, 2000, 1500, 1500],
    "BMC Manapur":   [2500, 2500, 2500, 2000, 2000, 2000, 1500, 1500],
    "BMC Dahid":     [2000, 2000, 2000, 1500, 1000],
    "BMC Dhad":      [2000, 2000, 2000, 1500, 1000],
    "DEFAULT":       [2000, 1500, 1000] 
}

def normalize_sorted_key(name):
    """Turns 'Dahid Bmc' -> 'BMC DAHID' for sorting/matching."""
    if pd.isna(name): return ""
    words = str(name).strip().upper().split()
    return " ".join(sorted(words))

def get_osrm_matrix_chunked(locations):
    n = len(locations)
    matrix = [[0] * n for _ in range(n)]
    chunks = [list(range(i, min(i + CHUNK_SIZE, n))) for i in range(0, n, CHUNK_SIZE)]
    
    total_chunks = len(chunks) ** 2
    print(f"   Fetching Matrix in {total_chunks} chunks (Total Nodes: {n})...")
    
    processed_count = 0
    for i, src_indices in enumerate(chunks):
        for j, dst_indices in enumerate(chunks):
            processed_count += 1
            print(f"     Processing Chunk {processed_count}/{total_chunks}...", end="\r")
            
            src_coords = [locations[idx] for idx in src_indices]
            dst_coords = [locations[idx] for idx in dst_indices]
            all_req_coords = src_coords + dst_coords
            
            coords_str = ";".join([f"{lon},{lat}" for lat, lon in all_req_coords])
            src_params = ";".join(str(x) for x in range(len(src_coords)))
            dst_params = ";".join(str(x + len(src_coords)) for x in range(len(dst_coords)))
            
            url = f"{OSRM_URL}{coords_str}?annotations=distance&sources={src_params}&destinations={dst_params}"
            
            try:
                resp = requests.get(url, timeout=30)
                if resp.status_code == 200:
                    data = resp.json()
                    distances = data['distances']
                    for r, row_global_idx in enumerate(src_indices):
                        for c, col_global_idx in enumerate(dst_indices):
                            dist_val = distances[r][c]
                            matrix[row_global_idx][col_global_idx] = int(dist_val) if dist_val is not None else 999999
            except Exception:
                pass
            time.sleep(0.1)
    
    print("\n   Matrix fetch complete.")
    return matrix

def prepare_global_data():
    if not INPUT_FILE.exists():
        print(f"ERROR: File not found at {INPUT_FILE}")
        return

    print(f"Loading {INPUT_FILE}...")
    df = pd.read_excel(INPUT_FILE)
    df.columns = df.columns.astype(str).str.strip().str.lower().str.replace(" ", "_")
    
    col_dispatch = "milk_dispatched_locations"
    col_name = "name" if "name" in df.columns else df.columns[1]
    col_qty = "milk_qty"
    
    # --- BUILD DEPOT LOOKUP TABLE ---
    known_depot_keys = set()
    for key in DEPOT_COORDINATES:
        known_depot_keys.add(normalize_sorted_key(key))

    unique_depots = df[col_dispatch].dropna().unique()
    nodes = []       
    depot_indices = [] 
    depot_real_names = {} 

    print(f"\n--- Processing {len(unique_depots)} Depots ---")
    for depot_name in unique_depots:
        d_key = normalize_sorted_key(depot_name)
        found_coord = None
        for coord_name, coords in DEPOT_COORDINATES.items():
            if normalize_sorted_key(coord_name) == d_key:
                found_coord = coords
                break
        
        if found_coord:
            lat, lon = found_coord
        else:
            proxy = df[df[col_dispatch] == depot_name].iloc[0]
            lat, lon = proxy["latitude"], proxy["longitude"]

        nodes.append({"name": f"DEPOT-{depot_name}", "lat": float(lat), "lon": float(lon), "demand": 0, "type": "depot"})
        current_idx = len(nodes) - 1
        depot_indices.append(current_idx)
        depot_real_names[current_idx] = str(depot_name).strip()

    print(f"--- Processing Customers ---")
    count_cust = 0
    skipped_count = 0
    
    for _, row in df.iterrows():
        qty = row[col_qty]
        raw_name = str(row[col_name]).strip()
        
        if pd.isna(qty) or qty == 0: continue
        if pd.isna(row["latitude"]) or pd.isna(row["longitude"]): continue
        
        # --- CORRECTED LOGIC ---
        # 1. Sorted Name Match: ONLY removes if it matches a KNOWN depot (e.g. "Dahid Bmc")
        if normalize_sorted_key(raw_name) in known_depot_keys:
            print(f"   [INFO] DEPOT REMOVED from Customer List: '{raw_name}' (Matches known depot)")
            skipped_count += 1
            continue

        # 2. (REMOVED) The aggressive "Name contains BMC/MCC" check is GONE.
        # Bmc Beena and Bmc Dhanala will now be added as normal customers.

        nodes.append({"name": raw_name, "lat": float(row["latitude"]), "lon": float(row["longitude"]), "demand": int(qty), "type": "customer"})
        count_cust += 1

    print(f"   Added {count_cust} Customers (Removed {skipped_count} Depots).")
    print(f"   Total Nodes: {len(nodes)}")
    
    locations = [(n["lat"], n["lon"]) for n in nodes]
    matrix = get_osrm_matrix_chunked(locations)

    if not matrix:
        print("   [FAIL] Matrix generation failed.")
        return

    print("--- Assigning Fleet ---")
    starts = []
    ends = []
    capacities = []

    for d_idx in depot_indices:
        real_name = depot_real_names[d_idx]
        fleet = FLEET_CONFIG.get(real_name)
        if not fleet:
             for key in FLEET_CONFIG:
                 if normalize_sorted_key(key) == normalize_sorted_key(real_name): 
                     fleet = FLEET_CONFIG[key]
                     break
        if not fleet: fleet = FLEET_CONFIG["DEFAULT"]

        for cap in fleet:
            starts.append(d_idx)
            ends.append(d_idx)
            capacities.append(cap)
    
    print(f"   Total Fleet Size: {len(capacities)} Vehicles")

    output = {
        "distance_matrix": matrix, "demands": [n["demand"] for n in nodes],
        "vehicle_capacities": capacities, "num_vehicles": len(starts),
        "starts": starts, "ends": ends,
        "names": [n["name"] for n in nodes], "locations": [{"lat": n["lat"], "lon": n["lon"]} for n in nodes]
    }

    OUTPUT_FILE.parent.mkdir(parents=True, exist_ok=True)
    with open(OUTPUT_FILE, "w") as f: json.dump(output, f, indent=2)
    print(f"\n[SUCCESS] Global MDVRP Data saved to: {OUTPUT_FILE}")

if __name__ == "__main__":
    prepare_global_data()