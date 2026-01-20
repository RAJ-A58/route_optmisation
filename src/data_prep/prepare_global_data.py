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
CHUNK_SIZE = 50  # Safe limit for public OSRM server

# 1. DEPOT COORDINATES
DEPOT_COORDINATES = {
    "MCC BULDHANA":  (20.977988, 76.192343),
    "MCC BUTTIBORI": (20.942681, 78.945645),
    "MCC DHANOLI":   (21.066017, 78.602454),
    "BMC MANAPUR":   (21.382528, 79.322877),
    "BMC DAHID":     (20.498811, 76.062318),
    "BMC DHAD":      (20.380343, 75.993664)
}

# 2. FLEET CONFIGURATION
FLEET_CONFIG = {
    "MCC Buldhana": [2000, 1000, 500, 500],
    "MCC Buttibori":[1000, 1000],
    "MCC Dhanoli":  [1000, 500],
    "BMC Manapur":  [1000, 1000],
    "BMC Dahid":    [500, 500],
    "BMC Dhad":     [500, 500],
    "DEFAULT":      [500, 500, 500] 
}
# =================================================

def get_osrm_matrix_chunked(locations):
    """
    Fetches the distance matrix using chunking to avoid OSRM limits.
    """
    n = len(locations)
    matrix = [[0] * n for _ in range(n)]
    
    # Create chunks of indices (e.g., [0..49], [50..99], etc.)
    chunks = [list(range(i, min(i + CHUNK_SIZE, n))) for i in range(0, n, CHUNK_SIZE)]
    
    total_requests = len(chunks) * len(chunks)
    print(f"   Fetching Matrix in {total_requests} chunks (Total Nodes: {n})...")
    
    req_count = 0
    for i, src_indices in enumerate(chunks):
        for j, dst_indices in enumerate(chunks):
            req_count += 1
            print(f"     Processing chunk {req_count}/{total_requests}...", end="\r")
            
            # Prepare coordinates for this specific request
            # We combine Source + Dest coordinates into one list for the URL
            src_coords = [locations[idx] for idx in src_indices]
            dst_coords = [locations[idx] for idx in dst_indices]
            
            all_req_coords = src_coords + dst_coords
            coords_str = ";".join([f"{lon},{lat}" for lat, lon in all_req_coords])
            
            # OSRM 'table' parameters:
            # sources=0,1,2... (indices in the URL string referring to src_coords)
            # destinations=len(src), len(src)+1... (referring to dst_coords)
            src_params = ";".join(str(x) for x in range(len(src_coords)))
            dst_params = ";".join(str(x + len(src_coords)) for x in range(len(dst_coords)))
            
            url = f"{OSRM_URL}{coords_str}?annotations=distance&sources={src_params}&destinations={dst_params}"
            
            try:
                resp = requests.get(url, timeout=30)
                if resp.status_code == 200:
                    data = resp.json()
                    distances = data['distances']
                    
                    # Map the sub-matrix back to the main matrix
                    for r, row_global_idx in enumerate(src_indices):
                        for c, col_global_idx in enumerate(dst_indices):
                            dist_val = distances[r][c]
                            # Convert float meters to int meters, handle None
                            matrix[row_global_idx][col_global_idx] = int(dist_val) if dist_val is not None else 999999
                else:
                    print(f"\n     [ERROR] Chunk {req_count} failed: Status {resp.status_code}")
                    return None
            except Exception as e:
                print(f"\n     [ERROR] Chunk {req_count} exception: {e}")
                return None
            
            # Politeness delay for public server
            time.sleep(0.1)
            
    print("\n   Matrix fetch complete.")
    return matrix

def normalize_name(name):
    return str(name).strip().upper()

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
    
    # --- STEP 1: IDENTIFY DEPOTS ---
    unique_depots = df[col_dispatch].dropna().unique()
    nodes = []       
    depot_indices = [] 
    depot_real_names = {} 

    print(f"\n--- Processing {len(unique_depots)} Depots ---")

    for depot_name in unique_depots:
        d_key = normalize_name(depot_name)
        if d_key in DEPOT_COORDINATES:
            lat, lon = DEPOT_COORDINATES[d_key]
            print(f"   OK: {depot_name} -> Found Coords ({lat}, {lon})")
        else:
            print(f"   WARN: {depot_name} NOT found in coordinate list. Using first customer proxy.")
            proxy = df[df[col_dispatch] == depot_name].iloc[0]
            lat, lon = proxy["latitude"], proxy["longitude"]

        nodes.append({
            "name": f"DEPOT-{depot_name}",
            "lat": float(lat), "lon": float(lon), "demand": 0, "type": "depot"
        })
        current_idx = len(nodes) - 1
        depot_indices.append(current_idx)
        depot_real_names[current_idx] = str(depot_name).strip()

    # --- STEP 2: IDENTIFY CUSTOMERS ---
    print(f"\n--- Processing Customers ---")
    count_cust = 0
    for _, row in df.iterrows():
        qty = row[col_qty]
        name = str(row[col_name]).strip()
        
        if pd.isna(qty) or qty == 0: continue
        if pd.isna(row["latitude"]) or pd.isna(row["longitude"]): continue
        
        if normalize_name(name) in DEPOT_COORDINATES: continue

        nodes.append({
            "name": name,
            "lat": float(row["latitude"]), "lon": float(row["longitude"]),
            "demand": int(qty), "type": "customer" 
        })
        count_cust += 1

    print(f"   Added {count_cust} Customers. Total Nodes: {len(nodes)}")

    # --- STEP 3: BUILD DISTANCE MATRIX (CHUNKED) ---
    locations = [(n["lat"], n["lon"]) for n in nodes]
    matrix = get_osrm_matrix_chunked(locations)

    if not matrix:
        print("   [FAIL] Matrix generation failed.")
        return

    # --- STEP 4: ASSIGN FLEET ---
    print("\n--- Assigning Fleet ---")
    starts = []
    ends = []
    capacities = []

    for d_idx in depot_indices:
        real_name = depot_real_names[d_idx]
        fleet = FLEET_CONFIG.get(real_name)
        if not fleet:
             for key in FLEET_CONFIG:
                 if key.upper() == real_name.upper():
                     fleet = FLEET_CONFIG[key]
                     break
        if not fleet: fleet = FLEET_CONFIG["DEFAULT"]

        for cap in fleet:
            starts.append(d_idx)
            ends.append(d_idx)
            capacities.append(cap)
    
    print(f"   Total Fleet Size: {len(capacities)} Vehicles")

    # --- STEP 5: SAVE JSON ---
    output = {
        "distance_matrix": matrix,
        "demands": [n["demand"] for n in nodes],
        "vehicle_capacities": capacities,
        "num_vehicles": len(starts),
        "starts": starts,
        "ends": ends,
        "names": [n["name"] for n in nodes],
        "locations": [{"lat": n["lat"], "lon": n["lon"]} for n in nodes]
    }

    OUTPUT_FILE.parent.mkdir(parents=True, exist_ok=True)
    with open(OUTPUT_FILE, "w") as f:
        json.dump(output, f, indent=2)
        
    print(f"\n[SUCCESS] Global MDVRP Data saved to: {OUTPUT_FILE}")

if __name__ == "__main__":
    prepare_global_data()