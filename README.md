# A Heterogeneous Fleet Approach to the Capacitated Multi-Depot Vehicle Routing Problem with Time Windows for Indian Cooperative Dairy

This repository contains the dataset preprocessing pipeline, optimization solver code, and analytical results for the paper submitted to the **11th International Conference on Soft Computing: Theories and Applications (SoCTA 2026)** at SVNIT Surat.

## 🚀 Overview

Milk collection in Indian cooperative dairies is a critical logistical challenge due to product perishability, geographically dispersed collection centers, and strict temporal constraints. This project implements a **Capacitated Multi-Depot Vehicle Routing Problem with Time Windows (CVRPTW)** supporting a heterogeneous vehicle fleet. 

### Key Contributions:
* **Real-World Road Geometry:** Integrates Google OR-Tools with the Open Source Routing Machine (OSRM) to eliminate Euclidean distance errors, which typically distort Indian rural road logistics by 30-50%.
* **Heterogeneous Fleet Optimization:** Models 378 collection points using a heterogeneous fleet tailored to Indian rural constraints, reducing daily operational costs by 60.4% compared to a homogeneous heavy-tanker fleet.
* **Strict Perishability Constraints:** Enforces a 4-hour maximum shift duration using time-dimension constraints and Guided Local Search (GLS) with node disjunctions.

## 📁 Repository Structure

```text
├── data/                       # Raw and processed optimization datasets (JSON/CSV)
├── graphs/                     # Generated evaluation plots and figures
├── results/                    # Optimization outputs, benchmark logs, and sensitivity analysis
├── src/
│   ├── data_prep/              # OSRM matrix generation and data cleaning scripts
│   └── vrp_ortools/            # Core OR-Tools Constraint Programming models
├── socta_dairy_mdvrp_source/   # LaTeX source code for the SoCTA conference paper
├── requirements.txt            # Project dependencies
└── README.md
```

## 🚦 Getting Started

### 1. Install Dependencies
Ensure you have Python 3.8+ installed, then run:
```bash
pip install -r requirements.txt
```

### 2. Generate Distance Matrix
Run the data preparation script to generate the real-road distance matrix via OSRM API:
```bash
python src/data_prep/prepare_global_data.py
```

### 3. Run Optimization
Execute the core OR-Tools solver to generate optimized routes across all depots:
```bash
python src/vrp_ortools/solve_mdvrp.py
```

## 📄 License & Citation
The code in this repository is available for academic and research purposes. If you build upon this work, please cite our SoCTA 2026 paper appropriately.
