# main.py

import sys
import os
import json
import numpy as np
import time
import matplotlib.pyplot as plt
from Map import Map
from Boundaries import Boundaries
from SearchEngine import build_graph, path_finding, compute_path_cost, h1, h2, discretize_coords

# Create plots directory
os.makedirs("plots", exist_ok=True)

# ---- Plotting Functions ----

def plot_radar_locations(boundaries, radar_locations, scenario_name, tolerance, heuristic_label):
    plt.figure(figsize=(8, 8))
    plt.title("Radar locations in the map")
    plt.plot([boundaries.min_lon, boundaries.max_lon, boundaries.max_lon, boundaries.min_lon, boundaries.min_lon],
             [boundaries.max_lat, boundaries.max_lat, boundaries.min_lat, boundaries.min_lat, boundaries.max_lat],
             label='Boundaries', linestyle='--', c='black')
    plt.scatter(radar_locations[:, 1], radar_locations[:, 0], label='Radars', c='green')
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.grid(True)
    plt.legend()
    filename = f"plots/{scenario_name}_tolerance_{tolerance}_{heuristic_label}_radars.png"
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    plt.close()

def plot_detection_fields(detection_map, scenario_name, tolerance, heuristic_label, bicubic=True):
    plt.figure(figsize=(8, 8))
    plt.title("Radar detection fields")
    im = plt.imshow(X=detection_map, cmap='Greens', interpolation='bicubic' if bicubic else None)
    plt.colorbar(im, label='Detection values')
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    filename = f"plots/{scenario_name}_tolerance_{tolerance}_{heuristic_label}_detection_map.png"
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    plt.close()

def plot_solution(detection_map, solution_plan, POIs, scenario_name, tolerance, heuristic_label, bicubic=True):
    detection_map = np.array(detection_map)
    H, W = detection_map.shape
    plt.figure(figsize=(8, 8))
    plt.title("Solution plan")
    if len(solution_plan) == 0:
        print("Solution not found with this tolerance. Consider increasing it.")
        return
    for path_segment in solution_plan:
        start_point = path_segment[0]
        plt.scatter(start_point[1], start_point[0], c='black', marker='*', zorder=2)
        path_array = np.array(path_segment)
        plt.plot(path_array[:, 1], path_array[:, 0], zorder=1)
    final_point = solution_plan[-1][-1]
    plt.scatter(final_point[1], final_point[0], c='black', marker='*', label='Waypoints', zorder=2)
    for idx, (y, x) in enumerate(POIs):
        plt.text(x + 0.4, y, f"$p_{{{idx+1}}}$", fontsize=9, verticalalignment='center')
    im = plt.imshow(X=detection_map, cmap='Greens', interpolation='bicubic' if bicubic else None)
    plt.colorbar(im, label='Detection values')
    plt.xticks(np.arange(W + 1) - 0.5, minor=True)
    plt.yticks(np.arange(H + 1) - 0.5, minor=True)
    plt.grid(which='minor', color='gray', linestyle='-', linewidth=0.5)
    plt.tick_params(which='both', bottom=False, left=False, labelbottom=False, labelleft=False)
    plt.legend()
    filename = f"plots/{scenario_name}_tolerance_{tolerance}_{heuristic_label}_solution.png"
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    plt.close()

# ---- A* Runner ----

def run_single_astar(scenario_name: str, config: dict, heuristic_fn, tolerance: float):
    import SearchEngine
    SearchEngine.NODES_EXPANDED = 0
    boundaries = Boundaries(config['max_lat'], config['min_lat'], config['max_lon'], config['min_lon'])
    M = Map(boundaries, config['H'], config['W'])
    M.generate_radars(n_radars=config['n_radars'])
    radar_locations = M.get_radars_locations_numpy()
    detection_map = M.compute_detection_map()
    G = build_graph(detection_map=detection_map, tolerance=tolerance)
    POIs = np.array(config['POIs'], dtype=np.float32)

    strt = time.time()
    try:
        solution_plan, nodes_expanded = path_finding(
            G=G,
            heuristic_function=heuristic_fn,
            locations=POIs,
            initial_location_index=0,
            boundaries=boundaries,
            map_width=M.width,
            map_height=M.height
        )
        path_cost = compute_path_cost(G=G, solution_plan=solution_plan)
    except Exception:
        return -1, -1, -1, None, None, None, None
    endt = time.time()
    return nodes_expanded, path_cost, round(endt - strt, 5), boundaries, radar_locations, detection_map, solution_plan

# ---- MAIN SINGLE ----

def main_single():
    if len(sys.argv) < 4:
        print("Usage: python main.py <scenario_name> <tolerance> <heuristic: h1|h2>")
        sys.exit(1)

    scenario_name_arg = sys.argv[1]
    tolerance_arg = float(sys.argv[2])
    heuristic_arg = sys.argv[3].lower()

    heuristic_map = {"h1": h1, "h2": h2}
    if heuristic_arg not in heuristic_map:
        print("Invalid heuristic. Use 'h1' or 'h2'.")
        sys.exit(1)

    heuristic_fn = heuristic_map[heuristic_arg]

    with open("scenarios.json", 'r') as f:
        all_scenarios = json.load(f)

    scenario_config = None
    for scenario in all_scenarios:
        if scenario_name_arg in scenario:
            scenario_config = scenario[scenario_name_arg]
            break

    if scenario_config is None:
        print(f"Scenario {scenario_name_arg} not found.")
        sys.exit(1)

    print(f"{'Scenario':<10}{'Heuristic':<10}{'Tolerance':<10}{'Nodes':<10}{'Cost':<12}{'Time(s)':<10}")
    print("-" * 62)

    nodes, cost, time_taken, boundaries, radar_locations, detection_map, solution_plan = run_single_astar(
        scenario_name_arg, scenario_config, heuristic_fn, tolerance_arg
    )

    print(f"{scenario_name_arg:<10}{heuristic_arg:<10}{tolerance_arg:<10.2f}{nodes:<10}{cost:<12.2f}{time_taken:<10.5f}")

    if detection_map is not None:
        discrete_POIs = discretize_coords(np.array(scenario_config['POIs'], dtype=np.float32),
                                          boundaries, scenario_config['W'], scenario_config['H'])

        plot_radar_locations(boundaries, radar_locations, scenario_name_arg, tolerance_arg, heuristic_arg)
        plot_detection_fields(detection_map, scenario_name_arg, tolerance_arg, heuristic_arg)
        plot_solution(detection_map, solution_plan, discrete_POIs, scenario_name_arg, tolerance_arg, heuristic_arg)

# ---- MAIN BATCH ----

def main_batch():
    with open("scenarios.json", 'r') as f:
        all_scenarios = json.load(f)

    heuristics = [("h1", h1), ("h2", h2)]
    tolerances = [0.2, 0.4, 0.6, 0.8, 1.0]

    print(f"{'Scenario':<10}{'Heuristic':<10}{'Tolerance':<10}{'Nodes':<10}{'Cost':<12}{'Time(s)':<10}")
    print("-" * 62)

    for scenario in all_scenarios:
        scenario_name = list(scenario.keys())[0]
        config = scenario[scenario_name]

        for tol in tolerances:
            for heuristic_label, heuristic_fn in heuristics:
                nodes, cost, time_taken, *_ = run_single_astar(scenario_name, config, heuristic_fn, tol)
                print(f"{scenario_name:<10}{heuristic_label:<10}{tol:<10.2f}{nodes:<10}{cost:<12.2f}{time_taken:<10.5f}")

# ---- EXECUTION ----

if __name__ == "__main__":
    main_single()   # ✅ Use CLI: python main.py scenario_3 0.4 h2
    # main_batch()  # 🔄 Uncomment this to batch test all scenarios
