===========================================
 A* RADAR PATHFINDING - EXECUTION GUIDE
===========================================

This project implements A* heuristic search for stealth UAV path planning over radar fields. The system models radar detection zones and uses A* with admissible heuristics to find low-risk paths between mission-critical waypoints.

This version allows running a single scenario with a selected tolerance and heuristic (h1 or h2).

-------------------------------------------
REQUIREMENTS
-------------------------------------------
Python version 3.8+ is required.

Install dependencies with:

    pip install numpy matplotlib networkx

-------------------------------------------
RUNNING main_single_updated.py
-------------------------------------------

Usage:

    python main_single_updated.py <scenario_name> <tolerance> <heuristic>

Arguments:
- <scenario_name>: name of the scenario from scenarios.json (e.g., scenario_3)
- <tolerance>: float between 0 and 1 controlling movement threshold
- <heuristic>: either "h1" or "h2"

Example:

    python main_single_updated.py scenario_4 0.6 h2

What it does:
- Runs A* with the selected heuristic and tolerance
- Prints:
    - Number of nodes expanded
    - Total path cost
    - Time taken
- Generates and saves 3 plots in the /plots directory:
    - scenario_4_tolerance_0.6_h2_radars.png
    - scenario_4_tolerance_0.6_h2_detection_map.png
    - scenario_4_tolerance_0.6_h2_solution.png

All plots contain:
- Grid lines
- POI labels (e.g., p₁, p₂, ...)
- Legend and detection heatmaps

-------------------------------------------
PLOT FILE NAMING CONVENTION
-------------------------------------------

Files are automatically saved as:

    plots/scenario_<name>_tolerance_<value>_<heuristic>_<type>.png

Example:

    scenario_3_tolerance_0.4_h1_solution.png

Plot types:
- radars
- detection_map
- solution

-------------------------------------------
TROUBLESHOOTING
-------------------------------------------

If you see:

    No path found for h2, scenario_2 with tolerance 0.2

This means the selected tolerance is too low to allow a viable path. Try increasing it to 0.4, 0.6, or higher.

If the script crashes with “ModuleNotFoundError”, make sure that `Map.py`, `SearchEngine.py`, and `Boundaries.py` are present in the same directory.

-------------------------------------------
SCENARIO CONFIGURATION
-------------------------------------------

All scenarios are located in `scenarios.json`. Each scenario includes:
- Map dimensions (H x W)
- Radar count and coordinates
- List of Points of Interest (POIs) to be visited

To add your own scenarios, replicate the format and ensure valid lat/lon values.
