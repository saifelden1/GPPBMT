# ROS 2 Standalone Global Path Planning Benchmarking Tool

A standalone, modular testing tool for evaluating any ROS 2 global path planning module as a black box. This tool will run as a configurable ROS 2 node, allowing easy integration into a larger benchmarking suite later.

## User Review Required

> [!IMPORTANT]
> - **Pre-Test Visual Verification:** Before running tests, the tool can be run with `--verify` to generate visual PNGs showing only the map and the pre-defined **Reference (Perfect) Route** so that you can verify their correctness.
> - **Post-Test Visual Comparison:** At the end of a test run, the tool generates a PNG showing the **Reference Route (Green)** and the **Planner Route (Red)** side-by-side on the map.
> - **Costmap / Height Map Representation:** Standard `nav_msgs/msg/OccupancyGrid` will be published to `/global_costmap/costmap` or `/map`.
> - **YAML Configurable:** Scenarios can be easily added, deleted, or edited in `scenarios.yaml`.

---

## Detailed Test Sequence (Before, During, and After)

### 1. Before the Test
* **Input:** `scenarios.yaml` file + Map files (PNG/PGM images).
* **Process:** The tool loads the YAML configuration. If the `--verify` flag is set, it reads each map, draws the pre-defined reference path on it, and saves it as `reports/verify_scenario_<id>.png`. The user can inspect this image.
* **Output:** Verification PNGs showing the reference route on the map.

### 2. During the Test
* **Input:** Costmap (published on `/global_costmap/costmap`), Start Pose, and Goal Pose (sent to the planner).
* **Process:** 
  1. The testing tool starts the global planner node.
  2. The tool publishes the scenario map to the planner.
  3. The tool sends a planning request (Start and Goal) via the standard Nav2 `ComputePathToPose` Action.
  4. The tool starts a timer (computation time).
  5. *Dynamic Updates:* If dynamic obstacles are defined in the scenario, the tool updates the costmap mid-test, detects if the planner is notified, and requests a replan.
  6. The tool receives the planned path from the planner and stops the timer.
* **Output:** Planned path (`nav_msgs/msg/Path`), planning status (Success/Failure/Timeout), and planning time (seconds).

### 3. After the Test
* **Process:**
  1. The tool calculates all benchmarking metrics (Safety margin, blocked cells, path length ratio, replanning score).
  2. The tool computes the final ERC Path Planning Score.
  3. The tool generates a comparison PNG showing the **Reference Route in Green** and the **Planner's Route in Red**.
  4. The tool saves all numerical metrics to `reports/results.json`.
* **Output:** Final benchmark score, comparison PNG, and JSON log file.

---

## The `scenarios.yaml` Schema

You can edit or add new scenarios to the configuration file using this structure:

```yaml
scenarios:
  - id: "marsyard_center_route"
    # Path to the map image file (black pixels are obstacles, white pixels are free space)
    map_image: "maps/marsyard_center.png"
    # Map metadata: resolution (meters per pixel) and origin (x, y of bottom-left corner)
    resolution: 0.05
    origin: [-10.0, -10.0]
    
    # Start and Goal coordinates in meters
    start: [ -2.0, 3.0 ]
    goal: [ 8.0, 12.0 ]
    
    # The pre-calculated optimal route waypoints in meters
    reference_path:
      - [ -2.0, 3.0 ]
      - [ 0.0, 5.0 ]
      - [ 3.0, 8.0 ]
      - [ 5.0, 10.0 ]
      - [ 8.0, 12.0 ]
      
    # Dynamic obstacles for testing height map updates and replanning
    dynamic_obstacles:
      - trigger_time: 1.0  # seconds after test start to place the obstacle
        x: 3.0
        y: 8.0
        radius: 0.5        # size of the obstacle to block the path
```

---

## Proposed Code Structure

We will create a clean standalone testing utility inside `Global_path_benchmarking/`:
1. `benchmarker.py`: Single-entry point Python script containing the ROS 2 node, metric logic, and image generator.
2. `mock_planner.py`: Lightweight planner for testing.
3. `scenarios.yaml`: Configuration file.
4. `maps/`: Directory for PNG maps.

## Verification Plan

### Automated Tests
1. Generate verification images:
   ```bash
   python3 benchmarker.py --config config/scenarios.yaml --verify
   ```
2. Run mock planner:
   ```bash
   python3 mock_planner.py
   ```
3. Run benchmark:
   ```bash
   python3 benchmarker.py --config config/scenarios.yaml --output reports/results.json
   ```
4. Verify visual reports generated in `reports/`.
