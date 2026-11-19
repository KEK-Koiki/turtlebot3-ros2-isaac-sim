# Reproduction notes

This public repository is a curated integration showcase. It contains selected code and observed results. The original course starter, six packaged USD stages, raw robot/room assets, and course report are retained outside this repository.

## Review without a simulator

With Python 3.9 or newer, run from the repository root:

```bash
python3 scripts/check_saved_evidence.py
```

The checker verifies the six received streams, exact expected TF edges, image and lidar sample fields, retained acceptance flags, and six checkpoint equivalence records. It checks saved evidence and does not execute a new simulation.

## Recorded environment

The original recorded environment was Ubuntu 24.04.5, NVIDIA Isaac Sim 6.0.1, ROS 2 Jazzy, Fast DDS (`rmw_fastrtps_cpp`), and ROS domain 0. The reported machine used an RTX 5070 Laptop GPU with 8 GB VRAM. The course handout specifies Humble; only the actual Jazzy environment is represented by these logs.

## Reconstruct with the original course scene

Obtain the starter scene through the [lab page](https://tatwik19.github.io/labs/lab-03-isaac-sim/) and follow its A–F sequence. For the original graph builder’s expected layout, place a completed Part C stage at `EK_Checkpoints/EK_Lab3_02_ros_drive.usda` beside `EK_Code/`. Run the builder using the Isaac Sim Python environment with the required extensions and scene assets available. It authors the lidar, camera, and odometry/TF stages. It is a historical construction utility; later packaging and renderer cleanup are not a single-command rebuild in this showcase.

The original completed stage used `isaacsim.ros2.bridge`, RTX sensor nodes, and Motion BVH for the rotary RTX lidar. Match the saved controller parameters, camera placement, frame IDs, and simulation timestamps described in the [engineering notes](engineering-notes.md).

## External ROS validation

Use a separate system ROS Python process rather than importing system ROS message bindings into Isaac Sim’s process. The original validator uses ROS 2 Jazzy message packages and Pillow for the camera snapshot. It expects sibling `EK_Logs/` and `EK_Evidence/` directories next to `EK_Code/`.

In a working copy, with a compatible scene already playing:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export EK_VALIDATION_NAME=new_live_validation
python3 lab/EK_Code/EK_validate_live.py
```

This approximately 32-second wall-clock session **commands simulated robot motion**: forward, stop, rotation, stop. Use an isolated simulation ROS domain and a working copy; it writes a log and camera image. The legacy validator records observations but does not itself assert every acceptance criterion. Review positive message counts, an empty `missing` list, and the expected TF edges. Its wall-clock command schedule means travel can vary with simulation speed.

The commands and RViz config in `lab/EK_Code/` support inspection. The saved measurements are examples from the original machine, not performance guarantees for a new environment.
