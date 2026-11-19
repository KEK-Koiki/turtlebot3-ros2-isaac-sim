# Engineering notes

The main engineering value in this project is connecting a supplied physics model to external ROS 2 consumers, then checking the complete system after packaging. The original report and logs preserve how the integration was diagnosed.

## Control and sensor configuration

The final stage contains clock, drive, lidar, camera, and odometry/TF graphs. A graph needs an execution signal, ROS context, data connections, and appropriate timestamps; having the nodes present is not sufficient.

| Component | Saved configuration |
| --- | --- |
| Differential drive | Wheel radius 0.025 m; track width 0.16 m |
| Controller limits | Linear 0.22 m/s; angular 1.0 rad/s |
| Wheel joints | `wheel_left_joint`, `wheel_right_joint` |
| Lidar | `Example_Rotary_2D`; mounted under `base_scan`; 10 simulation Hz |
| Camera | Under `base_footprint`; translation (0.1, 0, 0.2) m; XYZ rotation (90, 0, −90) degrees |
| RGB output | 640 × 480; `rgb8`; frame `Camera_1` |
| Odometry | Chassis `base_footprint`; parent `odom` |

Source: [checkpoint parameter inventory](../lab/EK_Logs/EK_checkpoint_parameters.json). The [Python graph builder](../lab/EK_Code/EK_build_parts_d_to_f.py) shows how lidar, camera, and odometry/TF nodes and connections were authored. It is a historical construction utility; the supplied final stages also contain later packaging and cleanup work.

## Separate the simulator and ROS Python environments

The report records an undefined `geometry_msgs` type-support symbol when system ROS bindings were used inside the Isaac Sim Python process. The fix moved subscriber validation into a separate process using the installed ROS 2 Jazzy environment.

This matters because the validation then exercises communication across the ROS bridge and DDS transport, rather than inspecting only internal graph connections.

Evidence: the original report records the error; the public [external validator](../lab/EK_Code/EK_validate_live.py) and [fresh-process results](../lab/EK_Logs/EK_fresh_process_validation.json) show the resulting validation approach.

## Validate the packaged stage after reopening

An earlier packaged stage was missing room prop references. The project collected the referenced geometry locally and made the lidar profile local. The robot geometry is shared through a USD sublayer, with recorded geometry and physics fingerprints unchanged for all six checkpoints.

The fresh-process run then received the sensor streams, odometry, clock, and seven TF edges. This is stronger evidence of packaging completeness than a screenshot of a scene that was already open before the files were moved.

Evidence: [geometry equivalence](../lab/EK_Logs/EK_geometry_equivalence.json) and [recorded fresh-process PASS](../lab/EK_Logs/EK_fresh_process_result.txt). The raw geometry and dependency collection utilities are not redistributed in this showcase.

The original packaged scene still references remote NVIDIA textures and an environment HDR. These dependencies and the raw geometry are not redistributed here.

## Resolve the camera orientation through output inspection

The submitted report documents conflicting camera rotations in E4 and the screenshot checklist. The final camera uses the E4 orientation `(90, 0, -90)` degrees. The retained rqt view shows the resulting forward-facing image, and the external TF capture includes `base_footprint → Camera_1`.

Evidence: [camera transform screenshot](../lab/EK_Evidence/EK_E_camera_transform.png), [camera output](../lab/EK_Evidence/EK_E_rqt_camera.png), and [received TF edges](../lab/EK_Logs/EK_fresh_process_validation.json).

## Handle render products and RTX sensing

Stale generated render products were removed before final packaging so the Action Graphs create fresh products on Play. Motion BVH was enabled for the rotary multi-tick RTX sensor. A reconstructed scene should preserve that setting; no simulator launcher is supplied in this showcase.

The legacy robot TF node can emit a camera pose warning even while a camera transform is received. The screenshot retains this warning. It is a known limitation, with received TF evidence, rather than a claim of a warning-free scene. Replacing the deprecated interface would be a future improvement that needs a new simulator run.

## Keep the performance claims bounded

The controller accepts forward and angular velocity commands, and the saved run records motion and a stop result. This is an integration check, not a controller accuracy study. The lidar sample contains 3,600 finite ranges, but that does not establish sensor accuracy or navigation performance.

The first run and fresh-process run have different wall-clock message rates. Rendering and load affect elapsed real time. Neither run establishes real-time execution or a guaranteed publishing rate on another GPU.

## Useful next extensions

Possible follow-on work includes replacing the legacy transform node, measuring simulation real-time factor, and adding AMCL plus a verified Nav2 goal. These are proposed extensions; they are not implemented or measured results in this portfolio.
