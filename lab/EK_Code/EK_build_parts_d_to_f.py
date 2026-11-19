"""Build and validate RAS545 Lab 3 checkpoints D, E, and F in Isaac Sim 6.0.1."""

from __future__ import annotations

import json
import sys
import os
import time
from pathlib import Path

os.environ.setdefault("ROS_DOMAIN_ID", "0")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")

from isaacsim import SimulationApp

LAB3 = Path(__file__).resolve().parents[1]
INPUT = LAB3 / "EK_Checkpoints" / "EK_Lab3_02_ros_drive.usda"
OUT_D = LAB3 / "EK_Checkpoints" / "EK_Lab3_03_lidar.usda"
OUT_E = LAB3 / "EK_Checkpoints" / "EK_Lab3_04_camera.usda"
OUT_F = LAB3 / "EK_Checkpoints" / "EK_Lab3_05_odometry_tf.usda"
RESULT_JSON = LAB3 / "EK_Logs" / "EK_parts_d_to_f_runtime_validation.json"

for output in (OUT_D, OUT_E, OUT_F):
    if output.exists() and "--rebuild" not in sys.argv:
        raise FileExistsError(f"Checkpoint exists: {output}; use --rebuild only if replacement is intended.")

print("LAB3 D-F BUILDER: starting Isaac Sim", flush=True)
kit = SimulationApp(
    launch_config={
        "headless": True,
        "sync_loads": True,
        "width": 640,
        "height": 480,
        "renderer": "RealTimePathTracing",
        "enable_motion_bvh": True,
        "limit_cpu_threads": 8,
    }
)

import omni
import omni.graph.core as og
import omni.kit.commands
import isaacsim.core.experimental.utils.app as app_utils
from isaacsim.core.experimental.utils.stage import is_stage_loading
from pxr import Gf, Sdf, UsdGeom, UsdUI


def update(count: int = 1) -> None:
    for _ in range(count):
        kit.update()


def wait_for_stage() -> None:
    while is_stage_loading():
        update()


def save_checkpoint(context: object, output: Path) -> None:
    update(3)
    if not context.save_as_stage(str(output)):
        raise RuntimeError(f"Failed to save {output}")
    update(3)
    print(f"PASS: saved {output}", flush=True)


def set_node_positions(stage, graph_path, positions):
    for name, position in positions.items():
        prim=stage.GetPrimAtPath(f"{graph_path}/{name}")
        if prim:
            ui=UsdUI.NodeGraphNodeAPI.Apply(prim)
            ui.CreatePosAttr().Set(Gf.Vec2f(*position))
            ui.CreateExpansionStateAttr().Set("open")


def require_prim(stage: object, path: str) -> object:
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        raise RuntimeError(f"Required prim is missing: {path}")
    return prim


def require_connection(stage: object, target: str, source: str) -> None:
    prim_path, attr_name = target.rsplit(".", 1)
    attr = require_prim(stage, prim_path).GetAttribute(attr_name)
    actual = {str(path) for path in attr.GetConnections()}
    if source not in actual:
        raise RuntimeError(f"Missing connection {source} -> {target}; actual={sorted(actual)}")


def clean_duplicate_clock(stage: object) -> None:
    # A duplicate graph accidentally authored at /Graph/ROS_Clock is not part of the handout.
    if stage.GetPrimAtPath("/Graph/ROS_Clock").IsValid():
        stage.RemovePrim("/Graph/ROS_Clock")
    graph_container = stage.GetPrimAtPath("/Graph")
    if graph_container.IsValid() and not list(graph_container.GetChildren()):
        stage.RemovePrim("/Graph")


def build_lidar(stage: object) -> None:
    lidar_path = "/turtlebot3_burger/base_scan/Lidar2D"
    graph_path = f"{lidar_path}/ROS_Lidar2D"
    if stage.GetPrimAtPath(lidar_path).IsValid() or stage.GetPrimAtPath(graph_path).IsValid():
        raise RuntimeError("Part D prims already exist in the Part C checkpoint")

    success, lidar_prim = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path=lidar_path,
        parent=None,
        config="Example_Rotary_2D",
        translation=Gf.Vec3d(0.0, 0.0, 0.0),
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
    )
    if not success or not lidar_prim or not lidar_prim.IsValid():
        raise RuntimeError("IsaacSensorCreateRtxLidar failed")

    xform = UsdGeom.Xformable(lidar_prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
    xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))
    xform.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(1.0, 1.0, 1.0))
    lidar_prim.GetAttribute("omni:sensor:tickRate").Set(10.0)
    if lidar_prim.HasAttribute("omni:sensor:Core:scanRateBaseHz"):
        lidar_prim.GetAttribute("omni:sensor:Core:scanRateBaseHz").Set(10.0)

    keys = og.Controller.Keys
    og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("RunOneSimulationFrame", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("PublishLaserScan", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "RunOneSimulationFrame.inputs:execIn"),
                ("RunOneSimulationFrame.outputs:step", "CreateRenderProduct.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "PublishLaserScan.inputs:execIn"),
                (
                    "CreateRenderProduct.outputs:renderProductPath",
                    "PublishLaserScan.inputs:renderProductPath",
                ),
                ("ROS2Context.outputs:context", "PublishLaserScan.inputs:context"),
            ],
            keys.SET_VALUES: [
                ("ROS2Context.inputs:domain_id", 0),
                ("ROS2Context.inputs:useDomainIDEnvVar", False),
                ("CreateRenderProduct.inputs:cameraPrim", [Sdf.Path(lidar_path)]),
                ("CreateRenderProduct.inputs:enabled", True),
                ("CreateRenderProduct.inputs:width", 640),
                ("CreateRenderProduct.inputs:height", 480),
                ("PublishLaserScan.inputs:type", "laser_scan"),
                ("PublishLaserScan.inputs:topicName", "/scan"),
                ("PublishLaserScan.inputs:frameId", "base_scan"),
                ("PublishLaserScan.inputs:enabled", True),
                ("PublishLaserScan.inputs:queueSize", 10),
                ("PublishLaserScan.inputs:useSystemTime", False),
                ("PublishLaserScan.inputs:resetSimulationTimeOnStop", False),
            ],
        },
    )
    set_node_positions(
        stage,
        graph_path,
        {
            "OnPlaybackTick": (-600, -80),
            "ROS2Context": (-600, 180),
            "RunOneSimulationFrame": (-320, -80),
            "CreateRenderProduct": (-20, -80),
            "PublishLaserScan": (330, -80),
        },
    )
    update(3)

    for name in (
        "OnPlaybackTick",
        "ROS2Context",
        "RunOneSimulationFrame",
        "CreateRenderProduct",
        "PublishLaserScan",
    ):
        require_prim(stage, f"{graph_path}/{name}")
    require_connection(
        stage,
        f"{graph_path}/PublishLaserScan.inputs:renderProductPath",
        f"{graph_path}/CreateRenderProduct.outputs:renderProductPath",
    )
    print("PASS: Part D Lidar2D and ROS_Lidar2D graph created", flush=True)


def build_camera(stage: object) -> None:
    camera_path = "/turtlebot3_burger/base_footprint/Camera_1"
    graph_path = f"{camera_path}/ROS_Camera"
    if stage.GetPrimAtPath(camera_path).IsValid() or stage.GetPrimAtPath(graph_path).IsValid():
        raise RuntimeError("Part E prims already exist in the Part D checkpoint")

    camera = UsdGeom.Camera.Define(stage, camera_path)
    camera_prim = camera.GetPrim()
    xform = UsdGeom.Xformable(camera_prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(0.1, 0.0, 0.2))
    xform.AddRotateXYZOp().Set(Gf.Vec3f(90.0, 0.0, -90.0))
    xform.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))
    camera.GetFocalLengthAttr().Set(18.0)
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 100.0))
    camera_prim.CreateAttribute("omni:sensor:tickRate", Sdf.ValueTypeNames.Float, False).Set(10.0)

    keys = og.Controller.Keys
    og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("RunOneSimulationFrame", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("PublishImage", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishCameraInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "RunOneSimulationFrame.inputs:execIn"),
                ("RunOneSimulationFrame.outputs:step", "CreateRenderProduct.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "PublishImage.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "PublishCameraInfo.inputs:execIn"),
                ("CreateRenderProduct.outputs:renderProductPath", "PublishImage.inputs:renderProductPath"),
                (
                    "CreateRenderProduct.outputs:renderProductPath",
                    "PublishCameraInfo.inputs:renderProductPath",
                ),
                ("ROS2Context.outputs:context", "PublishImage.inputs:context"),
                ("ROS2Context.outputs:context", "PublishCameraInfo.inputs:context"),
            ],
            keys.SET_VALUES: [
                ("ROS2Context.inputs:domain_id", 0),
                ("ROS2Context.inputs:useDomainIDEnvVar", False),
                ("CreateRenderProduct.inputs:cameraPrim", [Sdf.Path(camera_path)]),
                ("CreateRenderProduct.inputs:enabled", True),
                ("CreateRenderProduct.inputs:width", 640),
                ("CreateRenderProduct.inputs:height", 480),
                ("PublishImage.inputs:type", "rgb"),
                ("PublishImage.inputs:topicName", "/camera/image_raw"),
                ("PublishImage.inputs:frameId", "Camera_1"),
                ("PublishImage.inputs:enabled", True),
                ("PublishImage.inputs:queueSize", 10),
                ("PublishImage.inputs:useSystemTime", False),
                ("PublishImage.inputs:resetSimulationTimeOnStop", False),
                ("PublishCameraInfo.inputs:topicName", "/camera/camera_info"),
                ("PublishCameraInfo.inputs:frameId", "Camera_1"),
                ("PublishCameraInfo.inputs:enabled", True),
                ("PublishCameraInfo.inputs:queueSize", 10),
                ("PublishCameraInfo.inputs:useSystemTime", False),
                ("PublishCameraInfo.inputs:resetSimulationTimeOnStop", False),
            ],
        },
    )
    set_node_positions(
        stage,
        graph_path,
        {
            "OnPlaybackTick": (-650, -80),
            "ROS2Context": (-650, 220),
            "RunOneSimulationFrame": (-360, -80),
            "CreateRenderProduct": (-50, -80),
            "PublishImage": (310, -160),
            "PublishCameraInfo": (310, 180),
        },
    )
    update(3)
    for name in (
        "OnPlaybackTick",
        "ROS2Context",
        "RunOneSimulationFrame",
        "CreateRenderProduct",
        "PublishImage",
        "PublishCameraInfo",
    ):
        require_prim(stage, f"{graph_path}/{name}")
    require_connection(
        stage,
        f"{graph_path}/PublishImage.inputs:renderProductPath",
        f"{graph_path}/CreateRenderProduct.outputs:renderProductPath",
    )
    require_connection(
        stage,
        f"{graph_path}/PublishCameraInfo.inputs:renderProductPath",
        f"{graph_path}/CreateRenderProduct.outputs:renderProductPath",
    )
    print("PASS: Part E Camera_1 and ROS_Camera graph created", flush=True)


def build_odom_tf(stage: object) -> None:
    graph_path = "/turtlebot3_burger/ROS_OdomTF"
    chassis = "/turtlebot3_burger/base_footprint"
    if stage.GetPrimAtPath(graph_path).IsValid():
        raise RuntimeError("Part F graph already exists in the Part E checkpoint")

    targets = [
        Sdf.Path("/turtlebot3_burger/base_link"),
        Sdf.Path("/turtlebot3_burger/base_scan"),
        Sdf.Path("/turtlebot3_burger/caster_back_link"),
        Sdf.Path("/turtlebot3_burger/wheel_left_link"),
        Sdf.Path("/turtlebot3_burger/wheel_right_link"),
        Sdf.Path("/turtlebot3_burger/base_footprint/Camera_1"),
    ]
    keys = og.Controller.Keys
    og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimulationTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("PublishOdomTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                ("PublishRobotTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishRobotTF.inputs:execIn"),
                ("ROS2Context.outputs:context", "PublishOdometry.inputs:context"),
                ("ROS2Context.outputs:context", "PublishOdomTF.inputs:context"),
                ("ROS2Context.outputs:context", "PublishRobotTF.inputs:context"),
                ("ReadSimulationTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),
                ("ReadSimulationTime.outputs:simulationTime", "PublishOdomTF.inputs:timeStamp"),
                ("ReadSimulationTime.outputs:simulationTime", "PublishRobotTF.inputs:timeStamp"),
                ("ComputeOdometry.outputs:execOut", "PublishOdometry.inputs:execIn"),
                ("ComputeOdometry.outputs:execOut", "PublishOdomTF.inputs:execIn"),
                ("ComputeOdometry.outputs:position", "PublishOdometry.inputs:position"),
                ("ComputeOdometry.outputs:orientation", "PublishOdometry.inputs:orientation"),
                ("ComputeOdometry.outputs:linearVelocity", "PublishOdometry.inputs:linearVelocity"),
                ("ComputeOdometry.outputs:angularVelocity", "PublishOdometry.inputs:angularVelocity"),
                ("ComputeOdometry.outputs:position", "PublishOdomTF.inputs:translation"),
                ("ComputeOdometry.outputs:orientation", "PublishOdomTF.inputs:rotation"),
            ],
            keys.SET_VALUES: [
                ("ROS2Context.inputs:domain_id", 0),
                ("ROS2Context.inputs:useDomainIDEnvVar", False),
                ("ReadSimulationTime.inputs:resetOnStop", False),
                ("ComputeOdometry.inputs:chassisPrim", [Sdf.Path(chassis)]),
                ("PublishOdometry.inputs:chassisFrameId", "base_footprint"),
                ("PublishOdometry.inputs:odomFrameId", "odom"),
                ("PublishOdometry.inputs:topicName", "/odom"),
                ("PublishOdometry.inputs:queueSize", 10),
                ("PublishOdometry.inputs:publishRawVelocities", False),
                ("PublishOdomTF.inputs:parentFrameId", "odom"),
                ("PublishOdomTF.inputs:childFrameId", "base_footprint"),
                ("PublishOdomTF.inputs:topicName", "/tf"),
                ("PublishOdomTF.inputs:queueSize", 10),
                ("PublishOdomTF.inputs:staticPublisher", False),
                ("PublishRobotTF.inputs:parentPrim", [Sdf.Path(chassis)]),
                ("PublishRobotTF.inputs:targetPrims", targets),
                ("PublishRobotTF.inputs:topicName", "/tf"),
                ("PublishRobotTF.inputs:queueSize", 10),
                ("PublishRobotTF.inputs:staticPublisher", False),
            ],
        },
    )
    set_node_positions(
        stage,
        graph_path,
        {
            "OnPlaybackTick": (-700, -160),
            "ROS2Context": (-700, 100),
            "ReadSimulationTime": (-700, 350),
            "ComputeOdometry": (-360, -160),
            "PublishOdometry": (40, -300),
            "PublishOdomTF": (40, 40),
            "PublishRobotTF": (40, 380),
        },
    )
    update(3)
    for name in (
        "OnPlaybackTick",
        "ROS2Context",
        "ReadSimulationTime",
        "ComputeOdometry",
        "PublishOdometry",
        "PublishOdomTF",
        "PublishRobotTF",
    ):
        require_prim(stage, f"{graph_path}/{name}")
    if stage.GetPrimAtPath(f"{graph_path}/ComputeTransformTree").IsValid():
        raise RuntimeError("Forbidden Isaac Compute Transform Tree node exists")
    require_connection(
        stage,
        f"{graph_path}/PublishOdomTF.inputs:translation",
        f"{graph_path}/ComputeOdometry.outputs:position",
    )
    print("PASS: Part F ROS_OdomTF graph created without world -> odom", flush=True)


def runtime_validate():
    """Validate through a separate system ROS process to avoid mixed type support libraries."""
    import subprocess
    env=os.environ.copy()
    env['PYTHONPATH']='/opt/ros/jazzy/lib/python3.12/site-packages'
    env['LD_LIBRARY_PATH']='/opt/ros/jazzy/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu'
    env['PATH']='/usr/bin:/bin:/opt/ros/jazzy/bin'
    proc=subprocess.Popen(['/usr/bin/python3',str(LAB3/'EK_Code/EK_validate_live.py')],env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    timeline=omni.timeline.get_timeline_interface();timeline.play()
    try:
        deadline=time.monotonic()+65
        while proc.poll() is None and time.monotonic()<deadline:
            update();time.sleep(.005)
        if proc.poll() is None:
            proc.terminate();raise TimeoutError('ROS validation timed out')
        output=proc.communicate()[0]
        print(output,flush=True)
        if proc.returncode:raise RuntimeError(output)
        result=json.loads((LAB3/'EK_Logs/EK_live_validation.json').read_text())
        if result['missing'] or result['forbidden_world_odom']:
            raise RuntimeError('Validation failed: '+str(result))
        return result
    finally:
        timeline.stop()
        if proc.poll() is None:proc.terminate()


try:
    print("LAB3 D-F BUILDER: enabling ROS2 and RTX extensions", flush=True)
    for extension in (
        "isaacsim.ros2.bridge",
        "isaacsim.sensors.rtx",
        "isaacsim.sensors.rtx.nodes",
        "isaacsim.sensors.experimental.rtx",
    ):
        enabled = app_utils.enable_extension(extension)
        print(f"  extension {extension}: {enabled}", flush=True)
        update(2)
    update(8)

    context = omni.usd.get_context()
    print(f"LAB3 D-F BUILDER: opening {INPUT}", flush=True)
    context.open_stage(str(INPUT))
    update(4)
    wait_for_stage()
    stage = context.get_stage()
    clean_duplicate_clock(stage)

    build_lidar(stage)
    save_checkpoint(context, OUT_D)
    stage = context.get_stage()

    build_camera(stage)
    save_checkpoint(context, OUT_E)
    stage = context.get_stage()

    build_odom_tf(stage)
    save_checkpoint(context, OUT_F)

    print("LAB3 D-F BUILDER: running ROS 2 validation", flush=True)
    validation = runtime_validate()
    RESULT_JSON.write_text(json.dumps(validation, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(validation, indent=2), flush=True)
    print("PASS: Parts D, E, and F runtime validation succeeded", flush=True)
except BaseException:
    import traceback
    traceback.print_exc()
    Path(str(LAB3 / "EK_Logs" / "EK_build_exception.txt")).write_text(traceback.format_exc())
finally:
    kit.close()
