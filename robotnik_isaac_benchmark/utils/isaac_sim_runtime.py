"""Shared Isaac Sim stage setup for GUI and headless benchmark launchers."""

import time
from pathlib import Path


STAGE_LOAD_TIMEOUT = 120.0
LIDAR_CONFIG = "OS1"
LIDAR_VARIANT = "OS1_REV6_32ch10hz2048res"
CAMERA_A_TICK_RATE_HZ = 25.0
CAMERA_B_TICK_RATE_HZ = 25.0
CAMERA_A_RESOLUTION = (1920, 1080)
CAMERA_B_RESOLUTION = (1280, 720)
LIDAR_CHANNELS = 16
LIDAR_HORIZONTAL_SAMPLES = 1800
LIDAR_POINTS_PER_SCAN = LIDAR_CHANNELS * LIDAR_HORIZONTAL_SAMPLES
LIDAR_SCAN_RATE_HZ = 10.0
IMU_RATE_HZ = 100.0
PHYSICS_RATE_HZ = 50.0


def configure_realtime_pacing():
    """Keep Isaac's simulation, timeline, and Kit loop at the 50 Hz benchmark rate.

    The base Kit experience and standalone Python runner do not enable Kit's
    main-loop rate limiter by default.  Without it, a light headless scene can
    advance the 50 Hz PhysX clock faster than wall time.  Use Kit's native
    rate limiter rather than sleeping in the benchmark launcher so GUI and
    headless runs share the same real-time ceiling.
    """
    import carb.settings
    from isaacsim.core.rendering_manager import RenderingManager
    from isaacsim.core.simulation_manager import SimulationManager

    dt = 1.0 / PHYSICS_RATE_HZ
    settings = carb.settings.get_settings()
    settings.set("/app/runLoops/main/rateLimitEnabled", True)
    settings.set("/app/runLoops/main/rateLimitFrequency", PHYSICS_RATE_HZ)
    settings.set("/app/runLoops/main/rateLimitUseBusyLoop", False)

    # Isaac Sim has independent physics, timeline and application-loop
    # clocks.  Configure them together before play; changing only the Kit
    # loop frequency would make ROS publication cadence depend on the runner.
    SimulationManager.setup_simulation(dt=dt)
    RenderingManager.set_dt(dt)
    print(
        "Isaac real-time pacing enabled: "
        f"physics/timeline/run-loop={PHYSICS_RATE_HZ:g} Hz"
    )


def parse_runtime_config(script_file):
    """Validate environment-driven launcher configuration and resolve assets."""
    import os

    try:
        num_robots = int(os.environ.get("NUM_ROBOTS", "1"))
    except ValueError as exc:
        raise ValueError("NUM_ROBOTS must be an integer between 1 and 3") from exc
    if num_robots not in (1, 2, 3):
        raise ValueError("NUM_ROBOTS must be 1, 2, or 3")

    world_file = os.environ.get("WORLD_FILE", "empty_world.usd")
    if Path(world_file).name != world_file or Path(world_file).suffix != ".usd":
        raise ValueError("WORLD_FILE must be the name of a .usd file in the worlds directory")

    package_dir = Path(script_file).resolve().parent.parent
    world_path = package_dir / "worlds" / world_file
    model_path = package_dir / "models" / "rbwatcher.usd"
    if not world_path.is_file():
        raise FileNotFoundError(f"World USD does not exist: {world_path}")
    if not model_path.is_file():
        raise FileNotFoundError(f"Robot USD does not exist: {model_path}")

    return num_robots, world_path, model_path


def wait_for_stage(app, description, timeout=STAGE_LOAD_TIMEOUT):
    """Drive Kit until all pending USD loads finish, with a bounded wait."""
    from isaacsim.core.utils.stage import is_stage_loading

    deadline = time.monotonic() + timeout
    app.update()
    app.update()
    while app.is_running() and is_stage_loading():
        if time.monotonic() >= deadline:
            raise TimeoutError(f"Isaac Sim did not finish loading {description} in {timeout:g} s")
        app.update()
    if not app.is_running():
        raise RuntimeError(f"Isaac Sim stopped while loading {description}")


def open_world(app, world_path):
    """Open a benchmark world and return the fully loaded stage."""
    from omni.usd import get_context

    context = get_context()
    if not context.open_stage(str(world_path)):
        raise RuntimeError(f"Isaac Sim rejected world USD: {world_path}")
    wait_for_stage(app, world_path.name)
    stage = context.get_stage()
    if stage is None:
        raise RuntimeError(f"Isaac Sim did not create a stage for {world_path}")
    return stage


def _set_robot_namespace(robot_root, robot_name):
    """Namespace robot interfaces while keeping the shared TF topics global."""
    from pxr import Usd

    for prim in Usd.PrimRange(robot_root):
        if "/Graph/ROS2_Clock/" in str(prim.GetPath()):
            continue

        # RViz and tf2 subscribe to the canonical global topics.  An absolute
        # topic name is required here because the asset's TF publishers also
        # receive the per-robot node namespace below.  Leaving their default
        # relative names moves the complete world-rooted tree to
        # /<robot>/tf[_static], where the benchmark RViz never sees it.
        is_transform_publisher = "transform_tree" in prim.GetName()
        topic = prim.GetAttribute("inputs:topicName")
        if topic.IsValid() and is_transform_publisher:
            topic_name = topic.Get()
            # Isaac uses a null authored value for the default relative `tf`
            # topic.  The static publishers in this asset already author
            # `/tf_static`, so preserve that value while making the default
            # dynamic topic absolute as well.
            topic.Set(f"/{topic_name}" if topic_name else "/tf")

        namespace = prim.GetAttribute("inputs:nodeNamespace")
        if namespace.IsValid():
            # Keeping the TF nodes outside the robot namespace also covers
            # Isaac nodes whose topicName input is null and therefore cannot
            # be made absolute reliably through a referenced USD override.
            namespace.Set("" if is_transform_publisher else robot_name)


def _deactivate_legacy_sensor_graphs(stage, body_path, robot_index):
    """Disable sensor graphs authored for pre-6.0 RTX/Replicator APIs."""
    from pxr import Usd

    for graph_name in ("front_camera", "top_camera", "lidar"):
        graph = stage.GetPrimAtPath(f"{body_path}/Graph/{graph_name}")
        if graph.IsValid():
            # Disabling the nodes as well as the USD graph prevents an already
            # discovered OmniGraph from retaining an enabled helper in cache.
            for node in Usd.PrimRange(graph):
                enabled = node.GetAttribute("inputs:enabled")
                if enabled.IsValid():
                    enabled.Set(False)
            graph.SetActive(False)

    legacy_lidar = stage.GetPrimAtPath(
        f"{body_path}/base_link/laser_link/helios5515_sensor"
    )
    if legacy_lidar.IsValid():
        legacy_lidar.SetActive(False)

    clock_graph = stage.GetPrimAtPath(f"{body_path}/Graph/ROS2_Clock")
    if robot_index == 0:
        # RTF is only meaningful when the global clock is authored by the Kit
        # simulation timeline.  Fail startup instead of silently accepting a
        # wall-clock fallback if an asset revision drops this graph.
        if not clock_graph.IsValid():
            raise RuntimeError(f"Isaac Sim ROS2_Clock graph is missing: {body_path}")
    else:
        if clock_graph.IsValid():
            for node in Usd.PrimRange(clock_graph):
                enabled = node.GetAttribute("inputs:enabled")
                if enabled.IsValid():
                    enabled.Set(False)
            clock_graph.SetActive(False)


def _use_explicit_pinhole_projection(stage, body_path):
    """Make projection and camera publish cadence explicit and deterministic."""
    for link_name, tick_rate_hz in (
        ("front_camera_link", CAMERA_A_TICK_RATE_HZ),
        ("top_camera_link", CAMERA_B_TICK_RATE_HZ),
    ):
        camera = stage.GetPrimAtPath(
            f"{body_path}/base_link/{link_name}/Camera_OmniVision_OV9782_Color"
        )
        if not camera.IsValid():
            raise RuntimeError(f"Camera prim is missing: {camera.GetPath()}")
        projection = camera.GetAttribute("cameraProjectionType")
        if projection.IsValid():
            projection.Set("pinhole")
        # Isaac Sim 6 defaults a camera without OmniSensorAPI to autotrigger on
        # every simulation frame. That makes the ROS workload depend on the
        # renderer's current FPS and creates a feedback loop in multi-robot
        # cases. The supported 6.0 control is omni:sensor:tickRate.
        camera.ApplyAPI("OmniSensorAPI")
        tick_rate = camera.GetAttribute("omni:sensor:tickRate")
        if not tick_rate.IsValid():
            raise RuntimeError(f"Camera tick-rate attribute is missing: {camera.GetPath()}")
        tick_rate.Set(tick_rate_hz)


def _create_camera_publisher(
    app, graph_path, camera_path, namespace, topic_prefix, frame_id,
    width, height, enable_depth,
):
    """Create one current ROS camera pipeline synchronously."""
    import omni.graph.core as og
    import usdrt.Sdf

    keys = og.Controller.Keys
    nodes = [
        ("on_tick", "omni.graph.action.OnTick"),
        ("run_once", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
        ("create_render_product", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
        ("publish_image", "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ("publish_camera_info", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
    ]
    connections = [
        ("on_tick.outputs:tick", "run_once.inputs:execIn"),
        ("run_once.outputs:step", "create_render_product.inputs:execIn"),
        ("create_render_product.outputs:execOut", "publish_image.inputs:execIn"),
        ("create_render_product.outputs:execOut", "publish_camera_info.inputs:execIn"),
        (
            "create_render_product.outputs:renderProductPath",
            "publish_image.inputs:renderProductPath",
        ),
        (
            "create_render_product.outputs:renderProductPath",
            "publish_camera_info.inputs:renderProductPath",
        ),
    ]
    values = [
        ("create_render_product.inputs:cameraPrim", [usdrt.Sdf.Path(camera_path)]),
        ("create_render_product.inputs:width", width),
        ("create_render_product.inputs:height", height),
        ("publish_image.inputs:nodeNamespace", namespace),
        ("publish_image.inputs:frameId", frame_id),
        ("publish_image.inputs:topicName", f"{topic_prefix}/image_raw"),
        ("publish_image.inputs:type", "rgb"),
        ("publish_camera_info.inputs:nodeNamespace", namespace),
        ("publish_camera_info.inputs:frameId", frame_id),
        ("publish_camera_info.inputs:topicName", f"{topic_prefix}/camera_info"),
    ]
    if enable_depth:
        nodes.append(("publish_depth", "isaacsim.ros2.bridge.ROS2CameraHelper"))
        connections.extend([
            ("create_render_product.outputs:execOut", "publish_depth.inputs:execIn"),
            (
                "create_render_product.outputs:renderProductPath",
                "publish_depth.inputs:renderProductPath",
            ),
        ])
        values.extend([
            ("publish_depth.inputs:nodeNamespace", namespace),
            ("publish_depth.inputs:frameId", frame_id),
            ("publish_depth.inputs:topicName", f"{topic_prefix.replace('/color', '/depth')}/image_raw"),
            ("publish_depth.inputs:type", "depth"),
        ])

    graph, _, _, _ = og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: nodes,
            keys.CONNECT: connections,
            keys.SET_VALUES: values,
        },
    )

    # Isaac's helper nodes construct Replicator/SDG writers.  Initializing each
    # graph once and advancing Kit before creating the next graph avoids the
    # concurrent writer construction seen in the crash log.
    og.Controller.evaluate_sync(graph)
    app.update()
    app.update()


def _resize_lidar_emitter_arrays(prim, source_emitters, target_emitters):
    """Resize the profile's per-emitter arrays to match its configured count."""
    emitter_prefix = "omni:sensor:Core:emitterState:"
    for attribute in prim.GetAttributes():
        name = attribute.GetName()
        value = attribute.Get()
        if (
            name.startswith(emitter_prefix)
            and value is not None
            and hasattr(value, "__len__")
            and len(value) == source_emitters
        ):
            attribute.Set(value[:target_emitters])


def _validate_lidar_emitter_arrays(prim, target_emitters):
    """Reject a lidar profile whose declared emitters and arrays disagree."""
    emitter_prefix = "omni:sensor:Core:emitterState:"
    for attribute in prim.GetAttributes():
        name = attribute.GetName()
        value = attribute.Get()
        if (
            name.startswith(emitter_prefix)
            and value is not None
            and hasattr(value, "__len__")
            and len(value) > 1
            and len(value) != target_emitters
        ):
            raise RuntimeError(
                "Isaac 6 OmniLidar emitter array does not match the configured "
                f"channel count: {name} has {len(value)}, expected {target_emitters}"
            )


def _create_static_tf_graph(
    app, graph_path, parent_frame, child_frame, translation, rotation
):
    """Publish one static transform through Isaac's ROS 2 bridge."""
    import omni.graph.core as og

    keys = og.Controller.Keys
    graph, _, _, _ = og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            keys.CREATE_NODES: [
                ("on_tick", "omni.graph.action.OnTick"),
                ("read_sim_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("publish_tf", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ],
            keys.CONNECT: [
                ("on_tick.outputs:tick", "publish_tf.inputs:execIn"),
                ("read_sim_time.outputs:simulationTime", "publish_tf.inputs:timeStamp"),
            ],
            keys.SET_VALUES: [
                ("publish_tf.inputs:topicName", "/tf_static"),
                ("publish_tf.inputs:parentFrameId", parent_frame),
                ("publish_tf.inputs:childFrameId", child_frame),
                (
                    "publish_tf.inputs:translation",
                    [float(value) for value in translation],
                ),
                (
                    "publish_tf.inputs:rotation",
                    [float(value) for value in rotation],
                ),
                ("publish_tf.inputs:staticPublisher", True),
            ],
        },
    )
    og.Controller.evaluate_sync(graph)
    app.update()
    app.update()
    return graph


def _create_static_laser_tf_graph(app, stage, body_path, graph_path):
    """Publish the laser link's authored pose through Isaac's ROS 2 bridge."""
    from pxr import Usd, UsdGeom

    base_link_path = f"{body_path}/base_link"
    laser_link_path = f"{base_link_path}/laser_link"
    base_link = stage.GetPrimAtPath(base_link_path)
    laser_link = stage.GetPrimAtPath(laser_link_path)
    if not base_link.IsValid() or not laser_link.IsValid():
        raise RuntimeError(
            "Cannot publish laser TF: missing USD prim(s) "
            f"{base_link_path}, {laser_link_path}"
        )

    # Derive the relative pose from the composed stage.  Keeping the USD as
    # the source of truth avoids duplicating the sensor mounting transform.
    time_code = Usd.TimeCode.Default()
    base_world = UsdGeom.Xformable(base_link).ComputeLocalToWorldTransform(time_code)
    laser_world = UsdGeom.Xformable(laser_link).ComputeLocalToWorldTransform(time_code)
    base_to_laser = base_world.GetInverse() * laser_world
    translation = base_to_laser.ExtractTranslation()
    quaternion = base_to_laser.ExtractRotationQuat()
    imaginary = quaternion.GetImaginary()
    rotation = [
        imaginary[0], imaginary[1], imaginary[2], quaternion.GetReal()
    ]
    return _create_static_tf_graph(
        app, graph_path, "base_link", "laser_link", translation, rotation
    )


def _create_lidar_publisher(app, body_path, namespace):
    """Create a 16 x 1800, 10 Hz OmniLidar and its ROS writer."""
    from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor

    lidar_path = f"{body_path}/base_link/laser_link/benchmark_lidar"
    lidar = Lidar.create(
        path=lidar_path,
        config=LIDAR_CONFIG,
        variant=LIDAR_VARIANT,
        tick_rate=LIDAR_SCAN_RATE_HZ,
        translations=[[0.12507138640901638, 0.0, 3.131290466362735e-06]],
    )
    # Profile USDs reference an Xform at ``lidar_path`` and place the actual
    # OmniLidar below it.  ``Lidar.prims`` identifies that sensor prim.
    prim = lidar.prims[0]
    # The OS1 profile used as a base has 32 emitters.  Changing only
    # ``numberOfEmitters`` leaves the profile's per-emitter calibration arrays
    # at length 32, which makes LidarCore reject the model and results in a
    # advertised-but-empty PointCloud2 topic.  Resize every per-emitter array
    # before changing the declared count so the profile remains internally
    # consistent with the canonical 8-channel benchmark configuration.
    source_emitters = prim.GetAttribute("omni:sensor:Core:numberOfEmitters").Get()
    if not isinstance(source_emitters, int) or source_emitters < LIDAR_CHANNELS:
        raise RuntimeError(
            "Isaac 6 OmniLidar profile has an invalid emitter count: "
            f"{source_emitters!r}"
        )
    _resize_lidar_emitter_arrays(prim, source_emitters, LIDAR_CHANNELS)
    attributes = {
        "omni:sensor:Core:numberOfChannels": LIDAR_CHANNELS,
        "omni:sensor:Core:numberOfEmitters": LIDAR_CHANNELS,
        "omni:sensor:Core:patternFiringRateHz": float(
            LIDAR_HORIZONTAL_SAMPLES * LIDAR_SCAN_RATE_HZ
        ),
        "omni:sensor:Core:scanRateBaseHz": LIDAR_SCAN_RATE_HZ,
        "omni:sensor:Core:emitterState:s001:azimuthDeg": [0.0] * LIDAR_CHANNELS,
        "omni:sensor:Core:emitterState:s001:elevationDeg": [
            -15.0 + 30.0 * channel / (LIDAR_CHANNELS - 1)
            for channel in range(LIDAR_CHANNELS)
        ],
        "omni:sensor:Core:emitterState:s001:fireTimeNs": [0.0] * LIDAR_CHANNELS,
    }
    for name, value in attributes.items():
        attribute = prim.GetAttribute(name)
        if not attribute.IsValid():
            raise RuntimeError(f"Isaac 6 OmniLidar attribute is missing: {name}")
        attribute.Set(value)
    _validate_lidar_emitter_arrays(prim, LIDAR_CHANNELS)
    sensor = LidarSensor(lidar, annotators=[])
    sensor.attach_writer(
        "RtxLidarROS2PublishPointCloud",
        nodeNamespace=namespace,
        topicName="top_3d_laser/points",
        frameId="laser_link",
    )
    app.update()
    return sensor


def _configure_imu_profile(stage, body_path):
    """Set the existing IMU sensor/ROS graph to 100 Hz and fail on asset drift."""
    from pxr import Usd

    # IsaacImuSensor uses a period in seconds (``sensorPeriod``), whereas the
    # OmniGraph publisher variants use rates in Hz.  Keep the units explicit:
    # the rbwatcher asset currently authors the former with -1 (every step).
    cadence_attributes = {
        "omni:sensor:tickRate": IMU_RATE_HZ,
        "inputs:frequency": IMU_RATE_HZ,
        "inputs:publishRateHz": IMU_RATE_HZ,
        "inputs:updateRate": IMU_RATE_HZ,
        "sensorPeriod": 1.0 / IMU_RATE_HZ,
    }
    changed = []
    body = stage.GetPrimAtPath(body_path)
    for prim in Usd.PrimRange(body):
        if "imu" not in prim.GetName().lower():
            continue
        for attribute_name, value in cadence_attributes.items():
            attribute = prim.GetAttribute(attribute_name)
            if attribute.IsValid():
                attribute.Set(value)
                changed.append(f"{prim.GetPath()}.{attribute_name}")
    if not changed:
        raise RuntimeError(f"No configurable IMU cadence was found below {body_path}")


def _configure_physics_rate(stage):
    """Set the benchmark physics clock to 50 Hz (20 ms per step)."""
    changed = []
    for prim in stage.Traverse():
        attribute = prim.GetAttribute("physxScene:timeStepsPerSecond")
        if attribute.IsValid():
            attribute.Set(PHYSICS_RATE_HZ)
            changed.append(str(prim.GetPath()))
    if not changed:
        raise RuntimeError("No PhysX scene exposes physxScene:timeStepsPerSecond")


def spawn_and_configure_robots(app, stage, model_path, num_robots):
    """Spawn robots, namespace ROS interfaces, and initialize sensors serially."""
    from pxr import Gf, UsdGeom

    _configure_physics_rate(stage)
    robot_layout = []
    for index in range(num_robots):
        robot_name = f"rbwatcher_{index + 1}"
        container_path = f"/World/{robot_name}"
        container = stage.DefinePrim(container_path, "Xform")
        reference_path = f"{container_path}/rbwatcher"
        reference = stage.DefinePrim(reference_path, "Xform")
        if not reference.GetReferences().AddReference(str(model_path)):
            raise RuntimeError(f"Could not reference robot USD at {reference_path}")

        xform = UsdGeom.Xformable(container)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(0.0, float(index), 0.2))
        body_path = f"{reference_path}/robotnik_rbwatcher/rbwatcher"
        body = stage.GetPrimAtPath(body_path)
        if not body.IsValid():
            raise RuntimeError(
                f"Robot model root was not composed synchronously: {body_path}"
            )

        # Apply overrides before the first app.update().  Once Kit discovers an
        # automatic graph it may keep its helper nodes alive even if its USD
        # prim is later deactivated.
        _set_robot_namespace(body, robot_name)
        _deactivate_legacy_sensor_graphs(stage, body_path, index)
        _use_explicit_pinhole_projection(stage, body_path)
        _configure_imu_profile(stage, body_path)
        robot_layout.append((index, robot_name, container_path, body_path))

    wait_for_stage(app, f"{num_robots} robot reference(s)")

    for _index, _robot_name, _container_path, body_path in robot_layout:
        body = stage.GetPrimAtPath(body_path)
        if not body.IsValid():
            raise RuntimeError(f"Robot model root is missing after composition: {body_path}")

    # The benchmark RViz uses `world` as its fixed frame, while the imported
    # controller's dynamic tree starts at `odom`.  Publish the missing common
    # root explicitly; static TF is independent of simulation time.
    sensor_handles = [
        _create_static_tf_graph(
            app,
            "/World/benchmark_world_tf",
            "world",
            "odom",
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        )
    ]
    for _index, robot_name, container_path, body_path in robot_layout:
        graph_root = f"{container_path}/benchmark_sensor_graphs"
        stage.DefinePrim(graph_root, "Scope")
        front_camera = (
            f"{body_path}/base_link/front_camera_link/Camera_OmniVision_OV9782_Color"
        )
        top_camera = (
            f"{body_path}/base_link/top_camera_link/Camera_OmniVision_OV9782_Color"
        )
        _create_camera_publisher(
            app,
            f"{graph_root}/front_camera",
            front_camera,
            robot_name,
            "front_rgbd_camera/color",
            "front_camera_link",
            *CAMERA_A_RESOLUTION,
            False,
        )
        sensor_handles.append(
            _create_static_laser_tf_graph(
                app, stage, body_path, f"{graph_root}/laser_tf"
            )
        )
        _create_camera_publisher(
            app,
            f"{graph_root}/top_camera",
            top_camera,
            robot_name,
            "top_rgbd_camera/color",
            "top_camera_link",
            *CAMERA_B_RESOLUTION,
            True,
        )
        sensor_handles.append(_create_lidar_publisher(app, body_path, robot_name))

    print(
        f"Isaac stage ready: {num_robots} robot(s), namespaced sensor topics, "
        f"front RGB {CAMERA_A_RESOLUTION[0]}x{CAMERA_A_RESOLUTION[1]} at "
        f"{CAMERA_A_TICK_RATE_HZ:g} Hz, top RGB-D "
        f"{CAMERA_B_RESOLUTION[0]}x{CAMERA_B_RESOLUTION[1]} at "
        f"{CAMERA_B_TICK_RATE_HZ:g} Hz, {LIDAR_POINTS_PER_SCAN} points/scan "
        f"at {LIDAR_SCAN_RATE_HZ:g} Hz, {IMU_RATE_HZ:g} Hz IMU, one global /clock publisher"
    )
    return sensor_handles
