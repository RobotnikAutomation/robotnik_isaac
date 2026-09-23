import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _benchmark_gpu_index():
    """Return the single GPU reserved for a reproducible Isaac benchmark."""
    value = os.environ.get("ISAAC_BENCHMARK_GPU", "0")
    try:
        index = int(value)
    except ValueError as exc:
        raise ValueError(
            "ISAAC_BENCHMARK_GPU must be a non-negative integer"
        ) from exc
    if index < 0:
        raise ValueError("ISAAC_BENCHMARK_GPU must be a non-negative integer")
    return index


def _rviz_environment():
    """Keep Snap-injected desktop libraries out of the system RViz process."""
    environment = {}
    snap_desktop_variables = (
        "GDK_PIXBUF_MODULEDIR",
        "GDK_PIXBUF_MODULE_FILE",
        "GIO_LAUNCHED_DESKTOP_FILE",
        "GIO_MODULE_DIR",
        "GSETTINGS_SCHEMA_DIR",
        "GTK_EXE_PREFIX",
        "GTK_IM_MODULE_FILE",
        "GTK_MODULES",
        "GTK_PATH",
        "LOCPATH",
    )
    for variable in snap_desktop_variables:
        if "/snap/" in os.environ.get(variable, ""):
            environment[variable] = ""

    for variable in ("PATH", "XDG_DATA_DIRS"):
        value = os.environ.get(variable, "")
        cleaned = ":".join(part for part in value.split(":") if "/snap/" not in part)
        if cleaned != value:
            environment[variable] = cleaned

    xdg_data_home = os.environ.get("XDG_DATA_HOME", "")
    if "/snap/" in xdg_data_home:
        environment["XDG_DATA_HOME"] = os.path.expanduser("~/.local/share")

    for variable in os.environ:
        if variable == "SNAP" or variable.startswith("SNAP_"):
            environment[variable] = ""
    return environment


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'num_robots',
            default_value='1',
            description='Number of robots to spawn'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'world_file',
            default_value='empty_world.usd',
            description='World USD file name'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "run_rviz",
            default_value="true",
            description="Launch RViz2 with predefined config"
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Launch without a graphical interface"
        )
    )

    num_robots = LaunchConfiguration('num_robots')
    world_file = LaunchConfiguration('world_file')
    run_rviz = LaunchConfiguration("run_rviz")
    headless = LaunchConfiguration("headless")

    headless_script = PathJoinSubstitution([
        FindPackageShare('isaac_sim'),
        'utils/isaac_sim_launcher_headless.py'
    ])
    
    autorun_script = PathJoinSubstitution([
        FindPackageShare('isaac_sim'),
        'utils/isaac_sim_launcher.py'
    ])

    isaac_sim_root = os.path.abspath(
        os.path.expanduser(os.environ.get("ISAAC_SIM_ROOT", "~/isaacsim"))
    )
    gpu_index = _benchmark_gpu_index()
    isaac_environment = {
        'NUM_ROBOTS': num_robots,
        'WORLD_FILE': world_file,
        # Keep both launch paths on the same physical GPU.  SimulationApp uses
        # multi-GPU by default, whereas a direct Kit launch does not specify it.
        'ISAAC_BENCHMARK_GPU': str(gpu_index),
        # isaac-sim.sh supplied this default before we switched to Kit's base
        # GUI experience.  Keep the same ROS middleware selection.
        'RMW_IMPLEMENTATION': os.environ.get(
            'RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'
        ),
    }


    isaac_sim = ExecuteProcess(
         cmd=[
             # Do not use isaac-sim.sh here: it always selects the full editor
             # experience.  Its isaacsim.app.setup extension holds Kit's
             # app-ready event until a viewport has rendered its first frame.
             # On this machine that delayed our --exec benchmark launcher by
             # about 72 seconds.  The base experience keeps the GUI viewport,
             # RTX renderer, PhysX and sensor extensions, without that
             # editor-startup gate.
             os.path.join(isaac_sim_root, "kit", "kit"),
             os.path.join(isaac_sim_root, "apps", "isaacsim.exp.base.kit"),
             "--enable", "isaacsim.ros2.bridge",
             "--/crashreporter/skipOldDumpUpload=1",
             "--/renderer/multiGpu/enabled=false",
             f"--/renderer/multiGpu/maxGpuCount=1",
             f"--/renderer/activeGpu={gpu_index}",
             f"--/physics/cudaDevice={gpu_index}",
             # Match SimulationApp's explicit real-time path-tracing profile.
             "--/rtx/rendermode=RealTimePathTracing",
             "--/rtx/post/aa/op=3",
             "--/rtx/rtpt/maxBounces=3",
             "--/rtx/rtpt/maxSpecularAndTransmissionBounces=3",
             "--/rtx/rtpt/maxVolumeBounces=15",
             "--exec", autorun_script
         ],
         output="screen",
         additional_env=isaac_environment,
         on_exit=Shutdown(reason="Isaac Sim GUI process exited"),
         condition = UnlessCondition(headless)
    )

    isaac_sim_headless = ExecuteProcess(
        cmd=[
            os.path.join(isaac_sim_root, "python.sh"),
            headless_script
        ],
        output="screen",
        additional_env={
            'NUM_ROBOTS': num_robots,
            'WORLD_FILE': world_file,
            'ISAAC_BENCHMARK_GPU': str(gpu_index),
            'RMW_IMPLEMENTATION': os.environ.get(
                'RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'
            ),
        },
        on_exit=Shutdown(reason="Isaac Sim headless process exited"),
        condition = IfCondition(headless)
    )


    # run rviz
    rviz_config = PathJoinSubstitution([
        FindPackageShare("isaac_sim"), 
        "config",
        "rviz_config.rviz"
    ])

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        additional_env=_rviz_environment(),
        on_exit=Shutdown(reason="RViz process exited"),
        condition=IfCondition(run_rviz)
    )


    group = GroupAction([
        isaac_sim,
        isaac_sim_headless,
        rviz
    ])

    return LaunchDescription(declared_arguments + [group])
