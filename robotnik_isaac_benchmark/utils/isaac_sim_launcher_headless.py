from isaacsim import SimulationApp
simulation_app = SimulationApp(
    {
        "headless": True,
        "sync_loads": True,
        # Do not replay or upload crash dumps left by an earlier iteration.
        # Crash reporting for a new failure remains enabled.
        "extra_args": ["--/crashreporter/skipOldDumpUpload=1"],
    }
)

import signal
import sys
from pathlib import Path

import omni
import omni.timeline

from isaacsim.core.utils.extensions import enable_extension

sys.path.insert(0, str(Path(__file__).resolve().parent))
from isaac_sim_runtime import (
    configure_realtime_pacing,
    open_world,
    parse_runtime_config,
    spawn_and_configure_robots,
)


shutdown_requested = False


def request_shutdown(_signum, _frame):
    global shutdown_requested
    shutdown_requested = True


for handled_signal in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
    signal.signal(handled_signal, request_shutdown)

# -------------------------------
# EXT: habilitar ROS2 Bridge
# -------------------------------
enable_extension("isaacsim.ros2.bridge")
enable_extension("omni.graph.action")

# Let Kit register ROS writers before opening the stage.
simulation_app.update()
simulation_app.update()

num_robots, world_path, model_path = parse_runtime_config(__file__)
stage = open_world(simulation_app, world_path)
print(f"Spawning and configuring {num_robots} robot(s)...")
sensor_handles = spawn_and_configure_robots(
    simulation_app, stage, model_path, num_robots
)
configure_realtime_pacing()

timeline = omni.timeline.get_timeline_interface()
timeline.play()

print(f"Simulation running with {world_path}. Press Ctrl+C to stop.")

try:
    while simulation_app.is_running() and not shutdown_requested:
        simulation_app.update()
finally:
    timeline.stop()
    sensor_handles.clear()
    simulation_app.close()
