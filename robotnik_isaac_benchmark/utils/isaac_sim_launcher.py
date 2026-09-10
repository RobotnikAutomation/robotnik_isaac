import signal
import sys
import time
from pathlib import Path

import omni.kit.app
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
    """Let Kit finish its own shutdown instead of aborting the Python script."""
    global shutdown_requested
    shutdown_requested = True


app = omni.kit.app.get_app()
for handled_signal in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
    signal.signal(handled_signal, request_shutdown)

# The benchmark GUI uses Isaac Sim Base rather than the full editor experience
# so startup does not wait indefinitely for the editor viewport bootstrap.
# Enable the runtime dependencies explicitly; the full editor used to do this
# asynchronously in isaacsim.app.setup.
enable_extension("isaacsim.ros2.bridge")
enable_extension("omni.graph.action")
app.update()
app.update()

num_robots, world_path, model_path = parse_runtime_config(__file__)
stage = open_world(app, world_path)
print(f"Spawning and configuring {num_robots} robot(s)...")
sensor_handles = spawn_and_configure_robots(app, stage, model_path, num_robots)
configure_realtime_pacing()

timeline = omni.timeline.get_timeline_interface()
timeline.play()
print(f"Simulation running with {world_path}. Press Ctrl+C to stop.")

# A signal only requests shutdown. The update loop is then allowed to process
# Kit's uncancellable quit event and unload plugins in the correct order.
try:
    while app.is_running() and not shutdown_requested:
        app.update()
finally:
    timeline.stop()
    sensor_handles.clear()
    if app.is_running():
        app.post_uncancellable_quit(0)
        shutdown_deadline = time.monotonic() + 30.0
        while app.is_running() and time.monotonic() < shutdown_deadline:
            app.update()
