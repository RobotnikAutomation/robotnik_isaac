"""Regression tests for the benchmark sensor-frame contract."""

import ast
from pathlib import Path


RUNTIME = Path(__file__).parents[1] / "utils" / "isaac_sim_runtime.py"


def test_lidar_frame_is_published_into_the_robot_transform_tree():
    source = RUNTIME.read_text(encoding="utf-8")
    tree = ast.parse(source)
    functions = {
        node.name: node
        for node in tree.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
    }

    assert "_create_static_laser_tf_graph" in functions
    assert '"publish_tf.inputs:topicName", "/tf_static"' in source
    assert '"publish_tf.inputs:staticPublisher", True' in source
    laser_function = ast.get_source_segment(
        source, functions["_create_static_laser_tf_graph"]
    )
    assert '"base_link", "laser_link", translation, rotation' in laser_function
    assert "_create_static_laser_tf_graph(" in ast.get_source_segment(
        source, functions["spawn_and_configure_robots"]
    )


def test_asset_tf_publishers_are_kept_on_the_global_topics():
    source = RUNTIME.read_text(encoding="utf-8")
    tree = ast.parse(source)
    namespace_function = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "_set_robot_namespace"
    )
    function_source = ast.get_source_segment(source, namespace_function)

    assert '"transform_tree" in prim.GetName()' in function_source
    assert 'topic.Set(f"/{topic_name}" if topic_name else "/tf")' in function_source
    assert 'namespace.Set("" if is_transform_publisher else robot_name)' in function_source


def test_world_is_connected_to_the_odometry_root():
    source = RUNTIME.read_text(encoding="utf-8")

    assert '"/World/benchmark_world_tf"' in source
    assert '"world",\n            "odom"' in source


def test_lidar_message_uses_the_published_child_frame():
    source = RUNTIME.read_text(encoding="utf-8")

    assert 'frameId="laser_link"' in source
