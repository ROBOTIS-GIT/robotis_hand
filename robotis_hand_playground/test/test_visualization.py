"""Exercise actual ROS serialization, visualization callbacks and the tenth channel."""
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import pytest
import rclpy
from robotis_interfaces.msg import HandPressures, TactileSensor
from std_msgs.msg import Bool, Float32
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import Marker, MarkerArray


def pressure_message(count, value=0):
    msg = HandPressures()
    for f in range(1, 6):
        sensor = TactileSensor()
        sensor.sensor_name = f'finger_r_sensor{f}'
        sensor.pressure_names = [f'Present Pressure {i}' for i in range(1, count + 1)]
        sensor.pressure_values = [0] * count
        if f == 1:
            sensor.pressure_values[-1] = value
        msg.sensors.append(sensor)
    return msg


@pytest.mark.parametrize('count,geometry', [(9, False), (10, False), (10, True)])
def test_rviz_last_channel(count, geometry):
    executable = os.environ['TEST_RVIZ_EXECUTABLE']
    args = [executable, '--ros-args', '-p', f'num_taxels:={count}',
            '-p', 'baseline_seconds:=0.05', '-p', 'ema_alpha:=1.0']
    if geometry:
        args += ['-p', f'taxel_x:={[i * 0.001 for i in range(count)]}',
                 '-p', f'taxel_y:={[0.0] * count}']
    rclpy.init()
    node = rclpy.create_node('test_tactile_visualization')
    totals, markers = [], []
    node.create_subscription(Float32, '/tactile_force/hand_total',
                             lambda msg: totals.append(msg.data), 10)
    node.create_subscription(MarkerArray, '/tactile_force_markers',
                             lambda msg: markers.append(msg), 10)
    publisher = node.create_publisher(HandPressures, '/right_hand/finger_pressures', 10)
    process = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    try:
        def pump(msg, condition, timeout=5.0):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                assert process.poll() is None, process.stderr.read()
                publisher.publish(msg)
                rclpy.spin_once(node, timeout_sec=0.05)
                if condition():
                    return
            pytest.fail('Timed out waiting for tactile output')
        pump(pressure_message(count), lambda: publisher.get_subscription_count() > 0)
        # Deliver enough zero frames after discovery to complete calibration.
        for _ in range(15):
            publisher.publish(pressure_message(count))
            rclpy.spin_once(node, timeout_sec=0.05)
        pump(pressure_message(count, 100), lambda: bool(totals) and totals[-1] == 100.0)
        pump(pressure_message(count, 100), lambda: bool(markers))
        if count == 10 and not geometry:
            assert all(m.action == Marker.DELETE for m in markers[-1].markers)
        else:
            assert all(m.action == Marker.ADD for m in markers[-1].markers)
        bad = pressure_message(count, 200)
        bad.sensors[0].pressure_names[-1] = 'wrong channel'
        for _ in range(10):
            publisher.publish(bad)
            rclpy.spin_once(node, timeout_sec=0.05)
        assert totals[-1] == 100.0
    finally:
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        node.destroy_node()
        rclpy.shutdown()


def test_matplotlib_preserves_tenth_channel_and_rejects_mismatch():
    path = Path(__file__).parents[1] / 'scripts/matplot_visualize_tactile.py'
    spec = importlib.util.spec_from_file_location('pressure_viz_test', path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    rclpy.init(args=['--ros-args', '-p', 'num_taxels:=10', '-p', 'ema_alpha:=1.0'])
    node = module.PressureViz()
    try:
        node._baseline_ready = True
        node._cb(pressure_message(10, 100))
        assert node._pressure[0, 9] == 100.0
        assert node._pressure[0, :9].sum() == 0.0
        node._update_plot(0)
        assert node.images[0].get_array().shape == (1, 10)
        assert node.images[0].get_array()[0, 9] > 0
        before = node._pressure.copy()
        node._cb(pressure_message(9, 200))
        np.testing.assert_array_equal(node._pressure, before)
        assert module.PressureViz._taxels_to_grid(np.arange(9)).shape == (3, 3)
        with pytest.raises(ValueError):
            module.PressureViz._taxels_to_grid(np.arange(8))
    finally:
        module.plt.close(node.fig)
        node.destroy_node()
        rclpy.shutdown()


def test_force_controller_pauses_on_missing_invalid_and_stale_tactile():
    executable = os.environ['TEST_FORCE_EXECUTABLE']
    rclpy.init()
    node = rclpy.create_node('test_force_feedback_guard')
    targets = []
    node.create_subscription(
        JointTrajectory,
        '/leader/joint_trajectory_command_broadcaster_right_hand/joint_trajectory',
        lambda msg: targets.append(list(msg.points[0].positions)), 10)
    pressure = node.create_publisher(HandPressures, '/right_hand/finger_pressures', 10)
    grasp = node.create_publisher(Bool, '/grasp_start', 10)
    process = subprocess.Popen(
        [executable, '--ros-args', '-p', 'num_taxels:=10'],
        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    try:
        def pump(seconds, msg=None):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                assert process.poll() is None, process.stderr.read()
                if msg is not None:
                    pressure.publish(msg)
                rclpy.spin_once(node, timeout_sec=0.02)
                time.sleep(0.01)

        deadline = time.monotonic() + 5
        while not targets or grasp.get_subscription_count() == 0:
            assert time.monotonic() < deadline
            pump(0.05)
        grasp.publish(Bool(data=True))
        pump(0.2)
        initial = targets[-1]
        pump(0.2)
        assert targets[-1] == initial  # No calibrated feedback yet.
        pump(1.5, pressure_message(10))
        assert targets[-1] != initial
        invalid = pressure_message(9)
        pump(0.2, invalid)  # Drain any already queued valid frames.
        stopped = targets[-1]
        pump(0.2, invalid)
        assert targets[-1] == stopped
        pump(0.2, pressure_message(10))
        assert targets[-1] != stopped
        pump(0.8)  # The last valid reading is now older than 500 ms.
        stale = targets[-1]
        pump(0.2)
        assert targets[-1] == stale
    finally:
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        node.destroy_node()
        rclpy.shutdown()
