#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Howon Kim

from __future__ import annotations

from dataclasses import dataclass
import threading

import matplotlib
from matplotlib.animation import FuncAnimation
from matplotlib.colors import LinearSegmentedColormap
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from robotis_interfaces.msg import HandPressures

matplotlib.use('TkAgg')
matplotlib.rcParams['toolbar'] = 'None'


@dataclass(frozen=True)
class VizConfig:
    hand_side: str
    topic: str
    sensor_prefix: str
    num_fingers: int
    num_taxels: int
    update_hz: float
    use_best_effort: bool

    # Baseline filter
    baseline_seconds: float
    ema_alpha: float
    deadband: float
    clip_negative: bool

    # Display
    viz_gain: float
    color_max: float


def _make_qos(best_effort: bool) -> QoSProfile:
    if best_effort:
        return QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=DurabilityPolicy.VOLATILE,
    )


class PressureViz(Node):

    def __init__(self) -> None:
        super().__init__('pressure_viz')

        self.cfg = self._declare_and_read_params()

        if self.cfg.num_taxels != 9:
            self.get_logger().warn(
                f'num_taxels={self.cfg.num_taxels}. This visualizer assumes 9 taxels (3x3).'
            )

        self._lock = threading.Lock()

        self._pressure = np.zeros((self.cfg.num_fingers, self.cfg.num_taxels), dtype=float)
        self._ema = np.zeros_like(self._pressure)

        self._baseline = np.zeros_like(self._pressure)
        self._baseline_sum = np.zeros_like(self._pressure)
        self._baseline_count = 0
        self._baseline_ready = False
        self._baseline_frames = max(1, int(self.cfg.update_hz * self.cfg.baseline_seconds))

        qos = _make_qos(self.cfg.use_best_effort)
        self.sub = self.create_subscription(HandPressures, self.cfg.topic, self._cb, qos)

        self.get_logger().info(
            f'Subscribing: {self.cfg.topic} | baseline {self.cfg.baseline_seconds:.2f}s '
            f'(~{self._baseline_frames} frames) | gain={self.cfg.viz_gain:.2f} | '
            f'color_max={self.cfg.color_max:.2f}'
        )

        self._setup_figure()

        interval_ms = int(1000.0 / max(self.cfg.update_hz, 1.0))
        self.ani = FuncAnimation(self.fig, self._update_plot, interval=interval_ms)

    def _declare_and_read_params(self) -> VizConfig:
        self.declare_parameter('hand_side', 'right')
        self.declare_parameter('hand', '')
        hand_side = str(self.get_parameter('hand').value)
        if not hand_side:
            hand_side = str(self.get_parameter('hand_side').value)
        hand_side = 'left' if hand_side in ('left', 'l') else 'right'
        hand_suffix = 'l' if hand_side == 'left' else 'r'

        self.declare_parameter('topic', f'/{hand_side}_hand/finger_pressures')
        self.declare_parameter('sensor_prefix', f'finger_{hand_suffix}_sensor')
        self.declare_parameter('num_fingers', 5)
        self.declare_parameter('num_taxels', 9)
        self.declare_parameter('update_hz', 20.0)
        self.declare_parameter('use_best_effort', True)

        self.declare_parameter('baseline_seconds', 1.0)
        self.declare_parameter('ema_alpha', 0.25)
        self.declare_parameter('deadband', 1.0)
        self.declare_parameter('clip_negative', True)

        self.declare_parameter('viz_gain', 1.5)
        self.declare_parameter('color_max', 125.0)

        return VizConfig(
            hand_side=hand_side,
            topic=str(self.get_parameter('topic').value),
            sensor_prefix=str(self.get_parameter('sensor_prefix').value),
            num_fingers=int(self.get_parameter('num_fingers').value),
            num_taxels=int(self.get_parameter('num_taxels').value),
            update_hz=float(self.get_parameter('update_hz').value),
            use_best_effort=bool(self.get_parameter('use_best_effort').value),
            baseline_seconds=float(self.get_parameter('baseline_seconds').value),
            ema_alpha=float(self.get_parameter('ema_alpha').value),
            deadband=float(self.get_parameter('deadband').value),
            clip_negative=bool(self.get_parameter('clip_negative').value),
            viz_gain=float(self.get_parameter('viz_gain').value),
            color_max=float(self.get_parameter('color_max').value),
        )

    def _finger_index_from_sensor_name(self, sensor_name: str) -> int | None:
        if not sensor_name.startswith(self.cfg.sensor_prefix):
            return None

        suffix = sensor_name[len(self.cfg.sensor_prefix):]
        if not suffix.isdigit():
            return None

        idx = int(suffix) - 1
        return idx if 0 <= idx < self.cfg.num_fingers else None

    def _sensor_to_array(self, sensor_msg) -> np.ndarray:
        values = np.array(sensor_msg.pressure_values, dtype=float)

        if values.size < self.cfg.num_taxels:
            out = np.zeros(self.cfg.num_taxels, dtype=float)
            out[:values.size] = values
            return out

        return values[:self.cfg.num_taxels]

    def _cb(self, msg: HandPressures) -> None:
        with self._lock:
            for sensor in msg.sensors:
                finger_idx = self._finger_index_from_sensor_name(sensor.sensor_name)
                if finger_idx is None:
                    continue

                values = self._sensor_to_array(sensor)

                if not self._baseline_ready:
                    self._accumulate_baseline(finger_idx, values)
                else:
                    self._update_pressure(finger_idx, values)

            if not self._baseline_ready:
                self._baseline_count += 1
                if self._baseline_count >= self._baseline_frames:
                    self._finalize_baseline()

    def _accumulate_baseline(self, finger_idx: int, values: np.ndarray) -> None:
        self._baseline_sum[finger_idx, :] += values

    def _update_pressure(self, finger_idx: int, values: np.ndarray) -> None:
        for t in range(self.cfg.num_taxels):
            val = values[t] - self._baseline[finger_idx, t]

            if self.cfg.clip_negative and val < 0.0:
                val = 0.0
            if val < self.cfg.deadband:
                val = 0.0

            prev = self._ema[finger_idx, t]
            filt = (1.0 - self.cfg.ema_alpha) * prev + self.cfg.ema_alpha * val
            self._ema[finger_idx, t] = filt
            self._pressure[finger_idx, t] = filt

    def _finalize_baseline(self) -> None:
        self._baseline = self._baseline_sum / float(self._baseline_count)
        self._baseline_ready = True
        self._ema.fill(0.0)
        self._pressure.fill(0.0)
        self.get_logger().info('Baseline calibrated. Visualizing baseline-subtracted pressure.')

    def _setup_figure(self) -> None:
        self.fig, self.axes = plt.subplots(self.cfg.num_fingers, 1, figsize=(2, 7.5))
        if self.cfg.num_fingers == 1:
            self.axes = [self.axes]

        self.fig.canvas.manager.set_window_title(f'{self.cfg.hand_side.title()} Finger Pressure')
        self.fig.patch.set_facecolor('#d9d9d9')
        self.fig.suptitle(
            f'{self.cfg.hand_side.upper()} SENSOR',
            x=0.055,
            y=0.99,
            ha='left',
            fontsize=9,
            fontweight='bold')

        base_colors = [
            '#ff3b30',  # thumb:  red
            '#ff9500',  # index:  orange
            '#ffd60a',  # middle: yellow
            '#34c759',  # ring:   green
            '#007aff',  # little: blue
        ]

        finger_labels = ['thumb', 'index', 'middle', 'ring', 'little']

        self.finger_cmaps = []
        for color in base_colors[:self.cfg.num_fingers]:
            cmap = LinearSegmentedColormap.from_list(
                f'finger_cmap_{color}',
                [
                    '#0b0b0b',
                    color,
                ],
            )
            self.finger_cmaps.append(cmap)

        self.images = []

        for i, ax in enumerate(self.axes):
            ax.set_facecolor('white')
            img = ax.imshow(
                np.zeros((3, 3)),
                cmap=self.finger_cmaps[i],
                vmin=0.0,
                vmax=self.cfg.color_max,
                interpolation='nearest',
                origin='upper',
                aspect='auto',
            )
            self.images.append(img)

            ax.text(
                0.5, -0.05, finger_labels[i],
                transform=ax.transAxes,
                ha='center',
                va='top',
                fontsize=12,
            )

            ax.set_xticks([])
            ax.set_yticks([])

            for k in range(4):
                ax.axhline(k - 0.5, color='#1f1f1f', linewidth=1.2)
                ax.axvline(k - 0.5, color='#1f1f1f', linewidth=1.2)

            for spine in ax.spines.values():
                spine.set_edgecolor('#555555')
                spine.set_linewidth(1.0)
        self.fig.subplots_adjust(left=0.18, right=0.82, top=0.96, bottom=0.03, hspace=0.18)

    @staticmethod
    def _taxels_to_3x3(taxels: np.ndarray) -> np.ndarray:
        if taxels.size != 9:
            out = np.zeros((3, 3), dtype=float)
            n = min(taxels.size, 9)
            out.ravel()[:n] = taxels[:n]
            return out
        return taxels.reshape(3, 3)

    def _update_plot(self, _frame) -> None:
        with self._lock:
            data = self._pressure.copy()

        data_viz = data * self.cfg.viz_gain
        data_viz = np.sqrt(np.clip(data_viz, 0.0, None)) * 8.0

        for i, img in enumerate(self.images):
            grid = self._taxels_to_3x3(data_viz[i, :])
            grid = np.clip(grid, 0.0, self.cfg.color_max)
            img.set_data(grid)

        return self.images


def main() -> None:
    rclpy.init()
    node = PressureViz()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
