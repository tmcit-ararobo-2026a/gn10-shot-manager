"""3D Trajectory Visualizer, Release Velocity Analyzer & Launcher Parameter Generator
for gn10-shot-manager.

All trajectory analysis and visualization are performed in the robot's `base_link`
coordinate frame (X: forward [m], Y: lateral [m], Z: height [m], L = hypot(X, Y)).
"""

import json
import math
import os
import sys
import warnings
from typing import Dict, List, Optional, Tuple

import matplotlib
matplotlib.use("Qt5Agg")
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
import matplotlib.cm as cm
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import numpy as np

try:
    from scipy.optimize import least_squares
except ImportError:
    least_squares = None

from PyQt5 import QtCore, QtGui, QtWidgets


GRAVITY = 9.80665


class TrajectoryPoint:
    """Single trajectory observation in base_link frame."""

    def __init__(
        self,
        t: float,
        x: float,
        y: float,
        z: float,
        v_act: float,
        v_tgt: float,
        volt: float,
        curr: float,
    ):
        self.t = t
        self.x = x
        self.y = y
        self.z = z
        self.l = math.hypot(x, y)
        self.v_act = v_act
        self.v_tgt = v_tgt
        self.volt = volt
        self.curr = curr


class ShotRecord:
    """Data representation of a single shot trial in base_link coordinates."""

    LABEL_UNCLASSIFIED = "Unclassified"
    LABEL_ROLLED = "Rolled (Closed)"
    LABEL_OPENED = "Opened"
    LABEL_EXCLUDED = "Excluded (Noise/Invalid)"

    def __init__(self, raw_data: dict, index: int):
        self.index = index
        self.raw_data = raw_data
        self.shot_id = raw_data.get("shot_id", index)
        self.timestamp = raw_data.get("timestamp", "")
        self.target = raw_data.get("target", {})
        self.estimated_target = raw_data.get("estimated_target", {})
        self.robot_pose = raw_data.get("robot_pose", {})
        self.summary = raw_data.get("summary", {})
        self.landing = raw_data.get("landing_position", {})
        self.cloth_size = str(raw_data.get("cloth_size", "unknown")).lower()
        self.result = str(raw_data.get("result", "unconfirmed")).lower()
        self.success = raw_data.get("success", None)

        self.v_max = float(self.summary.get("max_velocity", 0.0))
        self.avg_voltage = float(self.summary.get("avg_voltage", 0.0))
        self.max_current = float(self.summary.get("max_current", 0.0))

        samples = raw_data.get("samples", [])

        # Extract true release target velocity from post-trigger samples (t >= 0)
        post_v_tgts = [
            float(s.get("v_tgt", 0.0))
            for s in samples
            if float(s.get("t", 0.0)) >= 0.0 and float(s.get("v_tgt", 0.0)) > 0.1
        ]
        all_v_tgts = [float(s.get("v_tgt", 0.0)) for s in samples if float(s.get("v_tgt", 0.0)) > 0.1]

        if "release_target_velocity" in self.summary and float(self.summary["release_target_velocity"]) > 0.1:
            self.v_tgt = float(self.summary["release_target_velocity"])
        elif post_v_tgts:
            self.v_tgt = float(max(post_v_tgts))
        elif all_v_tgts:
            self.v_tgt = float(max(all_v_tgts))
        else:
            self.v_tgt = float(self.target.get("target_velocity", 0.0))

        # Effective release velocity (prefer target command if set, else measured max velocity)
        self.v_release = self.v_tgt if self.v_tgt > 0.1 else self.v_max
        self.v_bin = round(self.v_release * 4.0) / 4.0  # 0.25 m/s grouping

        # Detect pre-trigger stale cloth coordinate (at t <= 0.02) so we can strip it
        stale_coords = set()
        for s in samples:
            if float(s.get("t", 0.0)) <= 0.02:
                cx, cy, cz = s.get("cx"), s.get("cy"), s.get("cz")
                if cx is not None and cy is not None and cz is not None:
                    stale_coords.add((round(float(cx), 4), round(float(cy), 4), round(float(cz), 4)))

        self.raw_points: List[TrajectoryPoint] = []
        for s in samples:
            t_val = float(s.get("t", 0.0))
            if t_val <= 0.02:
                continue
            cx, cy, cz = s.get("cx"), s.get("cy"), s.get("cz")
            if cx is not None and cy is not None and cz is not None:
                key = (round(float(cx), 4), round(float(cy), 4), round(float(cz), 4))
                if key in stale_coords:
                    continue
                self.raw_points.append(
                    TrajectoryPoint(
                        t=t_val,
                        x=float(cx),
                        y=float(cy),
                        z=float(cz),
                        v_act=float(s.get("v_act", 0.0)),
                        v_tgt=float(s.get("v_tgt", self.v_tgt)),
                        volt=float(s.get("volt", 0.0)),
                        curr=float(s.get("curr", 0.0)),
                    )
                )

        # Deduplicate consecutive identical points (camera hold frames) while keeping flight path
        self.filtered_points: List[TrajectoryPoint] = []
        self.filter_points()

        # Voltage is reference only; enable any shot that has enough trajectory points
        self.label = self.LABEL_UNCLASSIFIED
        self.enabled = len(self.filtered_points) >= 3 and self.v_release > 0.5
        if not self.enabled:
            self.label = self.LABEL_EXCLUDED

    def filter_points(self, min_z: float = 0.12, max_jump: float = 1.2):
        """Filter ground reflections, duplicate held frames, and jump outliers in base_link frame."""
        if not self.raw_points:
            self.filtered_points = []
            return

        cleaned: List[TrajectoryPoint] = []
        prev: Optional[TrajectoryPoint] = None

        for pt in self.raw_points:
            if pt.z < min_z or pt.x < 0.1:
                continue
            if prev is not None:
                dist = math.sqrt((pt.x - prev.x) ** 2 + (pt.y - prev.y) ** 2 + (pt.z - prev.z) ** 2)
                if dist < 1e-4:
                    # Identical held frame from camera
                    continue
                if dist > max_jump:
                    continue
            cleaned.append(pt)
            prev = pt

        self.filtered_points = cleaned

    @property
    def max_x(self) -> float:
        if not self.filtered_points:
            return 0.0
        return max(p.x for p in self.filtered_points)

    @property
    def max_l(self) -> float:
        if not self.filtered_points:
            return 0.0
        return max(p.l for p in self.filtered_points)

    @property
    def peak_z(self) -> float:
        if not self.filtered_points:
            return 0.0
        return max(p.z for p in self.filtered_points)

    @property
    def target_display_name(self) -> str:
        if self.estimated_target and self.estimated_target.get("name"):
            return str(self.estimated_target["name"])
        return str(self.target.get("name", f"Target_{self.target.get('target_id', 0)}"))


class BallisticModel:
    """Ballistic model in base_link coordinates matching moving_bucket_launcher_node."""

    def __init__(
        self,
        bucket_height: float = 0.255,
        aim_z_offset: float = 0.0,
        launcher_offset_x: float = 0.348,
        launcher_offset_z: float = 1.034,
        launch_angle_deg: float = 45.88,
        velocity_quad: float = 0.0,
        velocity_scale: float = 0.806,
        velocity_offset: float = -0.243,
        min_velocity: float = 2.5,
        max_velocity: float = 7.75,
    ):
        self.bucket_height = bucket_height
        self.aim_z_offset = aim_z_offset
        self.launcher_offset_x = launcher_offset_x
        self.launcher_offset_z = launcher_offset_z
        self.launch_angle_deg = launch_angle_deg
        self.velocity_quad = velocity_quad
        self.velocity_scale = velocity_scale
        self.velocity_offset = velocity_offset
        self.min_velocity = min_velocity
        self.max_velocity = max_velocity

    def belt_speed_to_ballistic_v0(self, v_belt: float) -> float:
        """Invert v_belt = quad * v0^2 + scale * v0 + offset for v0 > 0."""
        if abs(self.velocity_quad) < 1e-6:
            if abs(self.velocity_scale) < 1e-6:
                return max(v_belt, 0.1)
            return max((v_belt - self.velocity_offset) / self.velocity_scale, 0.1)
        # Solve quad * v0^2 + scale * v0 + (offset - v_belt) = 0
        a = self.velocity_quad
        b = self.velocity_scale
        c = self.velocity_offset - v_belt
        disc = b * b - 4.0 * a * c
        if disc < 0.0:
            return max((v_belt - self.velocity_offset) / max(self.velocity_scale, 1e-3), 0.1)
        v0 = (-b + math.sqrt(disc)) / (2.0 * a)
        return max(v0, 0.1)

    def predict_z(self, l_arr: np.ndarray, v_belt: float) -> np.ndarray:
        """Predict height z(L) in base_link frame for horizontal distance L and release speed v_belt."""
        v0 = self.belt_speed_to_ballistic_v0(v_belt)
        theta = math.radians(self.launch_angle_deg)
        cos_t = math.cos(theta)
        tan_t = math.tan(theta)
        dx = l_arr - self.launcher_offset_x
        z = (
            self.launcher_offset_z
            + dx * tan_t
            - (GRAVITY * dx * dx) / (2.0 * v0 * v0 * cos_t * cos_t)
        )
        return z

    def solve_belt_speed(self, l_dist: float, opening_z: float) -> Optional[float]:
        """Compute required belt release speed to pass through upper opening (l_dist, opening_z) in base_link."""
        dx = l_dist - self.launcher_offset_x
        if dx <= 0.05:
            return None
        dz = (opening_z + self.aim_z_offset) - self.launcher_offset_z
        theta = math.radians(self.launch_angle_deg)
        cos_t = math.cos(theta)
        tan_t = math.tan(theta)
        denom = 2.0 * cos_t * cos_t * (dx * tan_t - dz)
        if denom <= 0.0:
            return None
        v0 = math.sqrt((GRAVITY * dx * dx) / denom)
        v_belt = self.velocity_quad * v0 * v0 + self.velocity_scale * v0 + self.velocity_offset
        return v_belt

    def to_yaml_string(self) -> str:
        """Format parameters as config/moving_bucket_launcher_params.yaml."""
        return (
            "moving_bucket_launcher_node:\n"
            "  ros__parameters:\n"
            "    base_frame: base_link\n"
            "    opening_topic: /bucket_opening_tracker/target\n"
            "    bottom_topic: /bucket_opening_tracker/bottom\n"
            "    use_bottom_topic: false\n"
            f"    bucket_height: {self.bucket_height:.3f}\n"
            f"    aim_z_offset: {self.aim_z_offset:.3f}\n"
            f"    launcher_offset_x: {self.launcher_offset_x:.3f}\n"
            f"    launcher_offset_z: {self.launcher_offset_z:.3f}\n"
            f"    launch_angle_deg: {self.launch_angle_deg:.2f}\n"
            f"    velocity_quad: {self.velocity_quad:.4f}\n"
            f"    velocity_scale: {self.velocity_scale:.4f}\n"
            f"    velocity_offset: {self.velocity_offset:.4f}\n"
            f"    min_velocity: {self.min_velocity:.2f}\n"
            f"    max_velocity: {self.max_velocity:.2f}\n"
            "    min_horizontal_dist: 0.6\n"
            "    max_horizontal_dist: 5.5\n"
            "    max_tracking_age_sec: 0.5\n"
            "    use_sim_time: false\n"
        )


class DispersionAnalyzer:
    """Analyzes trajectory dispersion in base_link frame."""

    @staticmethod
    def interpolate_trajectories(
        records: List[ShotRecord], num_bins: int = 80
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Interpolate records on common horizontal distance grid L in base_link frame."""
        valid_records = [r for r in records if r.enabled and len(r.filtered_points) >= 4]
        if len(valid_records) < 1:
            return np.array([]), np.array([]), np.array([])

        l_min = max(min(r.filtered_points[0].l for r in valid_records), 0.4)
        l_max = min(max(r.max_l for r in valid_records), 6.0)
        if l_min >= l_max:
            return np.array([]), np.array([]), np.array([])

        grid_l = np.linspace(l_min, l_max, num_bins)
        interpolated_z = []

        for r in valid_records:
            ls = [p.l for p in r.filtered_points]
            zs = [p.z for p in r.filtered_points]
            unique_ls, unique_indices = np.unique(ls, return_index=True)
            unique_zs = np.array(zs)[unique_indices]
            if len(unique_ls) < 2:
                continue
            z_interp = np.interp(grid_l, unique_ls, unique_zs, left=np.nan, right=np.nan)
            interpolated_z.append(z_interp)

        if not interpolated_z:
            return np.array([]), np.array([]), np.array([])

        z_matrix = np.array(interpolated_z)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", category=RuntimeWarning)
            mean_z = np.nanmean(z_matrix, axis=0)
            std_z = np.nanstd(z_matrix, axis=0)

        return grid_l, mean_z, std_z

    @staticmethod
    def detect_branch_point(
        grid_l: np.ndarray, std_z: np.ndarray, std_thresh_ratio: float = 2.5
    ) -> Optional[float]:
        """Detect horizontal distance L where vertical dispersion abruptly increases."""
        valid_mask = np.isfinite(std_z) & (std_z > 0.001)
        if np.sum(valid_mask) < 8:
            return None

        gl = grid_l[valid_mask]
        sz = std_z[valid_mask]
        init_len = max(3, len(sz) // 5)
        baseline_std = max(float(np.median(sz[:init_len])), 0.02)
        threshold = baseline_std * std_thresh_ratio

        for l_val, s_val in zip(gl[init_len:], sz[init_len:]):
            if s_val >= threshold:
                return float(l_val)
        return None


class TrajectoryVisualizerApp(QtWidgets.QMainWindow):
    """Main window for base_link trajectory analysis, velocity comparison, and parameter export."""

    def __init__(self, log_path: str):
        super().__init__()
        self.setWindowTitle(
            "gn10 Shot Trajectory Analyzer (base_link Frame / Release Velocity / Launcher Parameter Fitter)"
        )
        self.resize(1420, 900)

        self.log_path = log_path
        self.records: List[ShotRecord] = []
        self.model = BallisticModel()

        self.init_ui()
        self.load_log_file(log_path)

    def init_ui(self):
        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)
        layout = QtWidgets.QHBoxLayout(central)

        # Left panel: Data source, Filters, Curation Table
        left_panel = QtWidgets.QWidget()
        left_panel.setFixedWidth(460)
        left_layout = QtWidgets.QVBoxLayout(left_panel)
        layout.addWidget(left_panel)

        # 1. File header
        file_box = QtWidgets.QGroupBox("Data Source (base_link Frame)")
        file_layout = QtWidgets.QHBoxLayout(file_box)
        self.lbl_path = QtWidgets.QLabel(f"{os.path.basename(self.log_path)}")
        self.lbl_path.setToolTip(self.log_path)
        file_layout.addWidget(self.lbl_path, stretch=1)

        btn_open = QtWidgets.QPushButton("Open...")
        btn_open.clicked.connect(self.browse_log_file)
        file_layout.addWidget(btn_open)

        btn_reload = QtWidgets.QPushButton("Reload")
        btn_reload.clicked.connect(lambda: self.load_log_file(self.log_path))
        file_layout.addWidget(btn_reload)
        left_layout.addWidget(file_box)

        # 2. Filters (Velocity, Cloth Size, Result, Point Filter)
        filter_box = QtWidgets.QGroupBox("Filtering & Grouping Controls")
        filter_layout = QtWidgets.QFormLayout(filter_box)

        self.cmb_vel_filter = QtWidgets.QComboBox()
        self.cmb_vel_filter.addItem("All Release Velocities", None)
        self.cmb_vel_filter.currentIndexChanged.connect(self.apply_attribute_filters)
        filter_layout.addRow("Release Velocity:", self.cmb_vel_filter)

        self.cmb_size_filter = QtWidgets.QComboBox()
        self.cmb_size_filter.addItems(["All Sizes", "large", "small", "unknown"])
        self.cmb_size_filter.currentIndexChanged.connect(self.apply_attribute_filters)
        filter_layout.addRow("Cloth Size:", self.cmb_size_filter)

        self.cmb_result_filter = QtWidgets.QComboBox()
        self.cmb_result_filter.addItems(["All Results", "success", "failure", "unconfirmed"])
        self.cmb_result_filter.currentIndexChanged.connect(self.apply_attribute_filters)
        filter_layout.addRow("Shot Result:", self.cmb_result_filter)

        noise_row = QtWidgets.QHBoxLayout()
        self.spin_min_z = QtWidgets.QDoubleSpinBox()
        self.spin_min_z.setRange(0.0, 1.5)
        self.spin_min_z.setSingleStep(0.05)
        self.spin_min_z.setValue(0.12)
        self.spin_min_z.setPrefix("MinZ: ")
        self.spin_min_z.setSuffix("m")
        self.spin_min_z.valueChanged.connect(self.on_filter_changed)

        self.spin_max_jump = QtWidgets.QDoubleSpinBox()
        self.spin_max_jump.setRange(0.2, 5.0)
        self.spin_max_jump.setSingleStep(0.1)
        self.spin_max_jump.setValue(1.2)
        self.spin_max_jump.setPrefix("MaxJump: ")
        self.spin_max_jump.setSuffix("m")
        self.spin_max_jump.valueChanged.connect(self.on_filter_changed)

        noise_row.addWidget(self.spin_min_z)
        noise_row.addWidget(self.spin_max_jump)
        filter_layout.addRow("Outlier Filter:", noise_row)

        left_layout.addWidget(filter_box)

        # 3. Curation Table
        table_box = QtWidgets.QGroupBox("Shot Curation (Human Selection for Fitting)")
        table_layout = QtWidgets.QVBoxLayout(table_box)

        btn_layout = QtWidgets.QHBoxLayout()
        btn_select_all = QtWidgets.QPushButton("Select All Valid")
        btn_select_all.clicked.connect(self.select_all_trials)
        btn_deselect_all = QtWidgets.QPushButton("Deselect All")
        btn_deselect_all.clicked.connect(self.deselect_all_trials)
        btn_auto_label = QtWidgets.QPushButton("Auto Classify Rolled/Opened")
        btn_auto_label.clicked.connect(self.auto_label_trials)
        btn_layout.addWidget(btn_select_all)
        btn_layout.addWidget(btn_deselect_all)
        btn_layout.addWidget(btn_auto_label)
        table_layout.addLayout(btn_layout)

        self.table = QtWidgets.QTableWidget()
        self.table.setColumnCount(8)
        self.table.setHorizontalHeaderLabels(
            ["Use", "Shot", "V_rel", "Target", "Size", "Result", "Pts", "Cloth State"]
        )
        for col in range(7):
            self.table.horizontalHeader().setSectionResizeMode(col, QtWidgets.QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(7, QtWidgets.QHeaderView.Stretch)
        self.table.itemSelectionChanged.connect(self.on_table_selection_changed)
        self.table.cellChanged.connect(self.on_table_cell_changed)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        table_layout.addWidget(self.table)

        label_btn_layout = QtWidgets.QHBoxLayout()
        btn_set_rolled = QtWidgets.QPushButton("Set Rolled (Closed)")
        btn_set_rolled.setStyleSheet("background-color: #1e88e5; color: white;")
        btn_set_rolled.clicked.connect(lambda: self.set_selected_label(ShotRecord.LABEL_ROLLED))

        btn_set_opened = QtWidgets.QPushButton("Set Opened")
        btn_set_opened.setStyleSheet("background-color: #e53935; color: white;")
        btn_set_opened.clicked.connect(lambda: self.set_selected_label(ShotRecord.LABEL_OPENED))

        btn_set_exclude = QtWidgets.QPushButton("Exclude")
        btn_set_exclude.setStyleSheet("background-color: #546e7a; color: white;")
        btn_set_exclude.clicked.connect(lambda: self.set_selected_label(ShotRecord.LABEL_EXCLUDED))

        label_btn_layout.addWidget(btn_set_rolled)
        label_btn_layout.addWidget(btn_set_opened)
        label_btn_layout.addWidget(btn_set_exclude)
        table_layout.addLayout(label_btn_layout)

        left_layout.addWidget(table_box, stretch=1)

        # 4. Summary Box
        self.stat_box = QtWidgets.QGroupBox("Summary (base_link Frame)")
        stat_layout = QtWidgets.QVBoxLayout(self.stat_box)
        self.lbl_stats = QtWidgets.QLabel("Loading...")
        self.lbl_stats.setStyleSheet("font-size: 11px;")
        stat_layout.addWidget(self.lbl_stats)
        left_layout.addWidget(self.stat_box)

        # Right panel: Visualization & Parameter Generation Tabs
        self.tabs = QtWidgets.QTabWidget()
        layout.addWidget(self.tabs, stretch=1)

        # Tab 1: Release Velocity Comparison in base_link Frame (2D L-Z & X-Y + Fitted Curves)
        self.tab_vel = QtWidgets.QWidget()
        layout_vel = QtWidgets.QVBoxLayout(self.tab_vel)
        self.fig_vel = Figure(figsize=(8, 6))
        self.canvas_vel = FigureCanvas(self.fig_vel)
        self.toolbar_vel = NavigationToolbar(self.canvas_vel, self.tab_vel)
        layout_vel.addWidget(self.toolbar_vel)
        layout_vel.addWidget(self.canvas_vel)
        self.tabs.addTab(self.tab_vel, "1. Release Velocity Trajectories (base_link)")

        # Tab 2: 3D Trajectory View (base_link Frame)
        self.tab_3d = QtWidgets.QWidget()
        layout_3d = QtWidgets.QVBoxLayout(self.tab_3d)
        self.fig_3d = Figure(figsize=(8, 6))
        self.canvas_3d = FigureCanvas(self.fig_3d)
        self.toolbar_3d = NavigationToolbar(self.canvas_3d, self.tab_3d)
        layout_3d.addWidget(self.toolbar_3d)
        layout_3d.addWidget(self.canvas_3d)
        self.tabs.addTab(self.tab_3d, "2. 3D View (base_link)")

        # Tab 3: Dispersion & Branching Analysis
        self.tab_disp = QtWidgets.QWidget()
        layout_disp = QtWidgets.QVBoxLayout(self.tab_disp)
        self.fig_disp = Figure(figsize=(8, 6))
        self.canvas_disp = FigureCanvas(self.fig_disp)
        self.toolbar_disp = NavigationToolbar(self.canvas_disp, self.tab_disp)
        layout_disp.addWidget(self.toolbar_disp)
        layout_disp.addWidget(self.canvas_disp)
        self.tabs.addTab(self.tab_disp, "3. Dispersion & Opening Branch")

        # Tab 4: Moving Bucket Launcher Parameter Fitting & YAML Export
        self.tab_param = QtWidgets.QWidget()
        self.init_param_tab(self.tab_param)
        self.tabs.addTab(self.tab_param, "4. Launcher Parameter Generator (YAML)")

    def init_param_tab(self, parent: QtWidgets.QWidget):
        """Build interactive ballistic parameter fitter & YAML generator tab."""
        layout = QtWidgets.QHBoxLayout(parent)

        # Left side of tab: Parameter controls & YAML preview
        ctrl_panel = QtWidgets.QWidget()
        ctrl_panel.setFixedWidth(390)
        ctrl_layout = QtWidgets.QVBoxLayout(ctrl_panel)

        fit_box = QtWidgets.QGroupBox("Ballistic Model Parameters (base_link)")
        form = QtWidgets.QFormLayout(fit_box)

        self.spin_bucket_h = self._make_spin(0.0, 1.5, 0.005, self.model.bucket_height, 3, " m")
        self.spin_aim_z = self._make_spin(-0.5, 0.5, 0.01, self.model.aim_z_offset, 3, " m")
        self.spin_off_x = self._make_spin(-0.5, 1.5, 0.01, self.model.launcher_offset_x, 3, " m")
        self.spin_off_z = self._make_spin(0.0, 2.0, 0.01, self.model.launcher_offset_z, 3, " m")
        self.spin_angle = self._make_spin(15.0, 75.0, 0.2, self.model.launch_angle_deg, 2, " deg")
        self.spin_v_quad = self._make_spin(-0.5, 0.5, 0.005, self.model.velocity_quad, 4, "")
        self.spin_v_scale = self._make_spin(0.1, 3.0, 0.01, self.model.velocity_scale, 4, "")
        self.spin_v_offset = self._make_spin(-3.0, 3.0, 0.02, self.model.velocity_offset, 4, " m/s")

        form.addRow("bucket_height (開口部高):", self.spin_bucket_h)
        form.addRow("aim_z_offset (狙いZ補正):", self.spin_aim_z)
        form.addRow("launcher_offset_x (x0):", self.spin_off_x)
        form.addRow("launcher_offset_z (z0):", self.spin_off_z)
        form.addRow("launch_angle_deg (θ):", self.spin_angle)
        form.addRow("velocity_quad (2次係数):", self.spin_v_quad)
        form.addRow("velocity_scale (1次係数):", self.spin_v_scale)
        form.addRow("velocity_offset (切片):", self.spin_v_offset)

        btn_fit_rolled = QtWidgets.QPushButton("Fit Parameters from Active 'Rolled' Shots")
        btn_fit_rolled.setStyleSheet("background-color: #1565c0; color: white; font-weight: bold; padding: 6px;")
        btn_fit_rolled.clicked.connect(lambda: self.fit_ballistic_parameters(only_rolled=True))

        btn_fit_all = QtWidgets.QPushButton("Fit Parameters from All Checked ('Use') Shots")
        btn_fit_all.clicked.connect(lambda: self.fit_ballistic_parameters(only_rolled=False))

        self.lbl_fit_status = QtWidgets.QLabel("Ready. Select clean shots and click Fit.")
        self.lbl_fit_status.setWordWrap(True)
        self.lbl_fit_status.setStyleSheet("font-size: 11px; color: #80cbc4;")

        ctrl_layout.addWidget(fit_box)
        ctrl_layout.addWidget(btn_fit_rolled)
        ctrl_layout.addWidget(btn_fit_all)
        ctrl_layout.addWidget(self.lbl_fit_status)

        yaml_box = QtWidgets.QGroupBox("Generated YAML (config/moving_bucket_launcher_params.yaml)")
        yaml_layout = QtWidgets.QVBoxLayout(yaml_box)
        self.txt_yaml = QtWidgets.QPlainTextEdit()
        self.txt_yaml.setReadOnly(True)
        self.txt_yaml.setStyleSheet("font-family: monospace; font-size: 11px;")
        yaml_layout.addWidget(self.txt_yaml)

        export_btns = QtWidgets.QHBoxLayout()
        btn_copy_yaml = QtWidgets.QPushButton("Copy YAML")
        btn_copy_yaml.clicked.connect(self.copy_yaml_to_clipboard)
        btn_save_yaml = QtWidgets.QPushButton("Save YAML to File...")
        btn_save_yaml.clicked.connect(self.save_yaml_to_file)
        export_btns.addWidget(btn_copy_yaml)
        export_btns.addWidget(btn_save_yaml)
        yaml_layout.addLayout(export_btns)

        ctrl_layout.addWidget(yaml_box, stretch=1)
        layout.addWidget(ctrl_panel)

        # Right side of tab: Fitted Curves & Required Velocity Curve
        plot_panel = QtWidgets.QWidget()
        plot_layout = QtWidgets.QVBoxLayout(plot_panel)
        self.fig_param = Figure(figsize=(7, 6))
        self.canvas_param = FigureCanvas(self.fig_param)
        self.toolbar_param = NavigationToolbar(self.canvas_param, plot_panel)
        plot_layout.addWidget(self.toolbar_param)
        plot_layout.addWidget(self.canvas_param)
        layout.addWidget(plot_panel, stretch=1)

        self.sync_model_from_spins()

    def _make_spin(
        self, vmin: float, vmax: float, step: float, val: float, decimals: int, suffix: str
    ) -> QtWidgets.QDoubleSpinBox:
        sp = QtWidgets.QDoubleSpinBox()
        sp.setDecimals(decimals)
        sp.setRange(vmin, vmax)
        sp.setSingleStep(step)
        sp.setValue(val)
        if suffix:
            sp.setSuffix(suffix)
        sp.valueChanged.connect(self.on_model_spin_changed)
        return sp

    def on_model_spin_changed(self):
        self.sync_model_from_spins()
        self.update_plots()

    def sync_model_from_spins(self):
        self.model.bucket_height = self.spin_bucket_h.value()
        self.model.aim_z_offset = self.spin_aim_z.value()
        self.model.launcher_offset_x = self.spin_off_x.value()
        self.model.launcher_offset_z = self.spin_off_z.value()
        self.model.launch_angle_deg = self.spin_angle.value()
        self.model.velocity_quad = self.spin_v_quad.value()
        self.model.velocity_scale = self.spin_v_scale.value()
        self.model.velocity_offset = self.spin_v_offset.value()
        self.txt_yaml.setPlainText(self.model.to_yaml_string())

    def browse_log_file(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Open Shot Trial JSONL File",
            os.path.dirname(self.log_path),
            "JSON Lines (*.jsonl);;All Files (*)",
        )
        if path:
            self.log_path = path
            self.lbl_path.setText(os.path.basename(path))
            self.lbl_path.setToolTip(path)
            self.load_log_file(path)

    def load_log_file(self, path: str):
        if not os.path.exists(path):
            QtWidgets.QMessageBox.warning(self, "File Not Found", f"Cannot find: {path}")
            return

        self.records = []
        with open(path, "r", encoding="utf-8") as f:
            for idx, line in enumerate(f):
                line = line.strip()
                if not line:
                    continue
                try:
                    data = json.loads(line)
                    self.records.append(ShotRecord(data, idx + 1))
                except Exception as e:
                    print(f"Error parsing line {idx + 1}: {e}")

        # Populate velocity filter dropdown
        v_bins = sorted({r.v_bin for r in self.records if r.v_bin > 0.1})
        self.cmb_vel_filter.blockSignals(True)
        self.cmb_vel_filter.clear()
        self.cmb_vel_filter.addItem("All Release Velocities", None)
        for vb in v_bins:
            count = sum(1 for r in self.records if abs(r.v_bin - vb) < 1e-3)
            self.cmb_vel_filter.addItem(f"{vb:.2f} m/s ({count} shots)", vb)
        self.cmb_vel_filter.blockSignals(False)

        self.auto_label_trials()

    def refresh_table(self):
        self.table.blockSignals(True)
        self.table.setRowCount(len(self.records))

        for row, r in enumerate(self.records):
            chk = QtWidgets.QTableWidgetItem()
            chk.setFlags(QtCore.Qt.ItemIsUserCheckable | QtCore.Qt.ItemIsEnabled)
            chk.setCheckState(QtCore.Qt.Checked if r.enabled else QtCore.Qt.Unchecked)
            self.table.setItem(row, 0, chk)

            self.table.setItem(row, 1, QtWidgets.QTableWidgetItem(f"#{r.shot_id}"))

            item_v = QtWidgets.QTableWidgetItem(f"{r.v_release:.2f}")
            item_v.setToolTip(
                f"v_tgt={r.v_tgt:.2f} m/s, v_max={r.v_max:.2f} m/s\n"
                f"Reference only: avg_volt={r.avg_voltage:.1f}V, max_curr={r.max_current:.1f}A"
            )
            self.table.setItem(row, 2, item_v)

            item_tgt = QtWidgets.QTableWidgetItem(r.target_display_name)
            if r.robot_pose.get("valid"):
                item_tgt.setToolTip(
                    f"map base_link: x={r.robot_pose.get('x', 0):.2f}, "
                    f"y={r.robot_pose.get('y', 0):.2f}, yaw={r.robot_pose.get('yaw', 0):.2f}"
                )
            self.table.setItem(row, 3, item_tgt)

            self.table.setItem(row, 4, QtWidgets.QTableWidgetItem(r.cloth_size))
            self.table.setItem(row, 5, QtWidgets.QTableWidgetItem(r.result))
            self.table.setItem(row, 6, QtWidgets.QTableWidgetItem(str(len(r.filtered_points))))

            item_state = QtWidgets.QTableWidgetItem(r.label)
            if r.label == ShotRecord.LABEL_ROLLED:
                item_state.setForeground(QtGui.QBrush(QtGui.QColor("#29b6f6")))
            elif r.label == ShotRecord.LABEL_OPENED:
                item_state.setForeground(QtGui.QBrush(QtGui.QColor("#ef5350")))
            elif r.label == ShotRecord.LABEL_EXCLUDED:
                item_state.setForeground(QtGui.QBrush(QtGui.QColor("#78909c")))
            self.table.setItem(row, 7, item_state)

        self.table.blockSignals(False)
        self.update_summary_stats()

    def on_table_cell_changed(self, row: int, col: int):
        if col == 0 and 0 <= row < len(self.records):
            chk = self.table.item(row, 0)
            if chk:
                self.records[row].enabled = chk.checkState() == QtCore.Qt.Checked
                self.update_summary_stats()
                self.update_plots()

    def on_table_selection_changed(self):
        self.update_plots()

    def on_filter_changed(self):
        min_z = self.spin_min_z.value()
        max_jump = self.spin_max_jump.value()
        for r in self.records:
            r.filter_points(min_z=min_z, max_jump=max_jump)
        self.refresh_table()
        self.update_plots()

    def apply_attribute_filters(self):
        target_v = self.cmb_vel_filter.currentData()
        size_filter = self.cmb_size_filter.currentText()
        res_filter = self.cmb_result_filter.currentText()

        for r in self.records:
            if len(r.filtered_points) < 3 or r.label == ShotRecord.LABEL_EXCLUDED:
                r.enabled = False
                continue
            v_ok = (target_v is None) or (abs(r.v_bin - float(target_v)) < 1e-3)
            s_ok = (size_filter == "All Sizes") or (r.cloth_size == size_filter)
            r_ok = (res_filter == "All Results") or (r.result == res_filter)
            r.enabled = v_ok and s_ok and r_ok

        self.refresh_table()
        self.update_plots()

    def select_all_trials(self):
        for r in self.records:
            if len(r.filtered_points) >= 3 and r.v_release > 0.5:
                r.enabled = True
        self.refresh_table()
        self.update_plots()

    def deselect_all_trials(self):
        for r in self.records:
            r.enabled = False
        self.refresh_table()
        self.update_plots()

    def set_selected_label(self, label: str):
        rows = set(item.row() for item in self.table.selectedItems())
        for row in rows:
            self.records[row].label = label
            self.records[row].enabled = label != ShotRecord.LABEL_EXCLUDED
        self.refresh_table()
        self.update_plots()

    def auto_label_trials(self):
        """Auto classify rolled vs opened trajectories in base_link frame based on velocity and stall behavior."""
        for r in self.records:
            if len(r.filtered_points) < 3 or r.v_release <= 0.5:
                r.label = ShotRecord.LABEL_EXCLUDED
                r.enabled = False
                continue

            # Detect steep terminal drop / stall characteristic of cloth opening in mid-air
            pts = r.filtered_points
            if r.v_release >= 5.8 and r.max_l < 3.4:
                r.label = ShotRecord.LABEL_OPENED
            elif len(pts) >= 5:
                # Check if horizontal advance stalls while dropping rapidly
                dl_tail = pts[-1].l - pts[max(0, len(pts) - 3)].l
                dz_tail = pts[-1].z - pts[max(0, len(pts) - 3)].z
                if r.v_release >= 5.5 and dl_tail < 0.08 and dz_tail < -0.35:
                    r.label = ShotRecord.LABEL_OPENED
                else:
                    r.label = ShotRecord.LABEL_ROLLED
            else:
                r.label = ShotRecord.LABEL_ROLLED
            r.enabled = True

        self.refresh_table()
        self.update_plots()

    def update_summary_stats(self):
        enabled = [r for r in self.records if r.enabled]
        rolled = [r for r in enabled if r.label == ShotRecord.LABEL_ROLLED]
        opened = [r for r in enabled if r.label == ShotRecord.LABEL_OPENED]

        total_en = len(enabled)
        roll_pct = (len(rolled) / total_en * 100.0) if total_en > 0 else 0.0
        open_pct = (len(opened) / total_en * 100.0) if total_en > 0 else 0.0
        v_groups = sorted({r.v_bin for r in enabled})
        v_str = ", ".join(f"{v:.2f}" for v in v_groups[:6])

        txt = (
            f"<b>Coordinate Frame:</b> <code>base_link</code> (L = hypot(x, y))<br>"
            f"<b>Active Trials:</b> {total_en} / {len(self.records)}<br>"
            f"<b>Rolled (Closed):</b> {len(rolled)} ({roll_pct:.1f}%) | "
            f"<b>Opened:</b> {len(opened)} ({open_pct:.1f}%)<br>"
            f"<b>Active Velocity Bins [m/s]:</b> {v_str if v_str else 'None'}"
        )
        self.lbl_stats.setText(txt)

    def fit_ballistic_parameters(self, only_rolled: bool = True):
        """Fit ballistic parameters in base_link frame from human-selected trajectories."""
        candidates = [
            r
            for r in self.records
            if r.enabled
            and len(r.filtered_points) >= 3
            and r.v_release >= 2.0
            and (not only_rolled or r.label == ShotRecord.LABEL_ROLLED)
        ]

        pts_l = []
        pts_z = []
        pts_v = []
        for r in candidates:
            for p in r.filtered_points:
                if 0.5 <= p.l <= 5.2 and p.z >= 0.15:
                    pts_l.append(p.l)
                    pts_z.append(p.z)
                    pts_v.append(r.v_release)

        if len(pts_l) < 6 or least_squares is None:
            self.lbl_fit_status.setText(
                f"Not enough points to fit ({len(pts_l)} pts across {len(candidates)} shots). "
                "Keep default/manual parameters or select more shots."
            )
            return

        l_arr = np.array(pts_l, dtype=float)
        z_arr = np.array(pts_z, dtype=float)
        v_arr = np.array(pts_v, dtype=float)

        distinct_v = len(set(np.round(v_arr, 1)))

        if distinct_v >= 2:
            # Fit full parameter set: [x0, z0, angle_deg, v_scale, v_offset]
            p0 = np.array([
                self.model.launcher_offset_x,
                self.model.launcher_offset_z,
                self.model.launch_angle_deg,
                self.model.velocity_scale,
                self.model.velocity_offset,
            ])
            bounds = (
                [0.05, 0.20, 25.0, 0.40, -1.50],
                [0.65, 1.35, 65.0, 1.60, 1.50],
            )

            def residuals(params):
                x0, z0, ang_deg, v_scale, v_off = params
                theta = np.radians(ang_deg)
                cos_t = np.cos(theta)
                tan_t = np.tan(theta)
                v0 = np.maximum((v_arr - v_off) / np.maximum(v_scale, 0.1), 1.0)
                dx = l_arr - x0
                z_pred = z0 + dx * tan_t - (GRAVITY * dx * dx) / (2.0 * v0 * v0 * cos_t * cos_t)
                return z_pred - z_arr

            res = least_squares(residuals, p0, bounds=bounds, loss="soft_l1", f_scale=0.15)
            x0, z0, ang_deg, v_scale, v_off = res.x
            rmse = float(np.sqrt(np.mean(residuals(res.x) ** 2)))
        else:
            # Single velocity bin: keep x0, z0, angle fixed and fit velocity_scale
            x0 = self.model.launcher_offset_x
            z0 = self.model.launcher_offset_z
            ang_deg = self.model.launch_angle_deg
            v_off = self.model.velocity_offset

            def residuals_single(params):
                (v_scale,) = params
                theta = np.radians(ang_deg)
                cos_t = np.cos(theta)
                tan_t = np.tan(theta)
                v0 = np.maximum((v_arr - v_off) / np.maximum(v_scale, 0.1), 1.0)
                dx = l_arr - x0
                z_pred = z0 + dx * tan_t - (GRAVITY * dx * dx) / (2.0 * v0 * v0 * cos_t * cos_t)
                return z_pred - z_arr

            res = least_squares(
                residuals_single, [self.model.velocity_scale], bounds=([0.4], [1.6]), loss="soft_l1"
            )
            (v_scale,) = res.x
            rmse = float(np.sqrt(np.mean(residuals_single(res.x) ** 2)))

        for sp in (self.spin_off_x, self.spin_off_z, self.spin_angle, self.spin_v_quad, self.spin_v_scale, self.spin_v_offset):
            sp.blockSignals(True)
        self.spin_off_x.setValue(float(x0))
        self.spin_off_z.setValue(float(z0))
        self.spin_angle.setValue(float(ang_deg))
        self.spin_v_quad.setValue(0.0)
        self.spin_v_scale.setValue(float(v_scale))
        self.spin_v_offset.setValue(float(v_off))
        for sp in (self.spin_off_x, self.spin_off_z, self.spin_angle, self.spin_v_quad, self.spin_v_scale, self.spin_v_offset):
            sp.blockSignals(False)

        self.sync_model_from_spins()
        self.lbl_fit_status.setText(
            f"Fitted from {len(candidates)} shots ({len(pts_l)} points, {distinct_v} velocity bins): "
            f"RMSE = {rmse:.3f} m"
        )
        self.update_plots()

    def copy_yaml_to_clipboard(self):
        QtWidgets.QApplication.clipboard().setText(self.txt_yaml.toPlainText())
        self.lbl_fit_status.setText("Copied moving_bucket_launcher_params.yaml to clipboard.")

    def save_yaml_to_file(self):
        default_path = os.path.expanduser(
            "~/ros2_ws/src/gn10-navigation/config/moving_bucket_launcher_params.yaml"
        )
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Save moving_bucket_launcher_params.yaml",
            default_path,
            "YAML Files (*.yaml *.yml);;All Files (*)",
        )
        if path:
            with open(path, "w", encoding="utf-8") as f:
                f.write(self.txt_yaml.toPlainText())
            self.lbl_fit_status.setText(f"Saved YAML parameters to: {path}")

    def update_plots(self):
        self.plot_velocity_comparison()
        self.plot_3d()
        self.plot_dispersion()
        self.plot_parameter_preview()

    def _velocity_color_map(self, v_bins: List[float]) -> Dict[float, Tuple[float, float, float, float]]:
        if not v_bins:
            return {}
        cmap = cm.get_cmap("turbo", max(len(v_bins), 2))
        return {vb: cmap(i / max(len(v_bins) - 1, 1)) for i, vb in enumerate(sorted(v_bins))}

    def plot_velocity_comparison(self):
        """Plot trajectories grouped by release velocity in base_link frame."""
        self.fig_vel.clear()
        ax_lz = self.fig_vel.add_subplot(211)
        ax_xy = self.fig_vel.add_subplot(212)

        active = [r for r in self.records if r.enabled and len(r.filtered_points) >= 3]
        selected_rows = set(item.row() for item in self.table.selectedItems())
        v_bins = sorted({r.v_bin for r in active})
        v_colors = self._velocity_color_map(v_bins)

        # Plot individual trajectories colored by release velocity
        for row, r in enumerate(self.records):
            if not r.enabled or len(r.filtered_points) < 3:
                continue
            ls = [p.l for p in r.filtered_points]
            xs = [p.x for p in r.filtered_points]
            ys = [p.y for p in r.filtered_points]
            zs = [p.z for p in r.filtered_points]

            is_sel = row in selected_rows
            alpha = 0.9 if (is_sel or not selected_rows) else 0.2
            lw = 2.4 if is_sel else 1.2
            ls_style = "--" if r.label == ShotRecord.LABEL_OPENED else "-"
            color = v_colors.get(r.v_bin, (0.2, 0.7, 0.9, 1.0))

            ax_lz.plot(ls, zs, color=color, alpha=alpha, linewidth=lw, linestyle=ls_style)
            ax_xy.plot(xs, ys, color=color, alpha=alpha, linewidth=lw, linestyle=ls_style)

        # Plot mean trajectory & fitted ballistic curve per velocity bin
        l_grid = np.linspace(0.35, 5.0, 100)
        for vb in v_bins:
            group = [r for r in active if abs(r.v_bin - vb) < 1e-3]
            color = v_colors[vb]
            gx, mean_z, _ = DispersionAnalyzer.interpolate_trajectories(group, num_bins=50)
            if len(gx) > 0:
                ax_lz.plot(
                    gx,
                    mean_z,
                    color=color,
                    linewidth=2.8,
                    label=f"Measured Mean V_rel={vb:.2f} m/s (n={len(group)})",
                )
            z_fit = self.model.predict_z(l_grid, vb)
            valid_fit = (z_fit >= 0.0) & (z_fit <= 4.2)
            if np.any(valid_fit):
                ax_lz.plot(
                    l_grid[valid_fit],
                    z_fit[valid_fit],
                    color=color,
                    linestyle=":",
                    linewidth=1.8,
                    label=f"Ballistic Model V_rel={vb:.2f} m/s",
                )

        # Draw upper bucket opening height line
        ax_lz.axhline(
            self.model.bucket_height,
            color="#66bb6a",
            linestyle="-.",
            linewidth=1.5,
            label=f"Bucket Upper Opening Z={self.model.bucket_height:.3f} m",
        )

        ax_lz.set_xlabel("Horizontal Distance L = hypot(x, y) in base_link [m]")
        ax_lz.set_ylabel("Height Z in base_link [m]")
        ax_lz.set_title("base_link Trajectory Profiles by Release Velocity (Solid=Rolled, Dashed=Opened, Dotted=Model)")
        ax_lz.set_xlim(0.0, 5.2)
        ax_lz.set_ylim(0.0, 4.0)
        ax_lz.grid(True, linestyle=":", alpha=0.6)
        ax_lz.legend(loc="upper right", fontsize=8, ncol=2)

        ax_xy.axhline(0.0, color="white", linestyle=":", alpha=0.4)
        ax_xy.set_xlabel("Forward X in base_link [m]")
        ax_xy.set_ylabel("Lateral Y in base_link [m]")
        ax_xy.set_title("Top-Down Lateral Drift in base_link Frame (X-Y)")
        ax_xy.set_xlim(0.0, 5.2)
        ax_xy.set_ylim(-1.2, 1.2)
        ax_xy.grid(True, linestyle=":", alpha=0.6)

        self.fig_vel.tight_layout()
        self.canvas_vel.draw()

    def plot_3d(self):
        self.fig_3d.clear()
        ax = self.fig_3d.add_subplot(111, projection="3d")

        selected_rows = set(item.row() for item in self.table.selectedItems())
        active = [r for r in self.records if r.enabled and r.filtered_points]
        v_bins = sorted({r.v_bin for r in active})
        v_colors = self._velocity_color_map(v_bins)

        count = 0
        for row, r in enumerate(self.records):
            if not r.enabled or not r.filtered_points:
                continue
            xs = [p.x for p in r.filtered_points]
            ys = [p.y for p in r.filtered_points]
            zs = [p.z for p in r.filtered_points]

            is_selected = row in selected_rows
            alpha = 0.9 if is_selected or not selected_rows else 0.25
            lw = 2.5 if is_selected else 1.3
            color = v_colors.get(r.v_bin, (0.0, 0.74, 0.83, 1.0))

            ax.plot(xs, ys, zs, color=color, alpha=alpha, linewidth=lw)
            count += 1

        ax.set_xlabel("base_link X (Forward) [m]")
        ax.set_ylabel("base_link Y (Lateral) [m]")
        ax.set_zlabel("base_link Z (Height) [m]")
        ax.set_title(f"3D Cloth Flight Trajectories in base_link Frame ({count} active, colored by V_rel)")
        ax.set_xlim(0.0, 5.2)
        ax.set_ylim(-1.5, 1.5)
        ax.set_zlim(0.0, 4.0)
        self.canvas_3d.draw()

    def plot_dispersion(self):
        self.fig_disp.clear()
        ax_top = self.fig_disp.add_subplot(211)
        ax_bot = self.fig_disp.add_subplot(212, sharex=ax_top)

        active = [r for r in self.records if r.enabled and len(r.filtered_points) >= 4]
        for r in active:
            ls = [p.l for p in r.filtered_points]
            zs = [p.z for p in r.filtered_points]
            color = "#f44336" if r.label == ShotRecord.LABEL_OPENED else "#00bcd4"
            ax_top.plot(ls, zs, color=color, alpha=0.35, linewidth=1.0)

        grid_l, mean_z, std_z = DispersionAnalyzer.interpolate_trajectories(active)
        if len(grid_l) > 0:
            ax_top.plot(grid_l, mean_z, color="white", linewidth=2.0, label="Mean Trajectory z(L)")
            ax_top.fill_between(
                grid_l, mean_z - std_z, mean_z + std_z, color="#29b6f6", alpha=0.2, label="±1σ Band"
            )
            ax_bot.plot(grid_l, std_z, color="#ab47bc", linewidth=2.0, label="Dispersion σ_z(L)")

            branch_l = DispersionAnalyzer.detect_branch_point(grid_l, std_z)
            if branch_l is not None:
                ax_top.axvline(
                    branch_l,
                    color="#ff5252",
                    linestyle="--",
                    linewidth=2.0,
                    label=f"Opening Branch Point (L={branch_l:.2f} m)",
                )
                ax_bot.axvline(
                    branch_l,
                    color="#ff5252",
                    linestyle="--",
                    linewidth=2.0,
                    label=f"Opening Branch Point (L={branch_l:.2f} m)",
                )

        ax_top.set_ylabel("base_link Z [m]")
        ax_top.set_title("base_link L-Z Flight Profile & Opening Dispersion Analysis")
        ax_top.set_ylim(0.0, 4.2)
        ax_top.grid(True, linestyle=":", alpha=0.6)
        ax_top.legend(loc="upper right", fontsize=8)

        ax_bot.set_xlabel("base_link Horizontal Distance L = hypot(x, y) [m]")
        ax_bot.set_ylabel("Dispersion σ_z [m]")
        ax_bot.grid(True, linestyle=":", alpha=0.6)
        ax_bot.legend(loc="upper left", fontsize=8)

        self.fig_disp.tight_layout()
        self.canvas_disp.draw()

    def plot_parameter_preview(self):
        """Plot fitted trajectories vs measured points and required belt velocity vs bucket distance."""
        self.fig_param.clear()
        ax_traj = self.fig_param.add_subplot(211)
        ax_req = self.fig_param.add_subplot(212)

        active = [r for r in self.records if r.enabled and len(r.filtered_points) >= 3]
        v_bins = sorted({r.v_bin for r in active})
        if not v_bins:
            v_bins = [3.0, 3.25, 3.5, 4.0]
        v_colors = self._velocity_color_map(v_bins)

        for r in active:
            ls = [p.l for p in r.filtered_points]
            zs = [p.z for p in r.filtered_points]
            ax_traj.scatter(ls, zs, color=v_colors.get(r.v_bin, "cyan"), s=14, alpha=0.55)

        l_grid = np.linspace(0.4, 5.0, 120)
        for vb in v_bins:
            z_pred = self.model.predict_z(l_grid, vb)
            mask = (z_pred >= 0.0) & (z_pred <= 4.0)
            if np.any(mask):
                ax_traj.plot(
                    l_grid[mask],
                    z_pred[mask],
                    color=v_colors[vb],
                    linewidth=2.2,
                    label=f"Model V_belt={vb:.2f} m/s",
                )

        ax_traj.axhline(
            self.model.bucket_height + self.model.aim_z_offset,
            color="#66bb6a",
            linestyle="--",
            linewidth=1.8,
            label=f"Target Upper Opening Z = {self.model.bucket_height + self.model.aim_z_offset:.3f} m",
        )
        ax_traj.set_xlabel("base_link Horizontal Distance L [m]")
        ax_traj.set_ylabel("base_link Height Z [m]")
        ax_traj.set_title("Model Trajectories vs Measured Points in base_link Frame")
        ax_traj.set_xlim(0.3, 5.0)
        ax_traj.set_ylim(0.0, 3.8)
        ax_traj.grid(True, linestyle=":", alpha=0.6)
        ax_traj.legend(loc="upper right", fontsize=8, ncol=2)

        # Bottom subplot: Required belt launcher speed vs bucket horizontal distance L for several opening heights
        dist_grid = np.linspace(0.8, 4.8, 100)
        for z_open, label_str, col in [
            (self.model.bucket_height, f"Configured Opening (z={self.model.bucket_height:.3f}m)", "#66bb6a"),
            (0.555, "Mid Bucket Opening (z=0.555m)", "#ffa726"),
            (0.855, "High Bucket Opening (z=0.855m)", "#ef5350"),
        ]:
            req_v = [self.model.solve_belt_speed(d, z_open) for d in dist_grid]
            valid_d = [d for d, v in zip(dist_grid, req_v) if v is not None]
            valid_v = [v for v in req_v if v is not None]
            if valid_d:
                ax_req.plot(valid_d, valid_v, color=col, linewidth=2.2, label=label_str)

        ax_req.set_xlabel("Bucket Upper Opening Distance L in base_link [m]")
        ax_req.set_ylabel("Commanded Belt Speed V_belt [m/s]")
        ax_req.set_title("MovingBucketLauncherNode Speed Curve: V_belt(L, z_open)")
        ax_req.set_xlim(0.8, 4.8)
        ax_req.set_ylim(2.0, 6.0)
        ax_req.grid(True, linestyle=":", alpha=0.6)
        ax_req.legend(loc="upper left", fontsize=8)

        self.fig_param.tight_layout()
        self.canvas_param.draw()


def main():
    default_log = os.path.expanduser("~/ros2_ws/data/shot-trial-log/trials.jsonl")
    if not os.path.exists(default_log):
        default_log = "/home/gn10/ros2_ws/data/shot-trial-log/trials.jsonl"

    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")

    palette = QtGui.QPalette()
    palette.setColor(QtGui.QPalette.Window, QtGui.QColor("#2b2b30"))
    palette.setColor(QtGui.QPalette.WindowText, QtCore.Qt.white)
    palette.setColor(QtGui.QPalette.Base, QtGui.QColor("#1e1e24"))
    palette.setColor(QtGui.QPalette.AlternateBase, QtGui.QColor("#2b2b30"))
    palette.setColor(QtGui.QPalette.ToolTipBase, QtCore.Qt.white)
    palette.setColor(QtGui.QPalette.ToolTipText, QtCore.Qt.white)
    palette.setColor(QtGui.QPalette.Text, QtCore.Qt.white)
    palette.setColor(QtGui.QPalette.Button, QtGui.QColor("#3a3a40"))
    palette.setColor(QtGui.QPalette.ButtonText, QtCore.Qt.white)
    palette.setColor(QtGui.QPalette.BrightText, QtCore.Qt.red)
    palette.setColor(QtGui.QPalette.Highlight, QtGui.QColor("#2979ff"))
    palette.setColor(QtGui.QPalette.HighlightedText, QtCore.Qt.black)
    app.setPalette(palette)

    win = TrajectoryVisualizerApp(default_log)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
