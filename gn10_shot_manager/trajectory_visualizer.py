"""3D Trajectory Visualizer & Dispersion Analyzer for gn10-shot-manager.

Analyzes cloth flight trajectories, detects dispersion branching points
(where rolled cloth opens and air resistance increases), filters noise,
and provides sample curation for bucket / flag targeting.
"""

import json
import math
import os
import sys
from typing import Dict, List, Optional, Tuple

import matplotlib
matplotlib.use("Qt5Agg")
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import numpy as np

from PyQt5 import QtCore, QtGui, QtWidgets


class TrajectoryPoint:
    """Single point in trajectory."""
    def __init__(self, t: float, x: float, y: float, z: float, v_act: float, volt: float, curr: float):
        self.t = t
        self.x = x
        self.y = y
        self.z = z
        self.v_act = v_act
        self.volt = volt
        self.curr = curr


class ShotRecord:
    """Data representation of a single shot trial."""

    LABEL_UNCLASSIFIED = "Unclassified"
    LABEL_ROLLED = "Rolled (Closed)"
    LABEL_OPENED = "Opened"
    LABEL_EXCLUDED = "Excluded (Noise/Invalid)"

    def __init__(self, raw_data: dict, index: int):
        self.index = index
        self.shot_id = raw_data.get("shot_id", index)
        self.timestamp = raw_data.get("timestamp", "")
        self.target = raw_data.get("target", {})
        self.summary = raw_data.get("summary", {})
        self.landing = raw_data.get("landing_position", {})

        self.v_tgt = 0.0
        self.v_max = self.summary.get("max_velocity", 0.0)
        self.avg_voltage = self.summary.get("avg_voltage", 0.0)
        self.max_current = self.summary.get("max_current", 0.0)

        # Parse raw samples
        self.raw_points: List[TrajectoryPoint] = []
        samples = raw_data.get("samples", [])
        if samples and "v_tgt" in samples[0]:
            self.v_tgt = samples[0].get("v_tgt", 0.0)

        for s in samples:
            cx, cy, cz = s.get("cx"), s.get("cy"), s.get("cz")
            if cx is not None and cy is not None and cz is not None:
                self.raw_points.append(
                    TrajectoryPoint(
                        t=s.get("t", 0.0),
                        x=float(cx),
                        y=float(cy),
                        z=float(cz),
                        v_act=float(s.get("v_act", 0.0)),
                        volt=float(s.get("volt", 0.0)),
                        curr=float(s.get("curr", 0.0)),
                    )
                )

        # Status and curated state
        self.is_valid_voltage = 15.0 <= self.avg_voltage <= 24.0
        self.label = self.LABEL_UNCLASSIFIED
        self.enabled = len(self.raw_points) >= 3 and self.is_valid_voltage
        if not self.is_valid_voltage or len(self.raw_points) < 3:
            self.label = self.LABEL_EXCLUDED

        # Filtered points cache
        self.filtered_points: List[TrajectoryPoint] = []
        self.filter_points()

    def filter_points(self, min_z: float = 0.15, max_jump: float = 1.0):
        """Filter ground noise and jump outliers."""
        if not self.raw_points:
            self.filtered_points = []
            return

        cleaned: List[TrajectoryPoint] = []
        prev: Optional[TrajectoryPoint] = None

        for pt in self.raw_points:
            # Cut ground reflections
            if pt.z < min_z:
                continue
            # Jump filter
            if prev is not None:
                dist = math.sqrt((pt.x - prev.x)**2 + (pt.y - prev.y)**2 + (pt.z - prev.z)**2)
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
    def peak_z(self) -> float:
        if not self.filtered_points:
            return 0.0
        return max(p.z for p in self.filtered_points)


class DispersionAnalyzer:
    """Analyzes trajectory dispersion to detect branch points where cloth opens."""

    @staticmethod
    def interpolate_trajectories(records: List[ShotRecord], num_bins: int = 100) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Interpolate all enabled records on common X grid to compute mean and variance."""
        valid_records = [r for r in records if r.enabled and len(r.filtered_points) >= 5]
        if len(valid_records) < 2:
            return np.array([]), np.array([]), np.array([])

        x_min = max(min(r.filtered_points[0].x for r in valid_records), 0.5)
        x_max = min(max(r.max_x for r in valid_records), 6.0)

        if x_min >= x_max:
            return np.array([]), np.array([]), np.array([])

        grid_x = np.linspace(x_min, x_max, num_bins)
        interpolated_z = []

        for r in valid_records:
            xs = [p.x for p in r.filtered_points]
            zs = [p.z for p in r.filtered_points]
            # Ensure strictly increasing xs for interp
            unique_xs, unique_indices = np.unique(xs, return_index=True)
            unique_zs = np.array(zs)[unique_indices]

            if len(unique_xs) < 2:
                continue

            # Linear interpolation with NaN outside range
            z_interp = np.interp(grid_x, unique_xs, unique_zs, left=np.nan, right=np.nan)
            interpolated_z.append(z_interp)

        if not interpolated_z:
            return np.array([]), np.array([]), np.array([])

        z_matrix = np.array(interpolated_z)
        mean_z = np.nanmean(z_matrix, axis=0)
        std_z = np.nanstd(z_matrix, axis=0)

        return grid_x, mean_z, std_z

    @staticmethod
    def detect_branch_point(grid_x: np.ndarray, std_z: np.ndarray, std_thresh_ratio: float = 2.5) -> Optional[float]:
        """Detect the X coordinate where dispersion abruptly increases."""
        valid_mask = np.isfinite(std_z) & (std_z > 0.001)
        if np.sum(valid_mask) < 10:
            return None

        gx = grid_x[valid_mask]
        sz = std_z[valid_mask]

        # Initial baseline standard deviation (first 20% of range)
        init_len = max(3, len(sz) // 5)
        baseline_std = np.median(sz[:init_len])

        # Find first point where std exceeds threshold ratio of baseline
        threshold = baseline_std * std_thresh_ratio
        for x, s in zip(gx[init_len:], sz[init_len:]):
            if s >= threshold:
                return float(x)
        return None


class TrajectoryVisualizerApp(QtWidgets.QMainWindow):
    """Main window for 3D trajectory visualizer and dispersion analysis."""

    def __init__(self, log_path: str):
        super().__init__()
        self.setWindowTitle("gn10 Cloth Shot Trajectory Visualizer & Dispersion Analyzer")
        self.resize(1300, 850)

        self.log_path = log_path
        self.records: List[ShotRecord] = []

        self.init_ui()
        self.load_log_file(log_path)

    def init_ui(self):
        # Central widget
        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)
        layout = QtWidgets.QHBoxLayout(central)

        # Left panel: Curation & Control (380px)
        left_panel = QtWidgets.QWidget()
        left_panel.setFixedWidth(400)
        left_layout = QtWidgets.QVBoxLayout(left_panel)
        layout.addWidget(left_panel)

        # File header & Reload
        file_box = QtWidgets.QGroupBox("Data Source")
        file_layout = QtWidgets.QVBoxLayout(file_box)
        self.lbl_path = QtWidgets.QLabel(f"File: {os.path.basename(self.log_path)}")
        self.lbl_path.setToolTip(self.log_path)
        file_layout.addWidget(self.lbl_path)

        btn_reload = QtWidgets.QPushButton("🔄 Reload JSONL")
        btn_reload.clicked.connect(lambda: self.load_log_file(self.log_path))
        file_layout.addWidget(btn_reload)
        left_layout.addWidget(file_box)

        # Noise & Ground Filter Settings
        filter_box = QtWidgets.QGroupBox("Noise & Ground Filter")
        filter_layout = QtWidgets.QFormLayout(filter_box)

        self.spin_min_z = QtWidgets.QDoubleSpinBox()
        self.spin_min_z.setRange(0.0, 1.5)
        self.spin_min_z.setSingleStep(0.05)
        self.spin_min_z.setValue(0.15)
        self.spin_min_z.setSuffix(" m")
        self.spin_min_z.valueChanged.connect(self.on_filter_changed)
        filter_layout.addRow("Min Z (Ground):", self.spin_min_z)

        self.spin_max_jump = QtWidgets.QDoubleSpinBox()
        self.spin_max_jump.setRange(0.2, 5.0)
        self.spin_max_jump.setSingleStep(0.1)
        self.spin_max_jump.setValue(1.0)
        self.spin_max_jump.setSuffix(" m")
        self.spin_max_jump.valueChanged.connect(self.on_filter_changed)
        filter_layout.addRow("Max Jump Dist:", self.spin_max_jump)

        left_layout.addWidget(filter_box)

        # Table for shot curation
        table_box = QtWidgets.QGroupBox("Shot Curation & Labeling")
        table_layout = QtWidgets.QVBoxLayout(table_box)

        # Batch actions
        btn_layout = QtWidgets.QHBoxLayout()
        btn_select_all = QtWidgets.QPushButton("Select All")
        btn_select_all.clicked.connect(self.select_all_trials)
        btn_deselect_all = QtWidgets.QPushButton("Deselect All")
        btn_deselect_all.clicked.connect(self.deselect_all_trials)
        btn_auto_label = QtWidgets.QPushButton("Auto Detect Opening")
        btn_auto_label.setToolTip("Auto label opened/rolled based on distance and dispersion")
        btn_auto_label.clicked.connect(self.auto_label_trials)

        btn_layout.addWidget(btn_select_all)
        btn_layout.addWidget(btn_deselect_all)
        btn_layout.addWidget(btn_auto_label)
        table_layout.addLayout(btn_layout)

        # Shot Table
        self.table = QtWidgets.QTableWidget()
        self.table.setColumnCount(6)
        self.table.setHorizontalHeaderLabels(["Use", "Shot", "V_tgt", "Volt", "Pts", "State"])
        self.table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(2, QtWidgets.QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(3, QtWidgets.QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(4, QtWidgets.QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(5, QtWidgets.QHeaderView.Stretch)
        self.table.itemSelectionChanged.connect(self.on_table_selection_changed)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        table_layout.addWidget(self.table)

        # Quick label assignment buttons
        label_btn_layout = QtWidgets.QHBoxLayout()
        btn_set_rolled = QtWidgets.QPushButton("🏷️ Rolled")
        btn_set_rolled.setStyleSheet("background-color: #1e88e5; color: white;")
        btn_set_rolled.clicked.connect(lambda: self.set_selected_label(ShotRecord.LABEL_ROLLED))

        btn_set_opened = QtWidgets.QPushButton("🏷️ Opened")
        btn_set_opened.setStyleSheet("background-color: #e53935; color: white;")
        btn_set_opened.clicked.connect(lambda: self.set_selected_label(ShotRecord.LABEL_OPENED))

        btn_set_exclude = QtWidgets.QPushButton("❌ Exclude")
        btn_set_exclude.setStyleSheet("background-color: #546e7a; color: white;")
        btn_set_exclude.clicked.connect(lambda: self.set_selected_label(ShotRecord.LABEL_EXCLUDED))

        label_btn_layout.addWidget(btn_set_rolled)
        label_btn_layout.addWidget(btn_set_opened)
        label_btn_layout.addWidget(btn_set_exclude)
        table_layout.addLayout(label_btn_layout)

        left_layout.addWidget(table_box)

        # Summary statistics
        self.stat_box = QtWidgets.QGroupBox("Statistical Summary")
        stat_layout = QtWidgets.QVBoxLayout(self.stat_box)
        self.lbl_stats = QtWidgets.QLabel("Loading...")
        self.lbl_stats.setStyleSheet("font-size: 11px;")
        stat_layout.addWidget(self.lbl_stats)
        left_layout.addWidget(self.stat_box)

        # Right panel: Visualization Tabs
        self.tabs = QtWidgets.QTabWidget()
        layout.addWidget(self.tabs, stretch=1)

        # Tab 1: 3D Flight Visualizer
        self.tab_3d = QtWidgets.QWidget()
        layout_3d = QtWidgets.QVBoxLayout(self.tab_3d)
        self.fig_3d = Figure(figsize=(7, 6))
        self.canvas_3d = FigureCanvas(self.fig_3d)
        self.toolbar_3d = NavigationToolbar(self.canvas_3d, self.tab_3d)
        layout_3d.addWidget(self.toolbar_3d)
        layout_3d.addWidget(self.canvas_3d)
        self.tabs.addTab(self.tab_3d, "3D Trajectory View")

        # Tab 2: Dispersion & Branch Analysis (2D)
        self.tab_disp = QtWidgets.QWidget()
        layout_disp = QtWidgets.QVBoxLayout(self.tab_disp)
        self.fig_disp = Figure(figsize=(7, 6))
        self.canvas_disp = FigureCanvas(self.fig_disp)
        self.toolbar_disp = NavigationToolbar(self.canvas_disp, self.tab_disp)
        layout_disp.addWidget(self.toolbar_disp)
        layout_disp.addWidget(self.canvas_disp)
        self.tabs.addTab(self.tab_disp, "Dispersion & Branching Analysis")

        # Tab 3: Velocity & Voltage Correlation
        self.tab_corr = QtWidgets.QWidget()
        layout_corr = QtWidgets.QVBoxLayout(self.tab_corr)
        self.fig_corr = Figure(figsize=(7, 6))
        self.canvas_corr = FigureCanvas(self.fig_corr)
        self.toolbar_corr = NavigationToolbar(self.canvas_corr, self.tab_corr)
        layout_corr.addWidget(self.toolbar_corr)
        layout_corr.addWidget(self.canvas_corr)
        self.tabs.addTab(self.tab_corr, "Targeting Probability (Bucket vs Flag)")

    def load_log_file(self, path: str):
        """Read trials.jsonl and populate table."""
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
                    print(f"Error parsing line {idx+1}: {e}")

        self.refresh_table()
        self.update_plots()

    def refresh_table(self):
        """Populate the curation table."""
        self.table.blockSignals(True)
        self.table.setRowCount(len(self.records))

        for row, r in enumerate(self.records):
            # Checkbox
            chk = QtWidgets.QTableWidgetItem()
            chk.setFlags(QtCore.Qt.ItemIsUserCheckable | QtCore.Qt.ItemIsEnabled)
            chk.setCheckState(QtCore.Qt.Checked if r.enabled else QtCore.Qt.Unchecked)
            self.table.setItem(row, 0, chk)

            # Shot ID
            item_id = QtWidgets.QTableWidgetItem(f"#{r.shot_id}")
            self.table.setItem(row, 1, item_id)

            # Target Velocity
            item_v = QtWidgets.QTableWidgetItem(f"{r.v_tgt:.2f}")
            self.table.setItem(row, 2, item_v)

            # Voltage
            item_volt = QtWidgets.QTableWidgetItem(f"{r.avg_voltage:.1f}")
            if not r.is_valid_voltage:
                item_volt.setForeground(QtGui.QBrush(QtGui.QColor("red")))
                item_volt.setToolTip("Abnormal voltage (<15V). Communication packet mismatch.")
            self.table.setItem(row, 3, item_volt)

            # Points count
            item_pts = QtWidgets.QTableWidgetItem(str(len(r.filtered_points)))
            self.table.setItem(row, 4, item_pts)

            # Label / State
            item_state = QtWidgets.QTableWidgetItem(r.label)
            if r.label == ShotRecord.LABEL_ROLLED:
                item_state.setForeground(QtGui.QBrush(QtGui.QColor("#29b6f6")))
            elif r.label == ShotRecord.LABEL_OPENED:
                item_state.setForeground(QtGui.QBrush(QtGui.QColor("#ef5350")))
            elif r.label == ShotRecord.LABEL_EXCLUDED:
                item_state.setForeground(QtGui.QBrush(QtGui.QColor("#78909c")))
            self.table.setItem(row, 5, item_state)

        self.table.blockSignals(False)
        self.table.cellChanged.connect(self.on_table_cell_changed)
        self.update_summary_stats()

    def on_table_cell_changed(self, row: int, col: int):
        if col == 0:
            chk = self.table.item(row, 0)
            if chk:
                self.records[row].enabled = (chk.checkState() == QtCore.Qt.Checked)
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

    def select_all_trials(self):
        for r in self.records:
            if r.is_valid_voltage and len(r.filtered_points) >= 3:
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
            if label == ShotRecord.LABEL_EXCLUDED:
                self.records[row].enabled = False
            else:
                self.records[row].enabled = True
        self.refresh_table()
        self.update_plots()

    def auto_label_trials(self):
        """Auto classify rolled vs opened based on distance and dispersion."""
        for r in self.records:
            if not r.is_valid_voltage or len(r.filtered_points) < 5:
                r.label = ShotRecord.LABEL_EXCLUDED
                r.enabled = False
                continue

            # In high velocity (>6.0 m/s), if max_x < 3.0m, it stalled and opened early
            if r.v_tgt >= 6.0 and r.max_x < 3.2:
                r.label = ShotRecord.LABEL_OPENED
            elif r.max_x >= 3.8:
                r.label = ShotRecord.LABEL_ROLLED
            else:
                r.label = ShotRecord.LABEL_UNCLASSIFIED

        self.refresh_table()
        self.update_plots()

    def update_summary_stats(self):
        enabled = [r for r in self.records if r.enabled]
        rolled = [r for r in enabled if r.label == ShotRecord.LABEL_ROLLED]
        opened = [r for r in enabled if r.label == ShotRecord.LABEL_OPENED]

        total_en = len(enabled)
        roll_pct = (len(rolled) / total_en * 100.0) if total_en > 0 else 0.0
        open_pct = (len(opened) / total_en * 100.0) if total_en > 0 else 0.0

        txt = (
            f"<b>Active Samples:</b> {total_en} / {len(self.records)}<br>"
            f"<b>Rolled (Bucket Target):</b> {len(rolled)} ({roll_pct:.1f}%)<br>"
            f"<b>Opened (Flag Target):</b> {len(opened)} ({open_pct:.1f}%)<br>"
            f"<b>Flag Reachability (Z>=2.8m):</b> "
            f"{sum(1 for r in enabled if r.peak_z >= 2.8)} shots<br>"
            f"<b>Bucket Candidate:</b> {len(rolled)} clean trajectories"
        )
        self.lbl_stats.setText(txt)

    def update_plots(self):
        self.plot_3d()
        self.plot_dispersion()
        self.plot_correlation()

    def plot_3d(self):
        self.fig_3d.clear()
        ax = self.fig_3d.add_subplot(111, projection="3d")

        selected_rows = set(item.row() for item in self.table.selectedItems())

        # Draw targets for reference
        # Flag target (~3m height at x=4.0m)
        ax.plot([4.0, 4.0], [0.0, 0.0], [0.0, 3.2], color="gold", linewidth=3, label="Flag (H~3m)")
        ax.scatter([4.0], [0.0], [3.0], color="red", s=80, marker="^")

        # Bucket target (~0.8m height at x=3.5m)
        ax.scatter([3.5], [0.0], [0.8], color="green", s=100, marker="o", label="Target Bucket (H~0.8m)")

        count = 0
        for row, r in enumerate(self.records):
            if not r.enabled or not r.filtered_points:
                continue

            xs = [p.x for p in r.filtered_points]
            ys = [p.y for p in r.filtered_points]
            zs = [p.z for p in r.filtered_points]

            is_selected = row in selected_rows
            alpha = 0.9 if is_selected or not selected_rows else 0.25
            lw = 2.5 if is_selected else 1.2

            if r.label == ShotRecord.LABEL_ROLLED:
                color = "#00bcd4"
            elif r.label == ShotRecord.LABEL_OPENED:
                color = "#f44336"
            else:
                color = "#9e9e9e"

            ax.plot(xs, ys, zs, color=color, alpha=alpha, linewidth=lw)
            count += 1

        ax.set_xlabel("X (Forward) [m]")
        ax.set_ylabel("Y (Lateral) [m]")
        ax.set_zlabel("Z (Height) [m]")
        ax.set_title(f"3D Cloth Flight Trajectories ({count} active)")
        ax.set_zlim(0.0, 4.0)
        ax.legend(loc="upper left")
        self.canvas_3d.draw()

    def plot_dispersion(self):
        self.fig_disp.clear()
        ax_top = self.fig_disp.add_subplot(211)
        ax_bot = self.fig_disp.add_subplot(212, sharex=ax_top)

        active = [r for r in self.records if r.enabled and len(r.filtered_points) >= 5]
        for r in active:
            xs = [p.x for p in r.filtered_points]
            zs = [p.z for p in r.filtered_points]
            color = "#f44336" if r.label == ShotRecord.LABEL_OPENED else "#00bcd4"
            ax_top.plot(xs, zs, color=color, alpha=0.35, linewidth=1.0)

        grid_x, mean_z, std_z = DispersionAnalyzer.interpolate_trajectories(active)
        if len(grid_x) > 0:
            ax_top.plot(grid_x, mean_z, color="black", linewidth=2.0, label="Mean Trajectory")
            ax_top.fill_between(grid_x, mean_z - std_z, mean_z + std_z, color="blue", alpha=0.15, label="±1σ Band")
            ax_top.fill_between(grid_x, mean_z - 2*std_z, mean_z + 2*std_z, color="blue", alpha=0.08, label="±2σ Band")

            # Dispersion curve on bottom axis
            ax_bot.plot(grid_x, std_z, color="purple", linewidth=2.0, label="Dispersion σ_z(X)")

            # Detect branch point
            branch_x = DispersionAnalyzer.detect_branch_point(grid_x, std_z)
            if branch_x is not None:
                ax_top.axvline(branch_x, color="red", linestyle="--", linewidth=2.0, label=f"Branch Point (X={branch_x:.2f}m)")
                ax_bot.axvline(branch_x, color="red", linestyle="--", linewidth=2.0, label=f"Branch Point (X={branch_x:.2f}m)")
                ax_bot.scatter([branch_x], [np.interp(branch_x, grid_x, std_z)], color="red", s=60, zorder=5)

        ax_top.set_ylabel("Height Z [m]")
        ax_top.set_title("X-Z Flight Profile & Statistical Dispersion")
        ax_top.set_ylim(0.0, 4.2)
        ax_top.grid(True, linestyle=":", alpha=0.6)
        ax_top.legend(loc="upper right", fontsize=8)

        ax_bot.set_xlabel("Forward Distance X [m]")
        ax_bot.set_ylabel("Dispersion σ_z [m]")
        ax_bot.grid(True, linestyle=":", alpha=0.6)
        ax_bot.legend(loc="upper left", fontsize=8)

        self.fig_disp.tight_layout()
        self.canvas_disp.draw()

    def plot_correlation(self):
        self.fig_corr.clear()
        ax1 = self.fig_corr.add_subplot(121)
        ax2 = self.fig_corr.add_subplot(122)

        active = [r for r in self.records if r.enabled and r.v_tgt > 0]
        if not active:
            self.canvas_corr.draw()
            return

        # Target Velocity vs Max Reach Distance
        rolled = [r for r in active if r.label == ShotRecord.LABEL_ROLLED]
        opened = [r for r in active if r.label == ShotRecord.LABEL_OPENED]

        if rolled:
            ax1.scatter([r.v_tgt for r in rolled], [r.max_x for r in rolled], color="#00bcd4", label="Rolled", s=40, alpha=0.8)
        if opened:
            ax1.scatter([r.v_tgt for r in opened], [r.max_x for r in opened], color="#f44336", label="Opened (Stalled)", s=40, alpha=0.8)

        ax1.set_xlabel("Target Velocity V_tgt [m/s]")
        ax1.set_ylabel("Max Reach Distance X [m]")
        ax1.set_title("Velocity vs Reach Distance")
        ax1.grid(True, linestyle=":", alpha=0.6)
        ax1.legend()

        # Opening Ratio by Velocity Group
        v_bins = {}
        for r in active:
            v_rounded = round(r.v_tgt * 2) / 2.0  # round to 0.5m/s bins
            if v_rounded not in v_bins:
                v_bins[v_rounded] = {"rolled": 0, "opened": 0, "total": 0}
            v_bins[v_rounded]["total"] += 1
            if r.label == ShotRecord.LABEL_OPENED:
                v_bins[v_rounded]["opened"] += 1
            elif r.label == ShotRecord.LABEL_ROLLED:
                v_bins[v_rounded]["rolled"] += 1

        sorted_v = sorted(v_bins.keys())
        open_rates = []
        for v in sorted_v:
            t = v_bins[v]["total"]
            o = v_bins[v]["opened"]
            open_rates.append((o / t * 100.0) if t > 0 else 0.0)

        ax2.bar(sorted_v, open_rates, width=0.35, color="#ff7043", alpha=0.8, edgecolor="#d84315")
        ax2.set_xlabel("Target Velocity V_tgt [m/s]")
        ax2.set_ylabel("Opening Rate [%]")
        ax2.set_title("Probability of Opening by Velocity")
        ax2.set_ylim(0.0, 105.0)
        ax2.grid(True, linestyle=":", alpha=0.6)

        self.fig_corr.tight_layout()
        self.canvas_corr.draw()


def main():
    default_log = os.path.expanduser("~/ros2_ws/data/shot-trial-log/trials.jsonl")
    if not os.path.exists(default_log):
        default_log = "/home/gn10/ros2_ws/data/shot-trial-log/trials.jsonl"

    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")

    # Dark Palette
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
