"""Rich 2D Interactive GUI for Shooting Point Tuning & Field Management."""

import json
import math
import os
import sys
from typing import Dict, List, Optional

from PyQt5 import QtCore, QtGui, QtWidgets
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8

from gn10_shot_manager.point_manager import PointManager


class SaveWorker(QtCore.QThread):
    """Background worker for saving points and committing to Git without freezing UI."""

    finished = QtCore.pyqtSignal(bool, str)

    def __init__(self, pm: PointManager, target_ids: List[int], commit_msg: str):
        super().__init__()
        self.pm = pm
        self.target_ids = target_ids
        self.commit_msg = commit_msg

    def run(self):
        try:
            # 1. Quick check for ROS 2 service without blocking UI
            sync_info = "ROS2 offline"
            if self.pm.is_service_ready(timeout_sec=0.03):
                ok_sync, msg_sync = self.pm.sync_points_to_ros2(self.target_ids)
                sync_info = "ROS2 synced" if ok_sync else f"ROS2 sync: {msg_sync}"

            # 2. Save YAML
            ok_save, msg_save = self.pm.save_via_ros2()

            # 3. Git commit
            ok_git, msg_git = self.pm.git_commit(self.commit_msg)

            status_msg = f"{msg_save} | {msg_git} [{sync_info}]"
            self.finished.emit(ok_save and ok_git, status_msg)
        except Exception as e:
            self.finished.emit(False, f"Save failed: {e}")



class FieldCanvas(QtWidgets.QWidget):
    """Interactive 2D field canvas for viewing and placing shooting points."""

    point_moved = QtCore.pyqtSignal(int, float, float)  # target_id, x, y
    yaw_changed = QtCore.pyqtSignal(int, float)         # target_id, yaw
    point_selected = QtCore.pyqtSignal(int)             # target_id

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(480, 520)
        self.setMouseTracking(True)

        # Field physical bounds [meters]
        self.field_x_min = -5.4
        self.field_x_max = 5.4
        self.field_y_min = -5.8
        self.field_y_max = 5.8

        self.points: Dict[int, dict] = {}
        self.selected_target_id: Optional[int] = 0
        self.field_objects: List[dict] = []

        self.dragging = False
        self.rotating = False

        self.load_field_map()

    def load_field_map(self):
        """Load obstacle/target objects from nhk2026_map.json if available."""
        map_paths = [
            os.path.expanduser("~/ros2_ws/src/gn10-navigation/config/nhk2026_map.json"),
            "/home/gn10/ros2_ws/src/gn10-navigation/config/nhk2026_map.json",
        ]
        for p in map_paths:
            if os.path.exists(p):
                try:
                    with open(p, "r", encoding="utf-8") as f:
                        self.field_objects = json.load(f)
                    return
                except Exception:
                    pass

    def set_points(self, points: Dict[int, dict], selected_id: Optional[int]):
        self.points = points
        self.selected_target_id = selected_id
        self.update()

    def set_selected_id(self, selected_id: Optional[int]):
        self.selected_target_id = selected_id
        self.update()

    # Coordinate transform: Field [m] -> Widget [pixels]
    def world_to_screen(self, x: float, y: float) -> QtCore.QPointF:
        w = self.width()
        h = self.height()
        margin = 35

        scale_x = (w - 2 * margin) / (self.field_x_max - self.field_x_min)
        scale_y = (h - 2 * margin) / (self.field_y_max - self.field_y_min)
        scale = min(scale_x, scale_y)

        cx = w / 2.0
        cy = h / 2.0

        # ROS coordinates: X is right/forward, Y is left/up
        sx = cx + x * scale
        sy = cy - y * scale
        return QtCore.QPointF(sx, sy)

    # Coordinate transform: Widget [pixels] -> Field [m]
    def screen_to_world(self, sx: float, sy: float) -> tuple[float, float]:
        w = self.width()
        h = self.height()
        margin = 35

        scale_x = (w - 2 * margin) / (self.field_x_max - self.field_x_min)
        scale_y = (h - 2 * margin) / (self.field_y_max - self.field_y_min)
        scale = min(scale_x, scale_y)

        cx = w / 2.0
        cy = h / 2.0

        x = (sx - cx) / scale
        y = (cy - sy) / scale
        return round(x, 4), round(y, 4)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        w = self.width()
        h = self.height()

        # Background
        painter.fillRect(self.rect(), QtGui.QColor("#1e1e24"))

        # 1. Field Zones (Red Zone: Y < 0, Blue Zone: Y > 0)
        p_top_left = self.world_to_screen(self.field_x_min, self.field_y_max)
        p_bottom_right = self.world_to_screen(self.field_x_max, self.field_y_min)
        p_center_left = self.world_to_screen(self.field_x_min, 0.0)
        p_center_right = self.world_to_screen(self.field_x_max, 0.0)

        rect_blue = QtCore.QRectF(p_top_left, p_center_right)
        rect_red = QtCore.QRectF(p_center_left, p_bottom_right)

        # Subtle colors for zones
        painter.fillRect(rect_blue, QtGui.QColor(30, 50, 85, 140))
        painter.fillRect(rect_red, QtGui.QColor(85, 30, 35, 140))

        # Field outer border
        border_pen = QtGui.QPen(QtGui.QColor("#808090"), 2)
        painter.setPen(border_pen)
        painter.drawRect(QtCore.QRectF(p_top_left, p_bottom_right))

        # Zone Labels
        painter.setPen(QtGui.QColor(100, 150, 255, 180))
        painter.setFont(QtGui.QFont("Sans", 11, QtGui.QFont.Bold))
        painter.drawText(
            QtCore.QRectF(p_top_left.x() + 10, p_top_left.y() + 10, 200, 25),
            QtCore.Qt.AlignLeft,
            "BLUE ZONE (Y > 0)",
        )

        painter.setPen(QtGui.QColor(255, 100, 110, 180))
        painter.drawText(
            QtCore.QRectF(p_center_left.x() + 10, p_center_left.y() + 10, 200, 25),
            QtCore.Qt.AlignLeft,
            "RED ZONE (Y < 0)",
        )

        # Center line (Center Stage / 教壇)
        center_pen = QtGui.QPen(QtGui.QColor("#ffaa00"), 2, QtCore.Qt.DashLine)
        painter.setPen(center_pen)
        painter.drawLine(p_center_left, p_center_right)

        # 2. Field Map Objects (Buckets, Desks, Flags, Obstacles)
        self.draw_field_objects(painter)

        # 3. Target Shooting Points
        self.draw_shooting_points(painter)

    def draw_field_objects(self, painter: QtGui.QPainter):
        """Render obstacles from nhk2026_map.json."""
        for obj in self.field_objects:
            obj_type = obj.get("type", "")
            ox = obj.get("x", 0.0)
            oy = obj.get("y", 0.0)
            comment = obj.get("comment", "")

            pos = self.world_to_screen(ox, oy)

            if "教壇" in comment:
                # Stage
                hx = obj.get("half_x", 5.25)
                hy = obj.get("half_y", 0.3)
                p1 = self.world_to_screen(ox - hx, oy + hy)
                p2 = self.world_to_screen(ox + hx, oy - hy)
                painter.setBrush(QtGui.QColor(70, 70, 75, 160))
                painter.setPen(QtGui.QPen(QtGui.QColor("#e0a000"), 1.5))
                painter.drawRect(QtCore.QRectF(p1, p2))
                continue

            if obj_type == "CYLINDER":
                radius = obj.get("radius", 0.15)
                p_edge = self.world_to_screen(ox + radius, oy)
                r_px = max(6.0, abs(p_edge.x() - pos.x()))

                if "バケツ" in comment:
                    painter.setBrush(QtGui.QColor("#20b2aa"))
                    painter.setPen(QtGui.QPen(QtGui.QColor("#00ffff"), 1.5))
                elif "旗" in comment:
                    painter.setBrush(QtGui.QColor("#ffd700"))
                    painter.setPen(QtGui.QPen(QtGui.QColor("#ffffff"), 1.5))
                else:
                    painter.setBrush(QtGui.QColor("#888888"))
                    painter.setPen(QtGui.QPen(QtGui.QColor("#aaaaaa"), 1))

                painter.drawEllipse(pos, r_px, r_px)

            elif obj_type == "BOX" and "外壁" not in comment and "避ける" not in comment:
                hx = obj.get("half_x", 0.2)
                hy = obj.get("half_y", 0.2)
                p1 = self.world_to_screen(ox - hx, oy + hy)
                p2 = self.world_to_screen(ox + hx, oy - hy)

                if "机" in comment:
                    painter.setBrush(QtGui.QColor(90, 60, 40, 180))
                    painter.setPen(QtGui.QPen(QtGui.QColor("#d2b48c"), 1.5))
                else:
                    painter.setBrush(QtGui.QColor(60, 60, 70, 160))
                    painter.setPen(QtGui.QPen(QtGui.QColor("#999999"), 1))

                painter.drawRect(QtCore.QRectF(p1, p2))

            # Draw concise label for primary target objects
            if any(k in comment for k in ["固定バケツ", "机1", "机2", "旗 布"]):
                painter.setFont(QtGui.QFont("Sans", 8))
                painter.setPen(QtGui.QColor("#ffffff"))
                # Shorten label
                short_lbl = comment.split("(")[0].strip()
                painter.drawText(
                    QtCore.QPointF(pos.x() + 8, pos.y() + 4), short_lbl
                )

    def draw_shooting_points(self, painter: QtGui.QPainter):
        """Draw each target's position, yaw heading arrow, and label."""
        for tid, pt in self.points.items():
            x = pt.get("x", 0.0)
            y = pt.get("y", 0.0)
            yaw = pt.get("yaw", 0.0)
            name = pt.get("name", f"Target_{tid}")
            v_tgt = pt.get("target_velocity", 0.0)

            pos = self.world_to_screen(x, y)
            is_selected = (tid == self.selected_target_id)

            # Colors: MovingBucket (7) cyan, StartZone (0) gray, Targets yellow/green
            if tid == 7:
                base_color = QtGui.QColor("#00e5ff")
            elif tid == 0:
                base_color = QtGui.QColor("#9e9e9e")
            else:
                base_color = QtGui.QColor("#76ff03") if not is_selected else QtGui.QColor("#ffea00")

            # Outer ring & circle
            radius = 16 if is_selected else 12
            painter.setBrush(base_color)
            painter.setPen(
                QtGui.QPen(QtGui.QColor("#ffffff") if is_selected else QtGui.QColor("#222222"), 2.5 if is_selected else 1.5)
            )
            painter.drawEllipse(pos, radius, radius)

            # Target ID number inside circle
            painter.setFont(QtGui.QFont("Sans", 9 if is_selected else 8, QtGui.QFont.Bold))
            painter.setPen(QtGui.QColor("#000000"))
            painter.drawText(
                QtCore.QRectF(pos.x() - radius, pos.y() - radius, radius * 2, radius * 2),
                QtCore.Qt.AlignCenter,
                str(tid),
            )

            # Yaw Heading Arrow (direction of shooting / robot heading)
            arrow_len = 32 if is_selected else 24
            # Screen coords: +X is right, +Y is up in world; screen sy = cy - y
            dx = arrow_len * math.cos(yaw)
            dy = -arrow_len * math.sin(yaw)
            arrow_tip = QtCore.QPointF(pos.x() + dx, pos.y() + dy)

            arrow_pen = QtGui.QPen(
                QtGui.QColor("#ffffff") if is_selected else base_color, 2.5 if is_selected else 1.8
            )
            painter.setPen(arrow_pen)
            painter.drawLine(pos, arrow_tip)

            # Arrow head
            head_size = 6
            angle = math.atan2(dy, dx)
            p_head1 = QtCore.QPointF(
                arrow_tip.x() - head_size * math.cos(angle - math.pi / 6),
                arrow_tip.y() - head_size * math.sin(angle - math.pi / 6),
            )
            p_head2 = QtCore.QPointF(
                arrow_tip.x() - head_size * math.cos(angle + math.pi / 6),
                arrow_tip.y() - head_size * math.sin(angle + math.pi / 6),
            )
            painter.drawLine(arrow_tip, p_head1)
            painter.drawLine(arrow_tip, p_head2)

            # Text tag
            painter.setFont(QtGui.QFont("Sans", 8, QtGui.QFont.Bold if is_selected else QtGui.QFont.Normal))
            painter.setPen(QtGui.QColor("#ffffff") if is_selected else QtGui.QColor("#cccccc"))
            info_str = f"[{tid}] {name}"
            if tid != 7 and v_tgt > 0.0:
                info_str += f" ({v_tgt:.1f}m/s)"
            painter.drawText(QtCore.QPointF(pos.x() + radius + 4, pos.y() + 4), info_str)

    # Mouse Events for placement & rotation
    def mousePressEvent(self, event: QtGui.QMouseEvent):
        if event.button() == QtCore.Qt.LeftButton:
            # Check if clicked near an existing target
            clicked_tid = self.find_target_near(event.pos())
            if clicked_tid is not None:
                self.selected_target_id = clicked_tid
                self.point_selected.emit(clicked_tid)
                self.dragging = True
            else:
                # Place selected target at clicked location
                if self.selected_target_id is not None:
                    wx, wy = self.screen_to_world(event.pos().x(), event.pos().y())
                    self.point_moved.emit(self.selected_target_id, wx, wy)
                    self.dragging = True
            self.update()

        elif event.button() == QtCore.Qt.RightButton:
            if self.selected_target_id is not None:
                self.rotating = True
                self.update_yaw_from_mouse(event.pos())

    def mouseMoveEvent(self, event: QtGui.QMouseEvent):
        if self.dragging and self.selected_target_id is not None:
            wx, wy = self.screen_to_world(event.pos().x(), event.pos().y())
            self.point_moved.emit(self.selected_target_id, wx, wy)
            self.update()
        elif self.rotating and self.selected_target_id is not None:
            self.update_yaw_from_mouse(event.pos())

    def mouseReleaseEvent(self, event: QtGui.QMouseEvent):
        self.dragging = False
        self.rotating = False

    def update_yaw_from_mouse(self, mouse_pos: QtCore.QPoint):
        if self.selected_target_id is None:
            return
        pt = self.points.get(self.selected_target_id)
        if not pt:
            return
        center_screen = self.world_to_screen(pt["x"], pt["y"])
        # Screen dx, dy -> world yaw
        dx = mouse_pos.x() - center_screen.x()
        dy = -(mouse_pos.y() - center_screen.y())
        yaw = math.atan2(dy, dx)
        self.yaw_changed.emit(self.selected_target_id, round(yaw, 4))
        self.update()

    def find_target_near(self, mouse_pos: QtCore.QPoint, threshold_px=22.0) -> Optional[int]:
        for tid, pt in self.points.items():
            screen_pos = self.world_to_screen(pt.get("x", 0.0), pt.get("y", 0.0))
            dist = math.hypot(mouse_pos.x() - screen_pos.x(), mouse_pos.y() - screen_pos.y())
            if dist <= threshold_px:
                return tid
        return None


class ShootingGuiApp(QtWidgets.QMainWindow):
    """Main application window for Shooting Point GUI Tuner."""

    def __init__(self, node: Node, pm: PointManager):
        super().__init__()
        self.node = node
        self.pm = pm
        self.dirty = False
        self.dirty_targets = set()
        self.is_saving = False
        self.save_worker = None

        self.target_id_pub = self.node.create_publisher(UInt8, "/robot/target_id", 10)

        filename = os.path.basename(self.pm.yaml_path)
        self.setWindowTitle(f"GN10 Shooting Point Field Editor - [{filename}]")
        self.resize(1120, 720)

        self.setup_ui()
        self.refresh_table_and_canvas()

        # ROS 2 periodic spin timer
        self.ros_timer = QtCore.QTimer(self)
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(25)  # 40Hz

    def setup_ui(self):
        main_widget = QtWidgets.QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QtWidgets.QHBoxLayout(main_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(12)

        # 1. Left side: Field 2D Canvas
        field_group = QtWidgets.QGroupBox("Field View (Click / Drag to Place & Orient)")
        field_layout = QtWidgets.QVBoxLayout(field_group)
        self.canvas = FieldCanvas(self)
        self.canvas.point_moved.connect(self.on_canvas_point_moved)
        self.canvas.yaw_changed.connect(self.on_canvas_yaw_changed)
        self.canvas.point_selected.connect(self.on_canvas_point_selected)
        field_layout.addWidget(self.canvas)
        main_layout.addWidget(field_group, stretch=65)

        # 2. Right side: Table, Controls, and Actions
        right_panel = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(8)

        # Target Table
        self.table = QtWidgets.QTableWidget()
        self.table.setColumnCount(6)
        self.table.setHorizontalHeaderLabels(["ID", "Name", "X [m]", "Y [m]", "Yaw [°]", "V [m/s]"])
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.table.itemSelectionChanged.connect(self.on_table_selection_changed)
        right_layout.addWidget(self.table, stretch=40)

        # Fine Adjustment Group
        adj_group = QtWidgets.QGroupBox("Micro Adjustment (Fine Tuning)")
        adj_layout = QtWidgets.QFormLayout(adj_group)
        adj_layout.setSpacing(6)

        # X input & step buttons
        self.spin_x = QtWidgets.QDoubleSpinBox()
        self.spin_x.setRange(-6.0, 6.0)
        self.spin_x.setSingleStep(0.005)
        self.spin_x.setDecimals(4)
        self.spin_x.valueChanged.connect(self.on_spin_x_changed)
        box_x = QtWidgets.QHBoxLayout()
        box_x.addWidget(self.spin_x)
        btn_x_minus = QtWidgets.QPushButton("-5cm")
        btn_x_minus.clicked.connect(lambda: self.adjust_delta("x", -0.05))
        btn_x_plus = QtWidgets.QPushButton("+5cm")
        btn_x_plus.clicked.connect(lambda: self.adjust_delta("x", 0.05))
        box_x.addWidget(btn_x_minus)
        box_x.addWidget(btn_x_plus)
        adj_layout.addRow("X [m]:", box_x)

        # Y input & step buttons
        self.spin_y = QtWidgets.QDoubleSpinBox()
        self.spin_y.setRange(-6.0, 6.0)
        self.spin_y.setSingleStep(0.005)
        self.spin_y.setDecimals(4)
        self.spin_y.valueChanged.connect(self.on_spin_y_changed)
        box_y = QtWidgets.QHBoxLayout()
        box_y.addWidget(self.spin_y)
        btn_y_minus = QtWidgets.QPushButton("-5cm")
        btn_y_minus.clicked.connect(lambda: self.adjust_delta("y", -0.05))
        btn_y_plus = QtWidgets.QPushButton("+5cm")
        btn_y_plus.clicked.connect(lambda: self.adjust_delta("y", 0.05))
        box_y.addWidget(btn_y_minus)
        box_y.addWidget(btn_y_plus)
        adj_layout.addRow("Y [m]:", box_y)

        # Yaw input & step buttons
        self.spin_yaw = QtWidgets.QDoubleSpinBox()
        self.spin_yaw.setRange(-180.0, 180.0)
        self.spin_yaw.setSingleStep(1.0)
        self.spin_yaw.setDecimals(1)
        self.spin_yaw.valueChanged.connect(self.on_spin_yaw_deg_changed)
        box_yaw = QtWidgets.QHBoxLayout()
        box_yaw.addWidget(self.spin_yaw)
        btn_yaw_minus = QtWidgets.QPushButton("-5°")
        btn_yaw_minus.clicked.connect(lambda: self.adjust_delta("yaw_deg", -5.0))
        btn_yaw_plus = QtWidgets.QPushButton("+5°")
        btn_yaw_plus.clicked.connect(lambda: self.adjust_delta("yaw_deg", 5.0))
        box_yaw.addWidget(btn_yaw_minus)
        box_yaw.addWidget(btn_yaw_plus)
        adj_layout.addRow("Yaw [deg]:", box_yaw)

        # Target Velocity
        self.spin_v = QtWidgets.QDoubleSpinBox()
        self.spin_v.setRange(0.0, 15.0)
        self.spin_v.setSingleStep(0.1)
        self.spin_v.setDecimals(2)
        self.spin_v.valueChanged.connect(self.on_spin_v_changed)
        box_v = QtWidgets.QHBoxLayout()
        box_v.addWidget(self.spin_v)
        btn_v_minus = QtWidgets.QPushButton("-0.5")
        btn_v_minus.clicked.connect(lambda: self.adjust_delta("v", -0.5))
        btn_v_plus = QtWidgets.QPushButton("+0.5")
        btn_v_plus.clicked.connect(lambda: self.adjust_delta("v", 0.5))
        box_v.addWidget(btn_v_minus)
        box_v.addWidget(btn_v_plus)
        adj_layout.addRow("Target Vel [m/s]:", box_v)

        # Memo
        self.edit_memo = QtWidgets.QLineEdit()
        self.edit_memo.textChanged.connect(self.on_memo_changed)
        adj_layout.addRow("Memo:", self.edit_memo)

        right_layout.addWidget(adj_group, stretch=35)

        # Action Buttons
        btn_layout = QtWidgets.QHBoxLayout()
        self.btn_save = QtWidgets.QPushButton("💾 Save & Commit")
        self.btn_save.setStyleSheet("background-color: #2e7d32; color: white; font-weight: bold; padding: 6px;")
        self.btn_save.clicked.connect(self.save_and_commit)
        btn_layout.addWidget(self.btn_save)

        self.btn_reload = QtWidgets.QPushButton("🔄 Reload YAML")
        self.btn_reload.clicked.connect(self.reload_yaml)
        btn_layout.addWidget(self.btn_reload)

        self.btn_test = QtWidgets.QPushButton("🎯 Test Target")
        self.btn_test.clicked.connect(self.publish_test_target)
        btn_layout.addWidget(self.btn_test)

        right_layout.addLayout(btn_layout)
        main_layout.addWidget(right_panel, stretch=35)

        # Status Bar
        self.statusBar().showMessage("Ready.")

    def spin_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def refresh_table_and_canvas(self):
        points = self.pm.points
        selected_id = self.canvas.selected_target_id
        if selected_id not in points and points:
            selected_id = sorted(points.keys())[0]

        self.table.blockSignals(True)
        self.table.setRowCount(len(points))

        row_to_select = 0
        for row, tid in enumerate(sorted(points.keys())):
            pt = points[tid]
            deg = math.degrees(pt.get("yaw", 0.0))

            self.table.setItem(row, 0, QtWidgets.QTableWidgetItem(str(tid)))
            self.table.setItem(row, 1, QtWidgets.QTableWidgetItem(str(pt.get("name", ""))))
            self.table.setItem(row, 2, QtWidgets.QTableWidgetItem(f"{pt.get('x', 0.0):.3f}"))
            self.table.setItem(row, 3, QtWidgets.QTableWidgetItem(f"{pt.get('y', 0.0):.3f}"))
            self.table.setItem(row, 4, QtWidgets.QTableWidgetItem(f"{deg:.1f}"))
            self.table.setItem(row, 5, QtWidgets.QTableWidgetItem(f"{pt.get('target_velocity', 0.0):.2f}"))

            if tid == selected_id:
                row_to_select = row

        self.table.selectRow(row_to_select)
        self.table.blockSignals(False)

        self.canvas.set_points(points, selected_id)
        self.update_inputs_from_selected()

    def get_selected_target_id(self) -> Optional[int]:
        rows = self.table.selectionModel().selectedRows()
        if not rows:
            return None
        tid_item = self.table.item(rows[0].row(), 0)
        return int(tid_item.text()) if tid_item else None

    def update_inputs_from_selected(self):
        tid = self.get_selected_target_id()
        if tid is None or tid not in self.pm.points:
            return
        pt = self.pm.points[tid]

        self.spin_x.blockSignals(True)
        self.spin_y.blockSignals(True)
        self.spin_yaw.blockSignals(True)
        self.spin_v.blockSignals(True)
        self.edit_memo.blockSignals(True)

        self.spin_x.setValue(pt.get("x", 0.0))
        self.spin_y.setValue(pt.get("y", 0.0))
        self.spin_yaw.setValue(round(math.degrees(pt.get("yaw", 0.0)), 1))
        self.spin_v.setValue(pt.get("target_velocity", 0.0))
        self.edit_memo.setText(pt.get("memo", ""))

        self.spin_x.blockSignals(False)
        self.spin_y.blockSignals(False)
        self.spin_yaw.blockSignals(False)
        self.spin_v.blockSignals(False)
        self.edit_memo.blockSignals(False)

    def on_table_selection_changed(self):
        tid = self.get_selected_target_id()
        if tid is not None:
            self.canvas.set_selected_id(tid)
            self.update_inputs_from_selected()

    def on_canvas_point_selected(self, tid: int):
        for r in range(self.table.rowCount()):
            if int(self.table.item(r, 0).text()) == tid:
                self.table.selectRow(r)
                break

    def on_canvas_point_moved(self, tid: int, x: float, y: float):
        if tid in self.pm.points:
            self.pm.points[tid]["x"] = x
            self.pm.points[tid]["y"] = y
            self.dirty = True
            self.dirty_targets.add(tid)
            self.refresh_table_and_canvas()
            self.statusBar().showMessage(f"Target {tid} moved to ({x:.3f}, {y:.3f}) [Unsaved]")

    def on_canvas_yaw_changed(self, tid: int, yaw: float):
        if tid in self.pm.points:
            self.pm.points[tid]["yaw"] = yaw
            self.dirty = True
            self.dirty_targets.add(tid)
            self.refresh_table_and_canvas()
            deg = math.degrees(yaw)
            self.statusBar().showMessage(f"Target {tid} yaw set to {deg:.1f}° [Unsaved]")

    def on_spin_x_changed(self, val: float):
        tid = self.get_selected_target_id()
        if tid is not None and tid in self.pm.points:
            self.pm.points[tid]["x"] = round(val, 4)
            self.dirty = True
            self.dirty_targets.add(tid)
            self.canvas.update()
            self.update_table_cell(tid, 2, f"{val:.3f}")

    def on_spin_y_changed(self, val: float):
        tid = self.get_selected_target_id()
        if tid is not None and tid in self.pm.points:
            self.pm.points[tid]["y"] = round(val, 4)
            self.dirty = True
            self.dirty_targets.add(tid)
            self.canvas.update()
            self.update_table_cell(tid, 3, f"{val:.3f}")

    def on_spin_yaw_deg_changed(self, deg_val: float):
        tid = self.get_selected_target_id()
        if tid is not None and tid in self.pm.points:
            yaw = round(math.radians(deg_val), 4)
            self.pm.points[tid]["yaw"] = yaw
            self.dirty = True
            self.dirty_targets.add(tid)
            self.canvas.update()
            self.update_table_cell(tid, 4, f"{deg_val:.1f}")

    def on_spin_v_changed(self, val: float):
        tid = self.get_selected_target_id()
        if tid is not None and tid in self.pm.points:
            self.pm.points[tid]["target_velocity"] = round(val, 3)
            self.dirty = True
            self.dirty_targets.add(tid)
            self.canvas.update()
            self.update_table_cell(tid, 5, f"{val:.2f}")

    def on_memo_changed(self, text: str):
        tid = self.get_selected_target_id()
        if tid is not None and tid in self.pm.points:
            self.pm.points[tid]["memo"] = text.strip()
            self.dirty = True
            self.dirty_targets.add(tid)

    def adjust_delta(self, param: str, delta: float):
        if param == "x":
            self.spin_x.setValue(self.spin_x.value() + delta)
        elif param == "y":
            self.spin_y.setValue(self.spin_y.value() + delta)
        elif param == "yaw_deg":
            val = self.spin_yaw.value() + delta
            while val > 180.0:
                val -= 360.0
            while val < -180.0:
                val += 360.0
            self.spin_yaw.setValue(val)
        elif param == "v":
            self.spin_v.setValue(max(0.0, self.spin_v.value() + delta))

    def update_table_cell(self, tid: int, col: int, text: str):
        for r in range(self.table.rowCount()):
            if int(self.table.item(r, 0).text()) == tid:
                self.table.item(r, col).setText(text)
                break

    def save_and_commit(self):
        if self.is_saving:
            return

        self.is_saving = True
        self.btn_save.setEnabled(False)
        self.btn_save.setText("💾 Saving...")
        self.statusBar().showMessage("Saving shooting points and committing to Git...")

        # Sync dirty targets (or all if none explicitly recorded)
        target_ids = list(self.dirty_targets) if self.dirty_targets else list(self.pm.points.keys())

        tid = self.get_selected_target_id() or (target_ids[0] if target_ids else 0)
        p = self.pm.points.get(tid, {})
        default_msg = (
            f"tune(gui_target_{tid}): x={p.get('x',0.0):.3f}, "
            f"y={p.get('y',0.0):.3f}, yaw={p.get('yaw',0.0):.3f}, "
            f"v={p.get('target_velocity',0.0):.2f}"
        )

        self.save_worker = SaveWorker(self.pm, target_ids, default_msg)
        self.save_worker.finished.connect(self.on_save_finished)
        self.save_worker.start()

    def on_save_finished(self, success: bool, message: str):
        self.is_saving = False
        self.btn_save.setEnabled(True)
        self.btn_save.setText("💾 Save & Commit")

        if success:
            self.dirty = False
            self.dirty_targets.clear()
            self.statusBar().showMessage(f"✅ {message}", 5000)
        else:
            self.statusBar().showMessage(f"⚠️ {message}", 5000)

    def reload_yaml(self):
        if self.pm.load_yaml():
            self.dirty = False
            self.dirty_targets.clear()
            self.refresh_table_and_canvas()
            filename = os.path.basename(self.pm.yaml_path)
            self.statusBar().showMessage(f"Reloaded {filename} successfully.")
        else:
            self.statusBar().showMessage("Failed to reload YAML.")

    def publish_test_target(self):
        tid = self.get_selected_target_id()
        if tid is not None:
            msg = UInt8()
            msg.data = int(tid)
            self.target_id_pub.publish(msg)
            p = self.pm.points.get(tid, {})
            self.statusBar().showMessage(f"Published /robot/target_id = {tid} ({p.get('name', '')})")


def main(args=None):
    rclpy.init(args=args)
    node = Node("shooting_gui_node")
    pm = PointManager(node=node)

    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")

    # Dark palette for clean robotics UI
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

    window = ShootingGuiApp(node, pm)
    window.show()

    try:
        sys.exit(app.exec_())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
