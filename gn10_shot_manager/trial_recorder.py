"""Trial Recorder: Synchronous recording of MCU sensor feedback, robot pose on map,
target estimation, and operator confirmation per shot.
"""

import collections
import datetime
import json
import math
import os
import subprocess
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32, UInt8
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
import tf2_ros

from gn10_shot_manager.point_manager import PointManager


# Standard target definitions and field object locations in map frame
TARGET_NAMES: Dict[int, str] = {
    0: "StartZone",
    1: "FixedBucket1",
    2: "FixedBucket2",
    3: "FixedBucket3",
    4: "Desk1",
    5: "Desk2",
    6: "Flag",
    7: "MovingBucket",
}

# Field target coordinates in map frame (x, y_abs, opening_z, typical_v_min, typical_v_max)
# For blue zone (robot y > 0), opponent targets are at y = -y_abs.
# For red zone (robot y < 0), opponent targets are at y = +y_abs.
FIELD_TARGETS_MAP = {
    1: {"name": "FixedBucket1", "x": 0.55, "y_abs": 0.87, "z": 0.255, "v_ref": 3.0},
    2: {"name": "FixedBucket2", "x": -1.27, "y_abs": 1.48, "z": 0.855, "v_ref": 3.25},
    3: {"name": "FixedBucket3", "x": 2.37, "y_abs": 1.48, "z": 0.555, "v_ref": 3.25},
    6: {"name": "Flag", "x": 0.70, "y_abs": 3.025, "z": 3.000, "v_ref": 7.0},
}


def normalize_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q) -> float:
    """Extract yaw [rad] from geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class TrialRecorderNode(Node):
    """Subscribes to MCU feedback, TF (map->base_link), and cloth tracking; records trial window on shot trigger."""

    def __init__(self):
        super().__init__("trial_recorder_node")

        # Declare parameters
        self.declare_parameter("record_duration_sec", 3.5)
        self.declare_parameter("pre_trigger_buffer_sec", 0.5)
        self.declare_parameter("output_dir", os.path.expanduser("~/ros2_ws/data/shot-trial-log"))
        self.declare_parameter("log_filename", "trials.jsonl")
        self.declare_parameter("auto_git_commit", True)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("confirm_each_shot", True)
        self.declare_parameter("default_cloth_size", "large")

        self.record_duration = float(self.get_parameter("record_duration_sec").value)
        self.pre_buffer_duration = float(self.get_parameter("pre_trigger_buffer_sec").value)
        self.output_dir = str(self.get_parameter("output_dir").value)
        self.log_filename = str(self.get_parameter("log_filename").value)
        self.auto_git_commit = bool(self.get_parameter("auto_git_commit").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.confirm_each_shot = bool(self.get_parameter("confirm_each_shot").value)
        self.last_cloth_size = str(self.get_parameter("default_cloth_size").value).lower()
        if self.last_cloth_size not in ("large", "small"):
            self.last_cloth_size = "large"

        os.makedirs(self.output_dir, exist_ok=True)
        self.output_path = os.path.join(self.output_dir, self.log_filename)
        self._file_lock = threading.Lock()
        self._prompt_lock = threading.Lock()

        # Load target points for coordinate reference
        self.pm = PointManager(node=self)

        # TF buffer & listener for map -> base_link
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Session tracking for summary commit on shutdown
        self.session_start_time = datetime.datetime.now()
        self.session_shots = []

        # Sensor state
        self.current_target_id = 0
        self.last_shot_count = None
        self.is_recording = False
        self.record_start_time = 0.0
        self.active_shot_id = 0
        self.last_cloth_msg_time = 0.0
        self.trigger_robot_pose = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "valid": False}

        # Latest readings
        self.latest_data = {
            "battery_voltage": 0.0,
            "drive_current": 0.0,
            "launcher_velocity": 0.0,
            "launcher_command": 0.0,
            "launcher_target": 0.0,
            "launcher_voltage": 0.0,
            "launcher_current": 0.0,
            "cloth_x": None,
            "cloth_y": None,
            "cloth_z": None,
            "robot_x": 0.0,
            "robot_y": 0.0,
            "robot_z": 0.0,
            "robot_yaw": 0.0,
            "robot_pose_valid": False,
        }

        # Circular buffer for pre-trigger data
        self.buffer = collections.deque(maxlen=200)

        # Active recording sample list
        self.active_samples = []
        self.cloth_path_points = []

        # Subscriptions to robot-port feedback
        self.create_subscription(UInt8, "/robot/feedback/shot_count", self.on_shot_count, 10)
        self.create_subscription(UInt8, "/robot/target_id", self.on_target_id, 10)

        self.create_subscription(Float32, "/robot/feedback/drive_battery_voltage", lambda m: self.update_val("battery_voltage", m.data), 10)
        self.create_subscription(Float32, "/robot/feedback/drive_current", lambda m: self.update_val("drive_current", m.data), 10)
        self.create_subscription(Float32, "/robot/feedback/belt_launcher/velocity", lambda m: self.update_val("launcher_velocity", m.data), 10)
        self.create_subscription(Float32, "/robot/feedback/belt_launcher/command", lambda m: self.update_val("launcher_command", m.data), 10)
        self.create_subscription(Float32, "/robot/feedback/belt_launcher/target", lambda m: self.update_val("launcher_target", m.data), 10)
        self.create_subscription(Float32, "/robot/feedback/belt_launcher/voltage", lambda m: self.update_val("launcher_voltage", m.data), 10)
        self.create_subscription(Float32, "/robot/feedback/belt_launcher/current", lambda m: self.update_val("launcher_current", m.data), 10)

        # Subscriptions to realsense_dust_cloth (published in base_link frame)
        self.create_subscription(PointStamped, "/cloth/position", self.on_cloth_position, 10)
        self.create_subscription(Path, "/cloth/measured_path", self.on_cloth_path, 10)

        # 50Hz periodic sampling timer
        self.sample_timer = self.create_timer(0.02, self.on_sample_timer)

        self.get_logger().info(f"TrialRecorderNode initialized. Logging to: {self.output_path}")

    def update_val(self, key: str, val: float):
        self.latest_data[key] = float(val)

    def on_target_id(self, msg: UInt8):
        self.current_target_id = int(msg.data)

    def on_cloth_position(self, msg: PointStamped):
        self.latest_data["cloth_x"] = float(msg.point.x)
        self.latest_data["cloth_y"] = float(msg.point.y)
        self.latest_data["cloth_z"] = float(msg.point.z)
        self.last_cloth_msg_time = time.time()

    def on_cloth_path(self, msg: Path):
        if self.is_recording:
            self.cloth_path_points = [
                {"x": float(p.pose.position.x), "y": float(p.pose.position.y), "z": float(p.pose.position.z)}
                for p in msg.poses
            ]

    def update_robot_pose_from_tf(self):
        """Look up latest map -> base_link transform."""
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
            )
            t = tf_msg.transform.translation
            q = tf_msg.transform.rotation
            self.latest_data["robot_x"] = round(float(t.x), 4)
            self.latest_data["robot_y"] = round(float(t.y), 4)
            self.latest_data["robot_z"] = round(float(t.z), 4)
            self.latest_data["robot_yaw"] = round(yaw_from_quaternion(q), 4)
            self.latest_data["robot_pose_valid"] = True
        except Exception:
            # Keep last coordinates if any, but mark invalid if never received
            pass

    def on_shot_count(self, msg: UInt8):
        count = int(msg.data)
        if self.last_shot_count is None:
            self.last_shot_count = count
            self.get_logger().info(f"Initial shot_count synchronized: {count}")
            return

        # 0に変化した際（原点取り等）は無視し、記録トリガは作動させない
        if count == 0:
            if self.last_shot_count != 0:
                self.get_logger().info("Shot count reset to 0 (origin homing). Ignoring trigger.")
            self.last_shot_count = 0
            return

        # インクリメント時に射出命令として記録開始
        is_increment = (count > self.last_shot_count) or (self.last_shot_count == 255 and count > 0)
        if is_increment:
            self.get_logger().info(f"Trigger! Shot count incremented: {self.last_shot_count} -> {count}")
            self.last_shot_count = count
            self.start_recording(shot_id=count)
        else:
            self.last_shot_count = count

    def start_recording(self, shot_id: int):
        self.is_recording = True
        self.active_shot_id = shot_id
        self.record_start_time = time.time()
        self.cloth_path_points = []

        # Clear stale cloth coordinates from previous shot
        self.latest_data["cloth_x"] = None
        self.latest_data["cloth_y"] = None
        self.latest_data["cloth_z"] = None

        # Snapshot robot pose at shot trigger
        self.update_robot_pose_from_tf()
        self.trigger_robot_pose = {
            "x": self.latest_data["robot_x"],
            "y": self.latest_data["robot_y"],
            "z": self.latest_data["robot_z"],
            "yaw": self.latest_data["robot_yaw"],
            "valid": self.latest_data["robot_pose_valid"],
        }

        # Pull pre-trigger buffer samples (clearing stale cloth coordinates before trigger)
        cutoff = self.record_start_time - self.pre_buffer_duration
        self.active_samples = []
        for s in self.buffer:
            if s["abs_time"] >= cutoff:
                s_copy = dict(s)
                s_copy["rel_time"] = round(s_copy["abs_time"] - self.record_start_time, 3)
                s_copy["cloth_x"] = None
                s_copy["cloth_y"] = None
                s_copy["cloth_z"] = None
                self.active_samples.append(s_copy)

    def on_sample_timer(self):
        now = time.time()
        self.update_robot_pose_from_tf()

        # Expire cloth position if no new PointStamped arrived within 0.15s
        if self.last_cloth_msg_time > 0.0 and (now - self.last_cloth_msg_time) > 0.15:
            self.latest_data["cloth_x"] = None
            self.latest_data["cloth_y"] = None
            self.latest_data["cloth_z"] = None

        sample = {
            "abs_time": now,
            "rel_time": round(now - self.record_start_time, 3) if self.is_recording else 0.0,
            "battery_voltage": self.latest_data["battery_voltage"],
            "drive_current": self.latest_data["drive_current"],
            "launcher_velocity": self.latest_data["launcher_velocity"],
            "launcher_command": self.latest_data["launcher_command"],
            "launcher_target": self.latest_data["launcher_target"],
            "launcher_voltage": self.latest_data["launcher_voltage"],
            "launcher_current": self.latest_data["launcher_current"],
            "cloth_x": self.latest_data["cloth_x"],
            "cloth_y": self.latest_data["cloth_y"],
            "cloth_z": self.latest_data["cloth_z"],
            "robot_x": self.latest_data["robot_x"],
            "robot_y": self.latest_data["robot_y"],
            "robot_yaw": self.latest_data["robot_yaw"],
        }
        self.buffer.append(sample)

        if self.is_recording:
            self.active_samples.append(sample)
            if now - self.record_start_time >= self.record_duration:
                self.finish_recording()

    def estimate_target(
        self,
        robot_pose: dict,
        release_v_tgt: float,
        current_target_id: int,
    ) -> Tuple[int, str, str]:
        """Estimate which target was aimed at from map base_link pose and target velocity."""
        # 1. High launch velocity (>= 5.8 m/s) is exclusively used for the Flag (height ~3m)
        if release_v_tgt >= 5.8:
            return 6, TARGET_NAMES[6], f"release_v_tgt={release_v_tgt:.2f} m/s (Flag range >= 5.8 m/s)"

        # 2. If robot pose on map is valid, match against field targets & shooting points
        if robot_pose.get("valid", False):
            rx = float(robot_pose.get("x", 0.0))
            ry = float(robot_pose.get("y", 0.0))
            ryaw = float(robot_pose.get("yaw", 0.0))

            # Determine opponent zone sign: if robot is in Blue (y >= 0), opponent targets are at y < 0
            opp_y_sign = -1.0 if ry >= 0.0 else 1.0

            # Check configured shooting points proximity first (if within 0.6m and 25 deg and matching velocity)
            best_sp_id = None
            best_sp_score = 1e9
            for tid, pt in self.pm.points.items():
                if tid == 0:
                    continue
                dist_sp = math.hypot(rx - float(pt.get("x", 0.0)), ry - float(pt.get("y", 0.0)))
                yaw_err = abs(normalize_angle(ryaw - float(pt.get("yaw", 0.0))))
                v_diff = abs(release_v_tgt - float(pt.get("target_velocity", 0.0)))
                if dist_sp < 0.65 and yaw_err < math.radians(25.0):
                    score = dist_sp + 0.5 * yaw_err + 0.3 * v_diff
                    if score < best_sp_score:
                        best_sp_score = score
                        best_sp_id = tid

            if best_sp_id is not None:
                name = self.pm.points[best_sp_id].get("name", TARGET_NAMES.get(best_sp_id, f"Target_{best_sp_id}"))
                return (
                    best_sp_id,
                    name,
                    f"matched shooting_point #{best_sp_id} (pose dist={best_sp_score:.2f}, v_tgt={release_v_tgt:.2f} m/s)",
                )

            # Ray-cast / line-of-sight check toward field targets (FixedBucket 1/2/3, Flag)
            best_ft_id = None
            best_ft_score = 1e9
            for tid, ft in FIELD_TARGETS_MAP.items():
                tx = ft["x"]
                ty = opp_y_sign * ft["y_abs"]
                dx = tx - rx
                dy = ty - ry
                dist_to_target = math.hypot(dx, dy)
                bearing = math.atan2(dy, dx)
                heading_err = abs(normalize_angle(bearing - ryaw))
                v_err = abs(release_v_tgt - ft["v_ref"])

                # Check if robot is facing toward this target within 30 deg
                if heading_err <= math.radians(30.0) and 0.6 <= dist_to_target <= 6.0:
                    score = heading_err * 2.0 + 0.25 * v_err + 0.05 * abs(dist_to_target - 3.2)
                    if score < best_ft_score:
                        best_ft_score = score
                        best_ft_id = tid

            if best_ft_id is not None:
                return (
                    best_ft_id,
                    TARGET_NAMES[best_ft_id],
                    f"line-of-sight from base_link ({rx:.2f}, {ry:.2f}, yaw={math.degrees(ryaw):.1f}deg) & v_tgt={release_v_tgt:.2f} m/s",
                )

            # Otherwise, if medium velocity in field, likely aiming at MovingBucket (7)
            if 2.3 <= release_v_tgt < 5.8:
                return (
                    7,
                    TARGET_NAMES[7],
                    f"manual/dynamic pose ({rx:.2f}, {ry:.2f}) not aligned with fixed buckets, v_tgt={release_v_tgt:.2f} m/s",
                )

        # 3. Fallback when TF is unavailable: use current_target_id if non-zero, else velocity heuristic
        if current_target_id in TARGET_NAMES and current_target_id != 0:
            return (
                current_target_id,
                TARGET_NAMES[current_target_id],
                f"from /robot/target_id={current_target_id} (v_tgt={release_v_tgt:.2f} m/s)",
            )

        if abs(release_v_tgt - 3.0) <= 0.12:
            return 1, TARGET_NAMES[1], f"velocity heuristic v_tgt={release_v_tgt:.2f} m/s (~3.0 m/s)"
        if abs(release_v_tgt - 3.25) <= 0.15:
            return 2, TARGET_NAMES[2], f"velocity heuristic v_tgt={release_v_tgt:.2f} m/s (~3.25 m/s)"
        return 7, TARGET_NAMES[7], f"velocity heuristic v_tgt={release_v_tgt:.2f} m/s"

    def finish_recording(self):
        self.is_recording = False
        duration = time.time() - self.record_start_time
        self.get_logger().info(
            f"Finished recording shot {self.active_shot_id} ({duration:.2f}s, {len(self.active_samples)} samples)"
        )

        # Determine release target velocity from post-trigger samples (t >= 0.0)
        post_trigger_v_tgts = [
            s["launcher_target"] for s in self.active_samples if s["rel_time"] >= 0.0 and s["launcher_target"] > 0.1
        ]
        if post_trigger_v_tgts:
            # Mode or max of target velocity during the shot
            release_v_tgt = round(float(max(post_trigger_v_tgts)), 2)
        else:
            all_v_tgts = [s["launcher_target"] for s in self.active_samples if s["launcher_target"] > 0.1]
            release_v_tgt = round(float(max(all_v_tgts)) if all_v_tgts else 0.0, 2)

        # Estimate target from base_link pose on map and release target velocity
        est_id, est_name, est_reason = self.estimate_target(
            self.trigger_robot_pose, release_v_tgt, self.current_target_id
        )

        target_pt = dict(
            self.pm.points.get(
                est_id,
                {
                    "target_id": est_id,
                    "name": est_name,
                    "x": self.trigger_robot_pose.get("x", 0.0),
                    "y": self.trigger_robot_pose.get("y", 0.0),
                    "yaw": self.trigger_robot_pose.get("yaw", 0.0),
                    "target_velocity": release_v_tgt,
                    "memo": "",
                },
            )
        )

        # Landing position in base_link frame (from cloth samples observed during this shot)
        landing_pos_base = None
        if self.cloth_path_points:
            landing_pos_base = self.cloth_path_points[-1]
        else:
            for s in reversed(self.active_samples):
                if s["rel_time"] >= 0.05 and s["cloth_x"] is not None:
                    landing_pos_base = {"x": s["cloth_x"], "y": s["cloth_y"], "z": s["cloth_z"]}
                    break

        # Transform landing position to map frame if robot_pose is valid
        landing_pos_map = None
        if landing_pos_base and self.trigger_robot_pose.get("valid", False):
            rx = self.trigger_robot_pose["x"]
            ry = self.trigger_robot_pose["y"]
            rz = self.trigger_robot_pose.get("z", 0.0)
            ryaw = self.trigger_robot_pose["yaw"]
            cx = landing_pos_base["x"]
            cy = landing_pos_base["y"]
            cz = landing_pos_base["z"]
            landing_pos_map = {
                "x": round(rx + cx * math.cos(ryaw) - cy * math.sin(ryaw), 4),
                "y": round(ry + cx * math.sin(ryaw) + cy * math.cos(ryaw), 4),
                "z": round(rz + cz, 4),
            }

        # Calculate statistics
        v_list = [s["launcher_velocity"] for s in self.active_samples if s["rel_time"] >= 0.0]
        c_list = [s["launcher_current"] for s in self.active_samples if s["rel_time"] >= 0.0]
        volt_list = [s["launcher_voltage"] for s in self.active_samples if s["rel_time"] >= 0.0]

        record = {
            "shot_id": self.active_shot_id,
            "timestamp": datetime.datetime.now().isoformat(),
            "coordinate_frame": self.base_frame,
            "robot_pose": self.trigger_robot_pose,
            "reported_target_id": self.current_target_id,
            "estimated_target": {
                "target_id": est_id,
                "name": est_name,
                "reason": est_reason,
            },
            "target": target_pt,
            "cloth_size": self.last_cloth_size,
            "result": "unconfirmed",
            "success": None,
            "target_confirmed": False,
            "summary": {
                "release_target_velocity": release_v_tgt,
                "max_velocity": round(max(v_list) if v_list else 0.0, 2),
                "max_current": round(max(c_list) if c_list else 0.0, 2),
                "avg_voltage": round(sum(volt_list) / len(volt_list) if volt_list else 0.0, 2),
            },
            "landing_position": landing_pos_base,
            "landing_position_map": landing_pos_map,
            "trajectory_points_count": sum(1 for s in self.active_samples if s["cloth_x"] is not None),
            "samples": [
                {
                    "t": s["rel_time"],
                    "v_act": round(s["launcher_velocity"], 2),
                    "v_cmd": round(s.get("launcher_command", 0.0), 2),
                    "v_tgt": round(s["launcher_target"], 2),
                    "curr": round(s["launcher_current"], 2),
                    "volt": round(s["launcher_voltage"], 2),
                    "cx": s["cloth_x"],
                    "cy": s["cloth_y"],
                    "cz": s["cloth_z"],
                    "rx": s["robot_x"],
                    "ry": s["robot_y"],
                    "ryaw": s["robot_yaw"],
                }
                for s in self.active_samples
            ],
        }

        # Write initial record immediately so data is safe on disk
        line_index = self._append_record_to_file(record)
        self.get_logger().info(
            f"Trial #{self.active_shot_id} recorded -> Estimated Target: {est_id} ({est_name}) [{est_reason}]"
        )

        self.session_shots.append({
            "shot_id": self.active_shot_id,
            "target_name": est_name,
            "timestamp": datetime.datetime.now(),
        })

        # Launch non-blocking confirmation prompt for operator verification
        if self.confirm_each_shot and line_index is not None:
            threading.Thread(
                target=self._confirm_shot_worker,
                args=(line_index, record),
                daemon=True,
            ).start()

    def _append_record_to_file(self, record: dict) -> Optional[int]:
        """Append record to JSONL and return its 0-based line index."""
        try:
            with self._file_lock:
                existing_lines = []
                if os.path.exists(self.output_path):
                    with open(self.output_path, "r", encoding="utf-8") as f:
                        existing_lines = f.readlines()
                line_idx = len(existing_lines)
                with open(self.output_path, "a", encoding="utf-8") as f:
                    f.write(json.dumps(record, ensure_ascii=False) + "\n")
                return line_idx
        except Exception as e:
            self.get_logger().error(f"Failed to write trial record: {e}")
            return None

    def _update_record_in_file(self, line_index: int, updated_record: dict):
        """Update a specific line in trials.jsonl after operator confirmation."""
        try:
            with self._file_lock:
                if not os.path.exists(self.output_path):
                    return
                with open(self.output_path, "r", encoding="utf-8") as f:
                    lines = f.readlines()
                if 0 <= line_index < len(lines):
                    lines[line_index] = json.dumps(updated_record, ensure_ascii=False) + "\n"
                    with open(self.output_path, "w", encoding="utf-8") as f:
                        f.writelines(lines)
        except Exception as e:
            self.get_logger().error(f"Failed to update confirmed trial record: {e}")

    def _confirm_shot_worker(self, line_index: int, record: dict):
        """Prompt operator via GUI dialog (or CLI fallback) to confirm target, cloth size, and success/failure."""
        with self._prompt_lock:
            est_id = int(record["estimated_target"]["target_id"])
            est_name = record["estimated_target"]["name"]
            est_reason = record["estimated_target"]["reason"]
            v_tgt = record["summary"]["release_target_velocity"]
            rpose = record["robot_pose"]

            confirmed = None
            if os.environ.get("DISPLAY"):
                confirmed = self._run_gui_confirmation(
                    shot_id=record["shot_id"],
                    est_id=est_id,
                    est_name=est_name,
                    est_reason=est_reason,
                    v_tgt=v_tgt,
                    rpose=rpose,
                    default_cloth_size=self.last_cloth_size,
                )

            if confirmed is None and sys.stdin is not None and sys.stdin.isatty():
                confirmed = self._run_cli_confirmation(
                    shot_id=record["shot_id"],
                    est_id=est_id,
                    est_name=est_name,
                    est_reason=est_reason,
                    v_tgt=v_tgt,
                    rpose=rpose,
                    default_cloth_size=self.last_cloth_size,
                )

            if confirmed is None:
                return

            confirmed_tid = confirmed["target_id"]
            cloth_size = confirmed["cloth_size"]
            success = confirmed["success"]

            self.last_cloth_size = cloth_size
            confirmed_name = TARGET_NAMES.get(confirmed_tid, f"Target_{confirmed_tid}")

            target_pt = dict(
                self.pm.points.get(
                    confirmed_tid,
                    {
                        "target_id": confirmed_tid,
                        "name": confirmed_name,
                        "x": rpose.get("x", 0.0),
                        "y": rpose.get("y", 0.0),
                        "yaw": rpose.get("yaw", 0.0),
                        "target_velocity": v_tgt,
                        "memo": "",
                    },
                )
            )

            record["target"] = target_pt
            record["cloth_size"] = cloth_size
            record["success"] = bool(success)
            record["result"] = "success" if success else "failure"
            record["target_confirmed"] = True
            record["confirmed_target_id"] = confirmed_tid

            self._update_record_in_file(line_index, record)
            self.get_logger().info(
                f"Shot #{record['shot_id']} confirmed: Target={confirmed_tid}({confirmed_name}), "
                f"Cloth={cloth_size}, Result={'SUCCESS' if success else 'FAILURE'}"
            )

    def _run_gui_confirmation(
        self,
        shot_id: int,
        est_id: int,
        est_name: str,
        est_reason: str,
        v_tgt: float,
        rpose: dict,
        default_cloth_size: str,
    ) -> Optional[dict]:
        """Run a lightweight PyQt5 subprocess dialog so it works cleanly from any thread."""
        script = """
import json, sys
from PyQt5 import QtWidgets, QtCore

data = json.loads(sys.argv[1])
app = QtWidgets.QApplication([])
dlg = QtWidgets.QDialog()
dlg.setWindowTitle(f"Shot #{data['shot_id']} 記録確認")
dlg.setMinimumWidth(420)
layout = QtWidgets.QVBoxLayout(dlg)

pose = data["rpose"]
pose_str = (
    f"x={pose['x']:.2f}m, y={pose['y']:.2f}m, yaw={pose['yaw']:.2f}rad"
    if pose.get("valid")
    else "TF未取得 (map->base_link)"
)
info = QtWidgets.QLabel(
    f"<b>Shot #{data['shot_id']} 射出完了</b><br>"
    f"ロボット位置 (map): {pose_str}<br>"
    f"目標リリース速度: <b>{data['v_tgt']:.2f} m/s</b><br>"
    f"推定ターゲット: <b>{data['est_id']}: {data['est_name']}</b><br>"
    f"<small>推定根拠: {data['est_reason']}</small>"
)
info.setWordWrap(True)
layout.addWidget(info)

form = QtWidgets.QFormLayout()

cmb_target = QtWidgets.QComboBox()
targets = [
    (1, "1: FixedBucket1 (中央手前バケツ)"),
    (2, "2: FixedBucket2 (左奥バケツ)"),
    (3, "3: FixedBucket3 (右奥バケツ)"),
    (6, "6: Flag (旗)"),
    (7, "7: MovingBucket (相手ロボット上バケツ)"),
    (4, "4: Desk1"),
    (5, "5: Desk2"),
    (0, "0: StartZone / その他"),
]
sel_idx = 0
for i, (tid, label) in enumerate(targets):
    cmb_target.addItem(label, tid)
    if tid == data["est_id"]:
        sel_idx = i
cmb_target.setCurrentIndex(sel_idx)
form.addRow("1. 狙ったターゲット (推定確認):", cmb_target)

size_widget = QtWidgets.QWidget()
size_layout = QtWidgets.QHBoxLayout(size_widget)
size_layout.setContentsMargins(0, 0, 0, 0)
rb_large = QtWidgets.QRadioButton("大きな雑巾 (Large)")
rb_small = QtWidgets.QRadioButton("小さな雑巾 (Small)")
if data["default_cloth_size"] == "small":
    rb_small.setChecked(True)
else:
    rb_large.setChecked(True)
size_layout.addWidget(rb_large)
size_layout.addWidget(rb_small)
form.addRow("2. 使用した雑巾サイズ:", size_widget)

res_widget = QtWidgets.QWidget()
res_layout = QtWidgets.QHBoxLayout(res_widget)
res_layout.setContentsMargins(0, 0, 0, 0)
rb_success = QtWidgets.QRadioButton("成功 (Success)")
rb_failure = QtWidgets.QRadioButton("失敗 (Failure)")
rb_failure.setChecked(True)
res_layout.addWidget(rb_success)
res_layout.addWidget(rb_failure)
form.addRow("3. 投擲結果 (成否):", res_widget)

layout.addLayout(form)

btns = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok)
btns.accepted.connect(dlg.accept)
layout.addWidget(btns)

dlg.setWindowFlags(dlg.windowFlags() | QtCore.Qt.WindowStaysOnTopHint)
dlg.exec_()

out = {
    "target_id": int(cmb_target.currentData()),
    "cloth_size": "large" if rb_large.isChecked() else "small",
    "success": bool(rb_success.isChecked()),
}
print("CONFIRM_JSON:" + json.dumps(out))
"""
        payload = json.dumps({
            "shot_id": shot_id,
            "est_id": est_id,
            "est_name": est_name,
            "est_reason": est_reason,
            "v_tgt": v_tgt,
            "rpose": rpose,
            "default_cloth_size": default_cloth_size,
        })
        try:
            proc = subprocess.run(
                [sys.executable, "-c", script, payload],
                capture_output=True,
                text=True,
                timeout=120,
            )
            for line in proc.stdout.splitlines():
                if line.startswith("CONFIRM_JSON:"):
                    return json.loads(line[len("CONFIRM_JSON:"):])
        except Exception as e:
            self.get_logger().warn(f"GUI confirmation dialog skipped: {e}")
        return None

    def _run_cli_confirmation(
        self,
        shot_id: int,
        est_id: int,
        est_name: str,
        est_reason: str,
        v_tgt: float,
        rpose: dict,
        default_cloth_size: str,
    ) -> Optional[dict]:
        """Interactive terminal prompt fallback."""
        try:
            print("\n" + "=" * 60)
            print(f"[Shot #{shot_id} 記録確認]")
            print(
                f"  base_link on map : x={rpose.get('x', 0.0):.2f}, y={rpose.get('y', 0.0):.2f}, "
                f"yaw={rpose.get('yaw', 0.0):.2f} rad (valid={rpose.get('valid', False)})"
            )
            print(f"  目標リリース速度 : {v_tgt:.2f} m/s")
            print(f"  推定ターゲット   : {est_id} ({est_name}) [{est_reason}]")
            t_in = input(
                f"1. 推定ターゲット [{est_id}:{est_name}] で合っていますか？ (Enter=はい / 番号 1,2,3,6,7 で変更): "
            ).strip()
            confirmed_tid = int(t_in) if t_in.isdigit() else est_id

            s_default = "L" if default_cloth_size == "large" else "S"
            s_in = input(f"2. 雑巾サイズ [L=大きな雑巾 / S=小さな雑巾] (デフォルト={s_default}): ").strip().upper()
            if s_in == "S":
                cloth_size = "small"
            elif s_in == "L":
                cloth_size = "large"
            else:
                cloth_size = default_cloth_size

            r_in = input("3. 投擲結果は成功しましたか？ [y=成功 / n=失敗] (デフォルト=n): ").strip().lower()
            success = r_in in ("y", "yes", "1", "s", "ok")
            print("=" * 60 + "\n")
            return {
                "target_id": confirmed_tid,
                "cloth_size": cloth_size,
                "success": success,
            }
        except Exception as e:
            self.get_logger().warn(f"CLI confirmation skipped: {e}")
            return None

    def commit_session_summary(self):
        """Commit accumulated session trial records to shot-trial-log repository on exit."""
        if not self.auto_git_commit:
            return
        git_dir = os.path.join(self.output_dir, ".git")
        if not os.path.exists(git_dir):
            return

        try:
            cmd_status = ["git", "status", "--porcelain", self.log_filename]
            res_status = subprocess.run(cmd_status, cwd=self.output_dir, capture_output=True, text=True)
            if not res_status.stdout.strip():
                self.get_logger().info("No uncommitted trials in session.")
                return

            cmd_add = ["git", "add", self.log_filename]
            subprocess.run(cmd_add, cwd=self.output_dir, check=True, capture_output=True)

            now = datetime.datetime.now()
            start_str = self.session_start_time.strftime("%Y-%m-%d %H:%M")
            end_str = now.strftime("%H:%M")
            total_shots = len(self.session_shots)

            target_counts = collections.Counter(s["target_name"] for s in self.session_shots)
            breakdown_lines = [f"- {name}: {count} shots" for name, count in target_counts.items()]
            breakdown_str = "\n".join(breakdown_lines)

            header = f"log: trial session summary ({start_str} - {end_str}, {total_shots} shots)"
            body = f"Recorded {total_shots} shots between {start_str} and {end_str}."
            if breakdown_str:
                body += f"\n\nTargets:\n{breakdown_str}"

            full_msg = f"{header}\n\n{body}"
            cmd_commit = ["git", "commit", "-m", full_msg]
            subprocess.run(cmd_commit, cwd=self.output_dir, check=True, capture_output=True)
            self.get_logger().info(f"Git committed session summary to shot-trial-log:\n{header}")
        except Exception as e:
            self.get_logger().warn(f"Auto git commit on shutdown skipped: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TrialRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.commit_session_summary()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
