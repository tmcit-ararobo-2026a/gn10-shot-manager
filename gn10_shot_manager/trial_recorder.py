"""Trial Recorder: Synchronous recording of MCU sensor feedback and dust cloth tracking per shot."""

import collections
import datetime
import json
import math
import os
import subprocess
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, UInt8
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from gn10_shot_manager.point_manager import PointManager


class TrialRecorderNode(Node):
    """Subscribes to MCU feedback and cloth tracking, records trial window on shot trigger."""

    def __init__(self):
        super().__init__("trial_recorder_node")

        # Declare parameters
        self.declare_parameter("record_duration_sec", 3.5)
        self.declare_parameter("pre_trigger_buffer_sec", 0.5)
        self.declare_parameter("output_dir", os.path.expanduser("~/ros2_ws/data/shot-trial-log"))
        self.declare_parameter("log_filename", "trials.jsonl")
        self.declare_parameter("auto_git_commit", True)

        self.record_duration = float(self.get_parameter("record_duration_sec").value)
        self.pre_buffer_duration = float(self.get_parameter("pre_trigger_buffer_sec").value)
        self.output_dir = str(self.get_parameter("output_dir").value)
        self.log_filename = str(self.get_parameter("log_filename").value)
        self.auto_git_commit = bool(self.get_parameter("auto_git_commit").value)

        os.makedirs(self.output_dir, exist_ok=True)
        self.output_path = os.path.join(self.output_dir, self.log_filename)

        # Load target points for coordinate reference
        self.pm = PointManager(node=self)

        # Session tracking for summary commit on shutdown
        self.session_start_time = datetime.datetime.now()
        self.session_shots = []

        # Sensor state
        self.current_target_id = 0
        self.last_shot_count = None
        self.is_recording = False
        self.record_start_time = 0.0
        self.active_shot_id = 0

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
        }

        # Circular buffer for pre-trigger data (stores tuples of (time, snapshot))
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

        # Subscriptions to realsense_dust_cloth
        self.create_subscription(PointStamped, "/cloth/position", self.on_cloth_position, 10)
        self.create_subscription(Path, "/cloth/measured_path", self.on_cloth_path, 10)

        # 50Hz periodic sampling timer
        self.sample_timer = self.create_timer(0.02, self.on_sample_timer)

        self.get_logger().info(f"TrialRecorderNode initialized. Logging to: {self.output_path}")

    def update_val(self, key, val):
        self.latest_data[key] = float(val)

    def on_target_id(self, msg: UInt8):
        self.current_target_id = int(msg.data)

    def on_cloth_position(self, msg: PointStamped):
        self.latest_data["cloth_x"] = float(msg.point.x)
        self.latest_data["cloth_y"] = float(msg.point.y)
        self.latest_data["cloth_z"] = float(msg.point.z)

    def on_cloth_path(self, msg: Path):
        if self.is_recording:
            self.cloth_path_points = [
                {"x": p.pose.position.x, "y": p.pose.position.y, "z": p.pose.position.z}
                for p in msg.poses
            ]

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
        # (通常増加、または uint8 255->1 オーバーフロー時)
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

        # Pull pre-trigger buffer samples
        cutoff = self.record_start_time - self.pre_buffer_duration
        self.active_samples = [
            s for s in self.buffer if s["abs_time"] >= cutoff
        ]
        # Normalize relative time
        for s in self.active_samples:
            s["rel_time"] = round(s["abs_time"] - self.record_start_time, 3)

    def on_sample_timer(self):
        now = time.time()
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
        }
        self.buffer.append(sample)

        if self.is_recording:
            self.active_samples.append(sample)
            # Check duration
            if now - self.record_start_time >= self.record_duration:
                self.finish_recording()

    def finish_recording(self):
        self.is_recording = False
        duration = time.time() - self.record_start_time
        self.get_logger().info(f"Finished recording shot {self.active_shot_id} ({duration:.2f}s, {len(self.active_samples)} samples)")

        # Target coordinates
        target_pt = self.pm.points.get(self.current_target_id, {
            "target_id": self.current_target_id,
            "name": f"Target_{self.current_target_id}",
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0,
            "memo": ""
        })

        # Landing detection from cloth samples or path
        landing_pos = None
        if self.cloth_path_points:
            landing_pos = self.cloth_path_points[-1]
        else:
            # Look for last valid cloth coordinate
            for s in reversed(self.active_samples):
                if s["cloth_x"] is not None:
                    landing_pos = {"x": s["cloth_x"], "y": s["cloth_y"], "z": s["cloth_z"]}
                    break

        error_data = None
        if landing_pos:
            dx = landing_pos["x"] - target_pt["x"]
            dy = landing_pos["y"] - target_pt["y"]
            dist = math.sqrt(dx * dx + dy * dy)
            error_data = {
                "dx": round(dx, 4),
                "dy": round(dy, 4),
                "distance": round(dist, 4),
            }

        # Calculate statistics
        v_list = [s["launcher_velocity"] for s in self.active_samples]
        c_list = [s["launcher_current"] for s in self.active_samples]
        volt_list = [s["launcher_voltage"] for s in self.active_samples]

        record = {
            "shot_id": self.active_shot_id,
            "timestamp": datetime.datetime.now().isoformat(),
            "target": target_pt,
            "summary": {
                "max_velocity": round(max(v_list) if v_list else 0.0, 2),
                "max_current": round(max(c_list) if c_list else 0.0, 2),
                "avg_voltage": round(sum(volt_list) / len(volt_list) if volt_list else 0.0, 2),
            },
            "landing_position": landing_pos,
            "error": error_data,
            "trajectory_points_count": len(self.cloth_path_points),
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
                }
                for s in self.active_samples
            ]
        }

        # Save to JSON Lines
        try:
            with open(self.output_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(record, ensure_ascii=False) + "\n")
            err_str = f"Error: {error_data['distance']:.3f}m" if error_data else "No landing detected"
            self.get_logger().info(f"Trial saved! Shot #{self.active_shot_id} -> Target {target_pt['name']} | {err_str}")

            # Track in-session record for summary commit on node exit
            self.session_shots.append({
                "shot_id": self.active_shot_id,
                "target_name": target_pt.get("name", f"Target_{self.current_target_id}"),
                "timestamp": datetime.datetime.now(),
                "error": error_data,
            })
        except Exception as e:
            self.get_logger().error(f"Failed to write trial record: {e}")

    def commit_session_summary(self):
        """Commit accumulated session trial records to shot-trial-log repository on exit."""
        if not self.auto_git_commit:
            return
        git_dir = os.path.join(self.output_dir, ".git")
        if not os.path.exists(git_dir):
            return

        try:
            # Check if there are uncommitted changes in the log file
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

            # Target breakdown
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

