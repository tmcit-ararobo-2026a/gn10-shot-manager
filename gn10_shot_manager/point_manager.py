"""Point manager: YAML and ROS2 Service handling with Git versioning."""

import os
import subprocess
import yaml
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

try:
    from gn10_navigation.srv import UpdatePoint
except ImportError:
    UpdatePoint = None


class PointManager:
    """Manages target shooting points via YAML and ROS 2 services."""

    def __init__(self, yaml_path: str = None, node: Node = None):
        if yaml_path is None:
            # Default location in source tree
            ws_dir = os.path.expanduser("/home/gn10/ros2_ws")
            self.yaml_path = os.path.join(
                ws_dir, "src/gn10-navigation/config/shooting_points.yaml"
            )
        else:
            self.yaml_path = yaml_path

        self.node = node
        self.points = {}  # target_id (int) -> dict
        self.load_yaml()

    def load_yaml(self) -> bool:
        """Load shooting points from YAML file."""
        if not os.path.exists(self.yaml_path):
            return False
        try:
            with open(self.yaml_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
            self.points.clear()
            for item in data.get("shooting_points", []):
                tid = int(item.get("target_id", 0))
                self.points[tid] = {
                    "target_id": tid,
                    "name": str(item.get("name", f"Target_{tid}")),
                    "x": float(item.get("x", 0.0)),
                    "y": float(item.get("y", 0.0)),
                    "yaw": float(item.get("yaw", 0.0)),
                    "target_velocity": float(item.get("target_velocity", 0.0)),
                    "memo": str(item.get("memo", "")),
                }
            return True
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Failed to load yaml: {e}")
            return False

    def save_yaml_direct(self) -> bool:
        """Directly write in-memory points back to YAML."""
        try:
            items = []
            for tid in sorted(self.points.keys()):
                p = self.points[tid]
                items.append({
                    "target_id": p["target_id"],
                    "name": p["name"],
                    "x": round(float(p["x"]), 4),
                    "y": round(float(p["y"]), 4),
                    "yaw": round(float(p["yaw"]), 4),
                    "target_velocity": round(float(p.get("target_velocity", 0.0)), 4),
                    "memo": p["memo"],
                })
            doc = {"shooting_points": items}
            with open(self.yaml_path, "w", encoding="utf-8") as f:
                yaml.dump(doc, f, sort_keys=False, allow_unicode=True)
            return True
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Failed to save yaml: {e}")
            return False

    def sync_to_ros2(self, target_id: int) -> tuple[bool, str]:
        """Call /target_manager/update_point service for target_id."""
        if self.node is None or UpdatePoint is None:
            return False, "ROS 2 node or UpdatePoint srv unavailable"

        client = self.node.create_client(UpdatePoint, "/target_manager/update_point")
        if not client.wait_for_service(timeout_sec=0.5):
            return False, "Service /target_manager/update_point not ready"

        p = self.points.get(target_id)
        if not p:
            return False, f"Target {target_id} not found"

        req = UpdatePoint.Request()
        req.target_id = int(target_id)
        req.x = float(p["x"])
        req.y = float(p["y"])
        req.yaw = float(p["yaw"])
        if hasattr(req, "target_velocity"):
            req.target_velocity = float(p.get("target_velocity", 0.0))
        req.memo = str(p["memo"])

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        if future.done() and future.result() is not None:
            res = future.result()
            return res.success, res.message
        return False, "Service call timed out"

    def save_via_ros2(self) -> tuple[bool, str]:
        """Call /target_manager/save_points service to flush in-memory points to disk."""
        if self.node is None:
            # Fallback to direct file save
            success = self.save_yaml_direct()
            return success, "Saved directly to file (ROS2 offline)"

        client = self.node.create_client(Trigger, "/target_manager/save_points")
        if not client.wait_for_service(timeout_sec=0.5):
            # Fallback to direct file save
            success = self.save_yaml_direct()
            return success, "Saved directly to file (Service not ready)"

        req = Trigger.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        if future.done() and future.result() is not None:
            res = future.result()
            return res.success, res.message
        # Fallback
        self.save_yaml_direct()
        return True, "Saved directly to file (Service timed out)"

    def git_commit(self, message: str) -> tuple[bool, str]:
        """Automatically create a git commit for shooting_points.yaml."""
        try:
            repo_dir = os.path.dirname(os.path.dirname(self.yaml_path))
            cmd_add = ["git", "add", self.yaml_path]
            subprocess.run(cmd_add, cwd=repo_dir, check=True, capture_output=True)

            # Check if there are changes to commit
            status_cmd = ["git", "status", "--porcelain", self.yaml_path]
            res_status = subprocess.run(status_cmd, cwd=repo_dir, check=True, capture_output=True, text=True)
            if not res_status.stdout.strip():
                return True, "No changes to commit"

            cmd_commit = ["git", "commit", "-m", message]
            res_commit = subprocess.run(cmd_commit, cwd=repo_dir, check=True, capture_output=True, text=True)
            return True, f"Git committed: {message}"
        except Exception as e:
            return False, f"Git commit failed: {e}"

