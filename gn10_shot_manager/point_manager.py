"""Point manager: YAML and ROS2 Service handling with Git versioning."""

import os
import subprocess
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import yaml
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

try:
    from gn10_navigation.srv import UpdatePoint
except ImportError:
    UpdatePoint = None


def find_default_yaml_path() -> str:
    """Find the best shooting points YAML file path."""
    # 1. Environment variable
    env_path = os.environ.get("GN10_SHOOTING_POINTS_PATH")
    if env_path and os.path.exists(env_path):
        return os.path.abspath(env_path)

    # 2. Dedicated data/shooting-points repository
    candidates = [
        os.path.expanduser("~/ros2_ws/data/shooting-points/shooting_points.yaml"),
        os.path.expanduser("~/ros2_ws/data/shooting-points/shooting_point.yaml"),
        os.path.expanduser("~/ros2_ws/data/shooting-point-config/shooting_points.yaml"),
        os.path.abspath("data/shooting-points/shooting_points.yaml"),
        os.path.abspath("data/shooting-points/shooting_point.yaml"),
    ]
    for c in candidates:
        if os.path.exists(c):
            return os.path.abspath(c)

    # 3. Source fallback
    src_fallback = os.path.expanduser(
        "~/ros2_ws/src/gn10-navigation/config/shooting_points.yaml"
    )
    if os.path.exists(src_fallback):
        return src_fallback

    # 4. If nothing exists yet, default to data/shooting-points
    return os.path.expanduser("~/ros2_ws/data/shooting-points/shooting_points.yaml")


def find_git_repo_root(file_path: str) -> Optional[str]:
    """Find the top-level git repository directory containing the given file path."""
    curr = os.path.abspath(file_path)
    if os.path.isfile(curr):
        curr = os.path.dirname(curr)
    while curr and curr != "/":
        if os.path.isdir(os.path.join(curr, ".git")):
            return curr
        parent = os.path.dirname(curr)
        if parent == curr:
            break
        curr = parent
    return None


class PointManager:
    """Manages target shooting points via YAML and ROS 2 services."""

    def __init__(self, yaml_path: Optional[str] = None, node: Optional[Node] = None):
        if yaml_path is None:
            self.yaml_path = find_default_yaml_path()
        else:
            self.yaml_path = os.path.abspath(yaml_path)

        self.node = node
        self.points: Dict[int, dict] = {}  # target_id (int) -> dict

        # Cached ROS 2 service clients
        self._update_client = None
        self._save_client = None

        if self.node is not None:
            self._init_clients()

        self.load_yaml()

    def set_node(self, node: Node):
        """Set or update the ROS 2 node."""
        self.node = node
        self._init_clients()

    def _init_clients(self):
        if self.node is not None and UpdatePoint is not None:
            if self._update_client is None:
                self._update_client = self.node.create_client(
                    UpdatePoint, "/target_manager/update_point"
                )
            if self._save_client is None:
                self._save_client = self.node.create_client(
                    Trigger, "/target_manager/save_points"
                )

    def load_yaml(self) -> bool:
        """Load shooting points from YAML file."""
        if not os.path.exists(self.yaml_path):
            if self.node:
                self.node.get_logger().warn(f"YAML path does not exist: {self.yaml_path}")
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
                self.node.get_logger().error(f"Failed to load yaml from {self.yaml_path}: {e}")
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

            parent_dir = os.path.dirname(self.yaml_path)
            if parent_dir and not os.path.exists(parent_dir):
                os.makedirs(parent_dir, exist_ok=True)

            with open(self.yaml_path, "w", encoding="utf-8") as f:
                yaml.dump(doc, f, sort_keys=False, allow_unicode=True)
            return True
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Failed to save yaml to {self.yaml_path}: {e}")
            return False

    def is_service_ready(self, timeout_sec: float = 0.05) -> bool:
        """Check quickly if /target_manager/update_point service is available."""
        if self.node is None or UpdatePoint is None:
            return False
        if self._update_client is None:
            self._init_clients()
        if self._update_client is None:
            return False
        return self._update_client.wait_for_service(timeout_sec=timeout_sec)

    def sync_to_ros2(self, target_id: int) -> Tuple[bool, str]:
        """Call /target_manager/update_point service for a single target_id."""
        if not self.is_service_ready(timeout_sec=0.05):
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

        try:
            future = self._update_client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=0.5)
            if future.done() and future.result() is not None:
                res = future.result()
                return res.success, res.message
            return False, "Service call timed out"
        except Exception as e:
            return False, f"Service error: {e}"

    def sync_points_to_ros2(self, target_ids: Optional[List[int]] = None) -> Tuple[bool, str]:
        """Sync a batch of points to ROS 2 quickly."""
        if not self.is_service_ready(timeout_sec=0.05):
            return False, "ROS 2 target_manager service offline"

        if target_ids is None:
            target_ids = list(self.points.keys())

        failed_count = 0
        for tid in target_ids:
            ok, _ = self.sync_to_ros2(tid)
            if not ok:
                failed_count += 1

        if failed_count == 0:
            return True, f"Synced {len(target_ids)} points to ROS 2"
        return False, f"Failed syncing {failed_count}/{len(target_ids)} points"

    def save_via_ros2(self) -> Tuple[bool, str]:
        """Call /target_manager/save_points service or fallback to direct file save."""
        # Check if service is immediately ready without long blocking
        if self._save_client is None:
            self._init_clients()

        if self.node is not None and self._save_client is not None:
            if self._save_client.wait_for_service(timeout_sec=0.05):
                try:
                    req = Trigger.Request()
                    future = self._save_client.call_async(req)
                    rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
                    if future.done() and future.result() is not None:
                        res = future.result()
                        # Also ensure direct save matches in case paths differ
                        self.save_yaml_direct()
                        return res.success, res.message
                except Exception as e:
                    if self.node:
                        self.node.get_logger().warn(f"Save service call failed: {e}")

        # Fallback to direct file save
        success = self.save_yaml_direct()
        msg = f"Saved directly to {os.path.basename(self.yaml_path)}"
        return success, msg

    def git_commit(self, message: str) -> Tuple[bool, str]:
        """Automatically create a git commit for shooting_points.yaml in its repository."""
        repo_dir = find_git_repo_root(self.yaml_path)
        if not repo_dir:
            return False, f"Not a git repository: {self.yaml_path}"

        try:
            filename = os.path.basename(self.yaml_path)
            # Add file to git index
            subprocess.run(["git", "add", filename], cwd=repo_dir, check=True, capture_output=True)

            # Check if there are changes to commit
            res_status = subprocess.run(
                ["git", "status", "--porcelain", filename],
                cwd=repo_dir,
                check=True,
                capture_output=True,
                text=True,
            )
            if not res_status.stdout.strip():
                return True, "No changes to commit"

            res_commit = subprocess.run(
                ["git", "commit", "-m", message],
                cwd=repo_dir,
                check=True,
                capture_output=True,
                text=True,
            )
            return True, f"Git committed: {message}"
        except Exception as e:
            return False, f"Git commit failed: {e}"
