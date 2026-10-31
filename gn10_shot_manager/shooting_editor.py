"""Interactive Terminal TUI for Shooting Point Micro-Adjustment and Management."""

import curses
import math
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from gn10_shot_manager.point_manager import PointManager


class ShootingEditorApp:
    """Curses-based TUI for shooting point adjustment."""

    def __init__(self, stdscr, node: Node, pm: PointManager):
        self.stdscr = stdscr
        self.node = node
        self.pm = pm

        # Publishers for active testing
        self.target_id_pub = self.node.create_publisher(UInt8, "/robot/target_id", 10)

        # UI state
        self.selected_idx = 0
        self.selected_field = 0  # 0: X, 1: Y, 2: Yaw, 3: V_tgt, 4: Memo
        self.fields = ["X (m)", "Y (m)", "Yaw (rad)", "V_tgt (m/s)", "Memo"]
        self.step_sizes = [0.005, 0.005, 0.01, 0.1, None]
        self.status_msg = "Ready. Use arrows to navigate, +/- to adjust, 's' to save."
        self.dirty = False

        # Sorted target list
        self.target_ids = sorted(self.pm.points.keys()) if self.pm.points else list(range(8))

    def run(self):
        curses.curs_set(0)
        self.stdscr.nodelay(True)
        self.stdscr.timeout(100)  # 100ms refresh
        curses.use_default_colors()

        # Color pairs
        if curses.has_colors():
            curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_CYAN)    # Header
            curses.init_pair(2, curses.COLOR_WHITE, curses.COLOR_BLUE)    # Selected row
            curses.init_pair(3, curses.COLOR_GREEN, -1)                  # Status OK
            curses.init_pair(4, curses.COLOR_RED, -1)                    # Warning/Modified
            curses.init_pair(5, curses.COLOR_YELLOW, -1)                 # Active Target
            curses.init_pair(6, curses.COLOR_CYAN, -1)                   # Keybinds

        while True:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            self.draw()
            ch = self.stdscr.getch()
            if ch == -1:
                continue

            if ch in [ord('q'), ord('Q'), 27]:  # ESC or q
                if self.dirty:
                    if self.confirm_prompt("Unsaved changes exist! Quit anyway? (y/n): "):
                        break
                else:
                    break
            elif ch in [curses.KEY_UP, ord('k')]:
                if self.selected_idx > 0:
                    self.selected_idx -= 1
            elif ch in [curses.KEY_DOWN, ord('j')]:
                if self.selected_idx < len(self.target_ids) - 1:
                    self.selected_idx += 1
            elif ch in [curses.KEY_LEFT, ord('h')]:
                self.adjust_value(delta_factor=-1.0)
            elif ch in [curses.KEY_RIGHT, ord('l')]:
                self.adjust_value(delta_factor=1.0)
            elif ch in [ord('+'), ord('=')]:
                self.adjust_value(delta_factor=1.0)
            elif ch in [ord('-'), ord('_')]:
                self.adjust_value(delta_factor=-1.0)
            elif ch in [ord('>'), ord('.')]:
                self.adjust_value(delta_factor=5.0)  # Coarse
            elif ch in [ord('<'), ord(',')]:
                self.adjust_value(delta_factor=-5.0)  # Coarse
            elif ch in [9, ord('\t')]:  # Tab
                self.selected_field = (self.selected_field + 1) % 5
            elif ch in [ord('1'), ord('2'), ord('3'), ord('4'), ord('5')]:
                self.selected_field = ch - ord('1')
            elif ch in [ord('e'), ord('E'), 10, curses.KEY_ENTER]:  # Enter or e to edit
                self.edit_current_field()
            elif ch in [ord('s'), ord('S')]:
                self.save_and_commit()
            elif ch in [ord('r'), ord('R')]:
                if self.pm.load_yaml(fetch_remote_first=True):
                    self.dirty = False
                    if self.pm.source_origin == "remote":
                        self.status_msg = f"Reloaded points from remote: {self.pm.remote_file_path}"
                    else:
                        self.status_msg = "Reloaded points from local YAML file."
                else:
                    self.status_msg = "Failed to reload points."
            elif ch in [ord('t'), ord('T')]:
                self.publish_active_target()

    def get_current_point(self):
        if not self.target_ids:
            return None
        tid = self.target_ids[self.selected_idx]
        return self.pm.points.get(tid)

    def adjust_value(self, delta_factor: float):
        pt = self.get_current_point()
        if not pt:
            return

        if self.selected_field == 0:  # X
            pt["x"] = round(pt["x"] + self.step_sizes[0] * delta_factor, 4)
            self.dirty = True
            self.status_msg = f"Target {pt['target_id']} X adjusted to {pt['x']:.4f} m"
        elif self.selected_field == 1:  # Y
            pt["y"] = round(pt["y"] + self.step_sizes[1] * delta_factor, 4)
            self.dirty = True
            self.status_msg = f"Target {pt['target_id']} Y adjusted to {pt['y']:.4f} m"
        elif self.selected_field == 2:  # Yaw
            pt["yaw"] = round(pt["yaw"] + self.step_sizes[2] * delta_factor, 4)
            # Wrap to [-pi, pi]
            if pt["yaw"] > math.pi:
                pt["yaw"] -= 2 * math.pi
            elif pt["yaw"] < -math.pi:
                pt["yaw"] += 2 * math.pi
            self.dirty = True
            deg = math.degrees(pt["yaw"])
            self.status_msg = f"Target {pt['target_id']} Yaw adjusted to {pt['yaw']:.4f} rad ({deg:.1f}°)"
        elif self.selected_field == 3:  # V_tgt
            new_v = round(pt.get("target_velocity", 0.0) + self.step_sizes[3] * delta_factor, 3)
            pt["target_velocity"] = max(0.0, new_v)
            self.dirty = True
            self.status_msg = f"Target {pt['target_id']} V_tgt adjusted to {pt['target_velocity']:.3f} m/s"

    def edit_current_field(self):
        pt = self.get_current_point()
        if not pt:
            return

        field_name = self.fields[self.selected_field]
        if self.selected_field == 4:
            current_val = pt["memo"]
        elif self.selected_field == 3:
            current_val = str(pt.get("target_velocity", 0.0))
        elif self.selected_field == 2:
            current_val = str(pt["yaw"])
        elif self.selected_field == 1:
            current_val = str(pt["y"])
        else:
            current_val = str(pt["x"])

        prompt = f"Enter new {field_name} (Current: {current_val}): "
        new_val = self.prompt_input(prompt)
        if new_val is None:
            return

        try:
            if self.selected_field == 0:
                pt["x"] = round(float(new_val), 4)
            elif self.selected_field == 1:
                pt["y"] = round(float(new_val), 4)
            elif self.selected_field == 2:
                pt["yaw"] = round(float(new_val), 4)
            elif self.selected_field == 3:
                pt["target_velocity"] = max(0.0, round(float(new_val), 4))
            elif self.selected_field == 4:
                pt["memo"] = str(new_val).strip()
            self.dirty = True
            self.status_msg = f"Updated {field_name} for Target {pt['target_id']}."
        except ValueError:
            self.status_msg = f"Invalid number input: '{new_val}'"

    def save_and_commit(self):
        """Sync via ROS2 update service, call save service, and git commit."""
        self.status_msg = "Saving points to memory & disk..."
        self.draw()

        # Update points to ROS2 node memory if online
        sync_ok, sync_msg = self.pm.sync_points_to_ros2(self.target_ids)

        # Save to disk
        ok_save, msg_save = self.pm.save_via_ros2()

        # Ask for optional commit comment or auto-generate
        curr_pt = self.get_current_point()
        default_commit = f"tune(target_{curr_pt['target_id']}): x={curr_pt['x']:.3f}, y={curr_pt['y']:.3f}, yaw={curr_pt['yaw']:.3f}, v={curr_pt.get('target_velocity', 0.0):.2f} [{curr_pt['memo']}]"
        ok_git, msg_git = self.pm.git_commit(default_commit)

        self.dirty = False
        self.status_msg = f"{msg_save} | {msg_git}"

    def publish_active_target(self):
        pt = self.get_current_point()
        if not pt:
            return
        msg = UInt8()
        msg.data = int(pt["target_id"])
        self.target_id_pub.publish(msg)
        self.status_msg = f"Sent /robot/target_id = {pt['target_id']} ({pt['name']}) to robot."

    def prompt_input(self, prompt: str) -> str:
        curses.echo()
        curses.curs_set(1)
        h, w = self.stdscr.getmaxyx()
        self.stdscr.addstr(h - 2, 2, " " * (w - 4))
        self.stdscr.addstr(h - 2, 2, prompt, curses.color_pair(5))
        self.stdscr.refresh()

        buf = []
        while True:
            ch = self.stdscr.getch()
            if ch in [10, curses.KEY_ENTER]:
                break
            elif ch == 27:  # ESC
                buf = None
                break
            elif ch in [curses.KEY_BACKSPACE, 127, 8]:
                if buf:
                    buf.pop()
                    self.stdscr.addstr(h - 2, 2 + len(prompt) + len(buf), " ")
                    self.stdscr.move(h - 2, 2 + len(prompt) + len(buf))
            elif 32 <= ch <= 126:
                buf.append(chr(ch))
                self.stdscr.addch(h - 2, 2 + len(prompt) + len(buf) - 1, ch)

        curses.noecho()
        curses.curs_set(0)
        return "".join(buf) if buf is not None else None

    def confirm_prompt(self, prompt: str) -> bool:
        h, w = self.stdscr.getmaxyx()
        self.stdscr.addstr(h - 2, 2, " " * (w - 4))
        self.stdscr.addstr(h - 2, 2, prompt, curses.color_pair(4) | curses.A_BOLD)
        self.stdscr.refresh()
        while True:
            ch = self.stdscr.getch()
            if ch in [ord('y'), ord('Y')]:
                return True
            elif ch in [ord('n'), ord('N'), 27]:
                return False

    def draw(self):
        self.stdscr.erase()
        h, w = self.stdscr.getmaxyx()

        # 1. Header
        source_label = (
            f"[Remote: {self.pm.remote_file_path or 'target_manager'}]"
            if self.pm.source_origin == "remote"
            else f"[Local: {os.path.basename(self.pm.yaml_path)}]"
        )
        header_text = f" GN10 SHOOTING POINT TUNER & EDITOR {source_label} {'[MODIFIED *]' if self.dirty else ''}"
        self.stdscr.attron(curses.color_pair(1) | curses.A_BOLD)
        self.stdscr.addstr(0, 0, header_text.ljust(w)[:w - 1])
        self.stdscr.attroff(curses.color_pair(1) | curses.A_BOLD)

        # 2. Table of points
        headers = ["ID", "Name", "X [m]", "Y [m]", "Yaw [rad]", "V_tgt [m/s]", "Memo / Notes"]
        table_top = 2
        header_line = f" {headers[0]:<4} {headers[1]:<14} {headers[2]:>9} {headers[3]:>9} {headers[4]:>12} {headers[5]:>11}  {headers[6]}"
        self.stdscr.addstr(table_top, 0, header_line[:w - 1], curses.A_UNDERLINE | curses.A_BOLD)

        for i, tid in enumerate(self.target_ids):
            row_y = table_top + 1 + i
            if row_y >= h - 10:
                break
            pt = self.pm.points.get(tid, {})
            name = pt.get("name", "")
            x = pt.get("x", 0.0)
            y = pt.get("y", 0.0)
            yaw = pt.get("yaw", 0.0)
            v_tgt = pt.get("target_velocity", 0.0)
            memo = pt.get("memo", "")
            deg = math.degrees(yaw)

            line = f" [{tid}] {name:<13} {x:>9.3f} {y:>9.3f} {yaw:>6.3f} ({deg:>4.0f}°) {v_tgt:>8.2f}m/s  {memo}"
            if i == self.selected_idx:
                self.stdscr.attron(curses.color_pair(2) | curses.A_BOLD)
                self.stdscr.addstr(row_y, 0, line.ljust(w - 1)[:w - 1])
                self.stdscr.attroff(curses.color_pair(2) | curses.A_BOLD)
            else:
                self.stdscr.addstr(row_y, 0, line[:w - 1])

        # 3. Micro-Adjustment Panel
        curr_pt = self.get_current_point()
        panel_y = table_top + len(self.target_ids) + 2
        if panel_y < h - 7 and curr_pt:
            self.stdscr.addstr(panel_y, 2, "── Fine Adjustment Controls ───────────────────────────────", curses.color_pair(6))
            field_line = " "
            for fi, fname in enumerate(self.fields):
                val_str = ""
                if fi == 0:
                    val_str = f"{curr_pt['x']:.4f}m"
                elif fi == 1:
                    val_str = f"{curr_pt['y']:.4f}m"
                elif fi == 2:
                    val_str = f"{curr_pt['yaw']:.4f}rad ({math.degrees(curr_pt['yaw']):.1f}°)"
                elif fi == 3:
                    val_str = f"{curr_pt.get('target_velocity', 0.0):.2f}m/s"
                elif fi == 4:
                    val_str = f"'{curr_pt['memo']}'"

                item_str = f" [{fi + 1}] {fname}: {val_str} "
                if fi == self.selected_field:
                    self.stdscr.attron(curses.color_pair(1) | curses.A_BOLD)
                    self.stdscr.addstr(panel_y + 1, len(field_line), item_str)
                    self.stdscr.attroff(curses.color_pair(1) | curses.A_BOLD)
                else:
                    self.stdscr.addstr(panel_y + 1, len(field_line), item_str)
                field_line += item_str + "  "

        # 4. Keybind guide
        guide_y = h - 5
        self.stdscr.addstr(guide_y, 2, "Keys: [↑/↓] Select  [←/→] ±Step  [</>] Coarse  [Tab/1-5] Field  [Enter/e] Direct Edit", curses.color_pair(6))
        self.stdscr.addstr(guide_y + 1, 2, "      [s] Save & Git Commit  [t] Set Active Target  [q] Quit", curses.color_pair(6))

        # 5. Status line
        status_color = curses.color_pair(4) if self.dirty else curses.color_pair(3)
        self.stdscr.addstr(h - 2, 2, f"Status: {self.status_msg}"[:w - 4], status_color | curses.A_BOLD)

        self.stdscr.refresh()


def main(args=None):
    rclpy.init(args=args)
    node = Node("shooting_editor_tui")
    pm = PointManager(node=node)

    try:
        curses.wrapper(lambda stdscr: ShootingEditorApp(stdscr, node, pm).run())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

