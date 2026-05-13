# robot_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
import json
import math

# ── Constants (must match environment_node) ──────────────────────────────────
PATROL_Y        = 5.0
ROBOT_SPEED     = 0.07
PICKUP_DIST     = 0.25

PATROL_X_MIN    = 1.0
PATROL_X_MAX    = 9.0

# The sections are open on the corridor side at these y boundaries
TOP_SECTION_Y_MIN    = 5.5   # top section starts here (open toward corridor)
TOP_SECTION_Y_MAX    = 9.0   # top border wall
BOTTOM_SECTION_Y_MIN = 1.0   # bottom border wall
BOTTOM_SECTION_Y_MAX = 4.5   # bottom section ends here (open toward corridor)

# Vertical walls (solid, robot cannot cross)
WALL_X      = [0.0, 2.0, 4.0, 6.0, 8.0, 10.0]
COL_CENTRES = [1.0, 3.0, 5.0, 7.0, 9.0]
WALL_HALF   = 0.06   # collision margin


def current_column(x):
    """Column index 0-4 for position x."""
    for i in range(1, len(WALL_X)):
        if x < WALL_X[i]:
            return i - 1
    return len(COL_CENTRES) - 1


def col_x_range(col_idx):
    """Passable x range inside column col_idx."""
    return WALL_X[col_idx] + WALL_HALF, WALL_X[col_idx + 1] - WALL_HALF


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def in_top_section(y):
    return y >= TOP_SECTION_Y_MIN

def in_bottom_section(y):
    return y <= BOTTOM_SECTION_Y_MAX

def in_corridor(y):
    return BOTTOM_SECTION_Y_MAX < y < TOP_SECTION_Y_MIN


class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')

        self.walls_data = None
        self.trash_list = []        # list of (x, y) from environment

        # Robot position — start at leftmost column centre, in corridor
        self.x = COL_CENTRES[0]
        self.y = PATROL_Y

        # FSM
        self.state      = 'patrol'
        self.patrol_dir = +1        # +1 = moving right, -1 = moving left
        self.target     = None      # current (x, y) destination
        self.return_x   = COL_CENTRES[0]

        # Async pickup guard: stores the position we just picked so we don't
        # misread its disappearance from trash_list as "target gone unexpectedly"
        self.just_picked = None

        self.create_subscription(String, '/world/walls', self.walls_cb, 1)
        self.create_subscription(String, '/world/trash', self.trash_cb, 10)

        self.pose_pub   = self.create_publisher(Pose2D, '/robot/pose',   10)
        self.picked_pub = self.create_publisher(Pose2D, '/trash/picked', 10)

        self.create_timer(0.05, self.update)
        self.get_logger().info("RobotNode ready.")

    # ── Callbacks ────────────────────────────────────────────────────────────
    def walls_cb(self, msg):
        self.walls_data = json.loads(msg.data)

    def trash_cb(self, msg):
        self.trash_list = [tuple(t) for t in json.loads(msg.data)]
        # Clear the async guard once environment confirms the item is gone
        if self.just_picked and self.just_picked not in self.trash_list:
            self.just_picked = None

    # ── Vision ───────────────────────────────────────────────────────────────
    def visible_trash(self):
        """
        Returns trash visible from current position.

        Rules:
          - Vertical walls are opaque: only trash in the SAME column lane is visible.
          - When in corridor: can see into BOTH sections in this column lane.
          - When inside a section: can only see trash in THIS section (horizontal
            border walls block sight into the other section).
          - The just_picked item is excluded (already collected this tick).
        """
        col = current_column(self.x)
        x_lo, x_hi = col_x_range(col)

        robot_in_top    = in_top_section(self.y)
        robot_in_bottom = in_bottom_section(self.y)
        robot_in_corr   = in_corridor(self.y)

        visible = []
        for (tx, ty) in self.trash_list:
            if (tx, ty) == self.just_picked:
                continue

            # Must be in same column lane
            if not (x_lo - 0.15 <= tx <= x_hi + 0.15):
                continue

            # Determine which section/zone the trash is in
            trash_top    = in_top_section(ty)
            trash_bottom = in_bottom_section(ty)

            if robot_in_corr:
                # From corridor: see into both sections in this column
                if trash_top or trash_bottom:
                    visible.append((tx, ty))

            elif robot_in_top:
                # Inside top section: only see trash also in top section
                if trash_top:
                    visible.append((tx, ty))

            elif robot_in_bottom:
                # Inside bottom section: only see trash also in bottom section
                if trash_bottom:
                    visible.append((tx, ty))

        return visible

    # ── Movement ─────────────────────────────────────────────────────────────
    def move_toward(self, tx, ty):
        """
        Move one step toward (tx, ty).
        Enforces wall collisions: robot is ALWAYS locked to its current column lane.
        It can only be in a different column if it navigated there via the corridor.
        """
        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)
        if dist < 1e-4:
            return dist

        step  = min(ROBOT_SPEED, dist)
        new_x = self.x + step * dx / dist
        new_y = self.y + step * dy / dist

        # Lock x to current column lane — walls are always solid
        col = current_column(self.x)
        x_lo, x_hi = col_x_range(col)
        new_x = clamp(new_x, x_lo, x_hi)

        # Lock y to world bounds (border walls)
        new_y = clamp(new_y, BOTTOM_SECTION_Y_MIN + WALL_HALF,
                      TOP_SECTION_Y_MAX - WALL_HALF)

        self.x = new_x
        self.y = new_y
        return dist

    # ── Patrol movement (corridor only, changes column freely) ────────────────
    def patrol_step(self):
        """Move left/right along patrol corridor. Column changes happen here."""
        new_x = self.x + self.patrol_dir * ROBOT_SPEED
        if new_x >= PATROL_X_MAX:
            new_x = PATROL_X_MAX
            self.patrol_dir = -1
        elif new_x <= PATROL_X_MIN:
            new_x = PATROL_X_MIN
            self.patrol_dir = +1
        self.x = new_x
        self.y = PATROL_Y   # snap to corridor line

    # ── Main update loop ──────────────────────────────────────────────────────
    def update(self):
        if self.walls_data is None:
            return

        visible = self.visible_trash()

        # ── PATROL ───────────────────────────────────────────────────────────
        if self.state == 'patrol':
            if visible:
                # Pick the closest visible trash as the intended target
                closest = min(visible,
                              key=lambda t: math.hypot(t[0]-self.x, t[1]-self.y))
                # Only turn in once x is aligned with the trash column
                aligned = abs(self.x - closest[0]) < ROBOT_SPEED * 1.5
                if aligned:
                    self.target   = closest
                    self.return_x = self.x
                    self.state    = 'go_to_trash'
                    self.get_logger().info(f"Aligned with {closest}, turning in.")
                else:
                    # Steer patrol toward the trash column first
                    self.patrol_dir = +1 if closest[0] > self.x else -1
                    self.patrol_step()
            else:
                self.patrol_step()

        # ── GO TO TRASH ───────────────────────────────────────────────────────
        elif self.state == 'go_to_trash':
            # Target gone and it wasn't us who just picked it → abort
            if self.target not in self.trash_list and self.target != self.just_picked:
                self.state = 'return'
                return

            dist = self.move_toward(self.target[0], self.target[1])

            if dist < PICKUP_DIST:
                # Pick it up
                self.just_picked = self.target
                self._pick(self.target)

                # Scan for more trash visible RIGHT NOW from this position
                # (just_picked is excluded inside visible_trash)
                visible_now = self.visible_trash()

                if visible_now:
                    # Chain directly to next trash — no return to middle
                    closest = min(visible_now,
                                  key=lambda t: math.hypot(t[0]-self.x, t[1]-self.y))
                    self.target = closest
                    self.get_logger().info(f"Chaining to next trash at {closest}.")
                else:
                    # Nothing left visible — return to corridor
                    self.state  = 'return'
                    self.target = None

        # ── RETURN TO CORRIDOR ────────────────────────────────────────────────
        elif self.state == 'return':
            dist = self.move_toward(self.return_x, PATROL_Y)
            if dist < 0.08:
                self.x     = self.return_x
                self.y     = PATROL_Y
                self.state = 'patrol'
                self.get_logger().info("Back on patrol.")

        # ── Publish pose ──────────────────────────────────────────────────────
        pose = Pose2D()
        pose.x = self.x
        pose.y = self.y
        pose.theta = float(self.patrol_dir)
        self.pose_pub.publish(pose)

    def _pick(self, pos):
        msg = Pose2D()
        msg.x, msg.y, msg.theta = pos[0], pos[1], 0.0
        self.picked_pub.publish(msg)
        self.get_logger().info(f"Picked trash at {pos}.")


def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()