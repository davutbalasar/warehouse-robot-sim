# environment_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
import random
import json

# ── World ────────────────────────────────────────────────────────────────────
WORLD_SIZE = 10.0

# Patrol corridor
PATROL_Y = 5.0

# Section y-ranges (open toward corridor)
TOP_SECTION_Y    = (5.5, 9.0)   # open at bottom (y=5.5), wall at top (y=9.0)
BOTTOM_SECTION_Y = (1.0, 4.5)   # wall at bottom (y=1.0), open at top (y=4.5)

# Border walls (horizontal, thin)
TOP_BORDER_Y    = 9.0
BOTTOM_BORDER_Y = 1.0

# Column layout: 5 columns
# Left outer wall at x=0.5, right outer wall at x=9.5
# Internal dividers at x=2.5, 4.5, 6.5, 8.5  → NOT the above
# Let's use:
#   outer walls:   x=0.5, x=9.5
#   dividers:      x=2.5, x=4.5, x=6.5, x=8.5  (NO — that gives 4 dividers = 5 columns OK)
# Column centres:  x=1.5, 3.5, 5.5, 7.5, 9.0   ← actually:
#   Between 0.5 and 2.5 → centre 1.5
#   Between 2.5 and 4.5 → centre 3.5
#   Between 4.5 and 6.5 → centre 5.5
#   Between 6.5 and 8.5 → centre 7.5
#   Between 8.5 and 9.5 → centre 9.0   ← narrow, use 9.0

OUTER_LEFT  = 0.5
OUTER_RIGHT = 9.5
DIVIDERS_X  = [2.5, 4.5, 6.5, 8.5]   # internal vertical walls

ALL_WALL_X = [OUTER_LEFT] + DIVIDERS_X + [OUTER_RIGHT]  # 6 walls → 5 columns

COLUMN_CENTRES_X = []
for i in range(len(ALL_WALL_X) - 1):
    cx = (ALL_WALL_X[i] + ALL_WALL_X[i+1]) / 2
    COLUMN_CENTRES_X.append(round(cx, 2))
# COLUMN_CENTRES_X = [1.5, 3.5, 5.5, 7.5, 9.0] — wait, last = (8.5+9.5)/2 = 9.0
# Better: make columns symmetric — adjust outer walls
# Let's use: outer walls at x=0.0, x=10.0 and dividers at 2.0, 4.0, 6.0, 8.0
# Centres: 1.0, 3.0, 5.0, 7.0, 9.0

OUTER_LEFT  = 0.0
OUTER_RIGHT = 10.0
DIVIDERS_X  = [2.0, 4.0, 6.0, 8.0]
ALL_WALL_X  = [OUTER_LEFT] + DIVIDERS_X + [OUTER_RIGHT]
COLUMN_CENTRES_X = [
    (ALL_WALL_X[i] + ALL_WALL_X[i+1]) / 2
    for i in range(len(ALL_WALL_X) - 1)
]
# = [1.0, 3.0, 5.0, 7.0, 9.0]

# Spawn row y positions (3 per section)
TOP_SPAWN_ROWS    = [8.5, 7.5, 6.5]
BOTTOM_SPAWN_ROWS = [3.5, 2.5, 1.5]

SPAWN_POINTS = []
for cx in COLUMN_CENTRES_X:
    for ry in TOP_SPAWN_ROWS + BOTTOM_SPAWN_ROWS:
        SPAWN_POINTS.append((round(cx, 2), round(ry, 2)))

# Wall definitions for collision / rendering
# Each wall = dict with type, and relevant coordinates
# Horizontal walls: full width, thin  (y fixed, x range)
# Vertical walls: full height per section, thin  (x fixed, y range)

HORIZONTAL_WALLS = [
    # (x_min, x_max, y)
    (0.0, 10.0, TOP_BORDER_Y),
    (0.0, 10.0, BOTTOM_BORDER_Y),
]

VERTICAL_WALLS = []
for wx in ALL_WALL_X:
    # Each vertical wall spans both sections + a bit
    VERTICAL_WALLS.append((wx, BOTTOM_BORDER_Y, wx, TOP_BORDER_Y))
    # (x, y_min, x, y_max)

WALL_THICKNESS = 0.12   # visual only; collision uses exact x/y

# Trash
TRASH_SPAWN_INTERVAL = 7.0
MAX_TRASH = 10


class EnvironmentNode(Node):
    def __init__(self):
        super().__init__('environment_node')

        self.active_trash = set()

        # Publishers
        self.shelf_pub  = self.create_publisher(String, '/world/walls',        1)
        self.trash_pub  = self.create_publisher(String, '/world/trash',       10)
        self.spawn_pub  = self.create_publisher(String, '/world/spawn_points', 1)

        # Subscriber
        self.create_subscription(Pose2D, '/trash/picked', self.picked_cb, 10)

        self._spawn_random(count=6)

        self.create_timer(0.1,                  self.publish_world)
        self.create_timer(TRASH_SPAWN_INTERVAL, self.spawn_timer_cb)

        self.get_logger().info("EnvironmentNode ready.")
        self.get_logger().info(f"Spawn points: {SPAWN_POINTS}")

    def _spawn_random(self, count=1):
        available = [p for p in SPAWN_POINTS if p not in self.active_trash]
        random.shuffle(available)
        for p in available[:count]:
            self.active_trash.add(p)
            self.get_logger().info(f"Spawned trash at {p}")

    def spawn_timer_cb(self):
        if len(self.active_trash) < MAX_TRASH:
            self._spawn_random(1)

    def picked_cb(self, msg):
        to_remove = None
        for (tx, ty) in self.active_trash:
            if abs(tx - msg.x) < 0.1 and abs(ty - msg.y) < 0.1:
                to_remove = (tx, ty)
                break
        if to_remove:
            self.active_trash.discard(to_remove)
            self.get_logger().info(f"Trash removed at {to_remove}")

    def publish_world(self):
        walls_data = {
            "horizontal": HORIZONTAL_WALLS,
            "vertical":   VERTICAL_WALLS,
            "thickness":  WALL_THICKNESS,
            "col_centres": COLUMN_CENTRES_X,
            "top_section":    list(TOP_SECTION_Y),
            "bottom_section": list(BOTTOM_SECTION_Y),
            "patrol_y":   PATROL_Y,
            "dividers_x": DIVIDERS_X,
            "outer_left":  OUTER_LEFT,
            "outer_right": OUTER_RIGHT,
        }
        w = String(); w.data = json.dumps(walls_data)
        self.shelf_pub.publish(w)

        t = String(); t.data = json.dumps(list(self.active_trash))
        self.trash_pub.publish(t)

        s = String(); s.data = json.dumps(SPAWN_POINTS)
        self.spawn_pub.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()