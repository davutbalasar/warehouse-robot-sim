# rviz_publisher_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import Pose2D, Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
import json
import math

# ── World constants (must match environment_node) ────────────────────────────
PATROL_Y         = 5.0
TOP_BORDER_Y     = 9.0
BOTTOM_BORDER_Y  = 1.0
WORLD_X_MIN      = 0.0
WORLD_X_MAX      = 10.0
WALL_X           = [0.0, 2.0, 4.0, 6.0, 8.0, 10.0]

# Visual heights (Z axis — gives depth in RViz)
SHELF_HEIGHT     = 1.8   # tall bookshelf
SHELF_THICKNESS  = 0.08
TRASH_HEIGHT     = 0.15  # trash sits on the floor
ROBOT_HEIGHT     = 0.3
FLOOR_Z          = 0.0

# How thick the horizontal border walls look
BORDER_THICKNESS = 0.08

FRAME_ID = "map"


def rgba(r, g, b, a=1.0):
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c


def point(x, y, z=0.0):
    p = Point()
    p.x, p.y, p.z = float(x), float(y), float(z)
    return p


def scale(x, y, z):
    v = Vector3()
    v.x, v.y, v.z = float(x), float(y), float(z)
    return v


def base_marker(ns, uid, mtype, frame=FRAME_ID):
    m = Marker()
    m.header.frame_id = frame
    m.ns              = ns
    m.id              = uid
    m.type            = mtype
    m.action          = Marker.ADD
    m.pose.orientation.w = 1.0   # no rotation
    return m


class RvizPublisherNode(Node):
    def __init__(self):
        super().__init__('rviz_publisher_node')

        self.walls_data   = None
        self.trash_list   = []
        self.spawn_points = []
        self.robot_pose   = None
        self.prev_trash_ids = set()

        # Subscribers
        self.create_subscription(String,  '/world/walls',        self.walls_cb,  1)
        self.create_subscription(String,  '/world/trash',        self.trash_cb, 10)
        self.create_subscription(String,  '/world/spawn_points', self.spawn_cb,  1)
        self.create_subscription(Pose2D,  '/robot/pose',         self.robot_cb, 10)

        # Publishers
        self.pub_walls   = self.create_publisher(MarkerArray, '/viz/walls',        1)
        self.pub_trash   = self.create_publisher(MarkerArray, '/viz/trash',       10)
        self.pub_spawns  = self.create_publisher(MarkerArray, '/viz/spawn_points', 1)
        self.pub_robot   = self.create_publisher(MarkerArray, '/viz/robot',       10)
        self.pub_patrol  = self.create_publisher(MarkerArray, '/viz/patrol_line',  1)
        self.pub_floor   = self.create_publisher(MarkerArray, '/viz/floor',        1)

        self.create_timer(0.05, self.publish_all)  
        self.get_logger().info("RvizPublisherNode ready.")

    # ── Callbacks ────────────────────────────────────────────────────────────
    def walls_cb(self, msg):
        self.walls_data = json.loads(msg.data)

    def trash_cb(self, msg):
        self.trash_list = [tuple(t) for t in json.loads(msg.data)]

    def spawn_cb(self, msg):
        self.spawn_points = [tuple(p) for p in json.loads(msg.data)]

    def robot_cb(self, msg):
        self.robot_pose = msg

    # ── Publishers ────────────────────────────────────────────────────────────
    def publish_all(self):
        now = self.get_clock().now().to_msg()
        self._publish_floor(now)
        self._publish_walls(now)
        self._publish_trash(now)
        self._publish_spawns(now)
        self._publish_robot(now)
        self._publish_patrol(now)

    # ── Floor ─────────────────────────────────────────────────────────────────
    def _publish_floor(self, now):
        m = base_marker("floor", 0, Marker.CUBE)
        m.header.stamp    = now
        m.pose.position   = point(5.0, 5.0, -0.05)
        m.scale           = scale(10.0, 10.0, 0.02)
        m.color           = rgba(0.15, 0.15, 0.18, 1.0)
        arr = MarkerArray(); arr.markers.append(m)
        self.pub_floor.publish(arr)

    # ── Walls ─────────────────────────────────────────────────────────────────
    def _publish_walls(self, now):
        if self.walls_data is None:
            return

        markers = []
        uid = 0
        half_h = SHELF_HEIGHT / 2.0

        top_open    = self.walls_data['top_section'][0]     # 5.5
        bot_open    = self.walls_data['bottom_section'][1]  # 4.5

        # ── Vertical walls (shelf dividers) ──────────────────────────────────
        for wx in WALL_X:
            for (y_min, y_max, label) in [
                (BOTTOM_BORDER_Y, bot_open,  "bot"),
                (top_open,        TOP_BORDER_Y, "top"),
            ]:
                length = y_max - y_min
                m = base_marker("walls_vertical", uid, Marker.CUBE)
                m.header.stamp  = now
                m.pose.position = point(wx, (y_min + y_max) / 2.0, half_h)
                m.scale         = scale(SHELF_THICKNESS, length, SHELF_HEIGHT)
                m.color         = rgba(0.55, 0.35, 0.10, 1.0)   # warm wood
                markers.append(m)
                uid += 1

        # ── Horizontal border walls (top and bottom) ──────────────────────────
        width = WORLD_X_MAX - WORLD_X_MIN
        for wy in [TOP_BORDER_Y, BOTTOM_BORDER_Y]:
            m = base_marker("walls_horizontal", uid, Marker.CUBE)
            m.header.stamp  = now
            m.pose.position = point((WORLD_X_MIN + WORLD_X_MAX) / 2.0, wy, half_h)
            m.scale         = scale(width, BORDER_THICKNESS, SHELF_HEIGHT)
            m.color         = rgba(0.55, 0.35, 0.10, 1.0)
            markers.append(m)
            uid += 1

        arr = MarkerArray(); arr.markers = markers
        self.pub_walls.publish(arr)

    # ── Trash ─────────────────────────────────────────────────────────────────
    def _publish_trash(self, now):
        markers = []
        active_ids = set()

        for uid, (tx, ty) in enumerate(self.trash_list):
            active_ids.add(uid)

            # Cylinder body
            m = base_marker("trash", uid, Marker.CYLINDER)
            m.header.stamp  = now
            m.pose.position = point(tx, ty, TRASH_HEIGHT / 2.0)
            m.scale         = scale(0.25, 0.25, TRASH_HEIGHT)
            m.color         = rgba(0.0, 0.85, 0.3, 1.0)
            markers.append(m)

            # Sphere on top
            s = base_marker("trash_top", uid, Marker.SPHERE)
            s.header.stamp  = now
            s.pose.position = point(tx, ty, TRASH_HEIGHT + 0.08)
            s.scale         = scale(0.18, 0.18, 0.18)
            s.color         = rgba(0.0, 1.0, 0.5, 1.0)
            markers.append(s)

        # Explicitly DELETE ids that disappeared since last frame
        for old_id in self.prev_trash_ids - active_ids:
            for ns in ["trash", "trash_top"]:
                d = Marker()
                d.header.frame_id = FRAME_ID
                d.header.stamp    = now
                d.ns              = ns
                d.id              = old_id
                d.action          = Marker.DELETE
                markers.append(d)

        self.prev_trash_ids = active_ids

        if markers:
            arr = MarkerArray()
            arr.markers = markers
            self.pub_trash.publish(arr)

    # ── Spawn points ──────────────────────────────────────────────────────────
    def _publish_spawns(self, now):
        markers = []
        for uid, (px, py) in enumerate(self.spawn_points):
            m = base_marker("spawns", uid, Marker.CYLINDER)
            m.header.stamp  = now
            m.pose.position = point(px, py, 0.01)
            m.scale         = scale(0.12, 0.12, 0.02)
            m.color         = rgba(0.4, 0.4, 0.5, 0.6)   # subtle grey, semi-transparent
            markers.append(m)
        arr = MarkerArray(); arr.markers = markers
        self.pub_spawns.publish(arr)

    # ── Robot ─────────────────────────────────────────────────────────────────
    def _publish_robot(self, now):
        if self.robot_pose is None:
            return

        markers = []
        rx, ry = self.robot_pose.x, self.robot_pose.y
        direction = self.robot_pose.theta   # +1 or -1 (patrol direction)

        # Robot body — cylinder
        body = base_marker("robot_body", 0, Marker.CYLINDER)
        body.header.stamp  = now
        body.pose.position = point(rx, ry, ROBOT_HEIGHT / 2.0)
        body.scale         = scale(0.35, 0.35, ROBOT_HEIGHT)
        body.color         = rgba(0.15, 0.5, 1.0, 1.0)   # bright blue
        markers.append(body)

        # Robot top dome — sphere
        dome = base_marker("robot_dome", 1, Marker.SPHERE)
        dome.header.stamp  = now
        dome.pose.position = point(rx, ry, ROBOT_HEIGHT + 0.12)
        dome.scale         = scale(0.28, 0.28, 0.22)
        dome.color         = rgba(0.4, 0.75, 1.0, 1.0)
        markers.append(dome)

        # Direction arrow
        arrow = base_marker("robot_arrow", 2, Marker.ARROW)
        arrow.header.stamp = now
        arrow.points       = [
            point(rx, ry, ROBOT_HEIGHT + 0.05),
            point(rx + direction * 0.35, ry, ROBOT_HEIGHT + 0.05),
        ]
        arrow.scale        = scale(0.06, 0.12, 0.12)   # shaft_d, head_d, head_len
        arrow.color        = rgba(1.0, 1.0, 0.2, 1.0)  # yellow arrow
        markers.append(arrow)

        arr = MarkerArray(); arr.markers = markers
        self.pub_robot.publish(arr)

    # ── Patrol line ───────────────────────────────────────────────────────────
    def _publish_patrol(self, now):
        line = base_marker("patrol", 0, Marker.LINE_STRIP)
        line.header.stamp = now
        line.points = [
            point(WORLD_X_MIN, PATROL_Y, 0.02),
            point(WORLD_X_MAX, PATROL_Y, 0.02),
        ]
        line.scale.x = 0.04   # line width
        line.color   = rgba(0.9, 0.9, 0.2, 0.6)   # dashed yellow, semi-transparent
        arr = MarkerArray(); arr.markers = [line]
        self.pub_patrol.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = RvizPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()