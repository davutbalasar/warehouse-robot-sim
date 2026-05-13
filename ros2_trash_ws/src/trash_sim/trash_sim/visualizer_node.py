# visualizer_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
import json, sys, math

try:
    import pygame
except ImportError:
    print("Install pygame:  pip install pygame")
    sys.exit(1)

WIN   = 700
PAD   = 50
WORLD = 10.0
SCALE = (WIN - 2 * PAD) / WORLD   # pixels per world unit


def s(x, y):
    """World → screen (y-flip)."""
    return int(PAD + x * SCALE), int(WIN - PAD - y * SCALE)


def sr(v):
    """World length → pixel length."""
    return max(1, int(v * SCALE))


# ── Colours ──────────────────────────────────────────────────────────────────
BG        = ( 20,  20,  25)
WALL_COL  = (180, 140,  80)   # warm wood colour
PATROL_C  = (200, 200,  50)
SPAWN_C   = ( 70,  70,  80)
TRASH_C   = (  0, 210,  80)
ROBOT_C   = ( 50, 130, 255)
TEXT_C    = (220, 220, 220)
GRID_C    = ( 40,  40,  45)


class VisualizerNode(Node):
    def __init__(self, screen, font, sfont):
        super().__init__('visualizer_node')
        self.screen = screen
        self.font   = font
        self.sfont  = sfont

        self.walls_data   = None
        self.trash_list   = []
        self.spawn_points = []
        self.robot_pose   = None

        self.create_subscription(String, '/world/walls',        self.walls_cb,  1)
        self.create_subscription(String, '/world/trash',        self.trash_cb, 10)
        self.create_subscription(String, '/world/spawn_points', self.spawn_cb,  1)
        self.create_subscription(Pose2D, '/robot/pose',         self.robot_cb, 10)

        self.create_timer(0.033, self.draw)   # ~30 fps
        self.get_logger().info("VisualizerNode ready.")

    def walls_cb(self, msg):
        self.walls_data = json.loads(msg.data)

    def trash_cb(self, msg):
        self.trash_list = [tuple(t) for t in json.loads(msg.data)]

    def spawn_cb(self, msg):
        self.spawn_points = [tuple(p) for p in json.loads(msg.data)]

    def robot_cb(self, msg):
        self.robot_pose = msg

    # ─────────────────────────────────────────────────────────────────────────
    def draw(self):
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                pygame.quit(); rclpy.shutdown(); sys.exit(0)
            if ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:
                pygame.quit(); rclpy.shutdown(); sys.exit(0)

        self.screen.fill(BG)

        if self.walls_data is None:
            pygame.display.flip()
            return

        wd = self.walls_data
        patrol_y      = wd['patrol_y']
        dividers_x    = wd['dividers_x']
        outer_left    = wd['outer_left']
        outer_right   = wd['outer_right']
        top_border_y  = wd['horizontal'][0][2]   # y of top wall
        bot_border_y  = wd['horizontal'][1][2]   # y of bottom wall
        top_open_y    = wd['top_section'][0]     # open bottom of top section
        bot_open_y    = wd['bottom_section'][1]  # open top of bottom section

        wall_x_all    = [outer_left] + dividers_x + [outer_right]
        W = 3   # wall pixel thickness

        # ── Subtle background grid ────────────────────────────────────────────
        for gx in range(0, int(WORLD) + 1):
            ax, ay = s(gx, 0); bx, by = s(gx, WORLD)
            pygame.draw.line(self.screen, GRID_C, (ax, ay), (bx, by), 1)
        for gy in range(0, int(WORLD) + 1):
            ax, ay = s(0, gy); bx, by = s(WORLD, gy)
            pygame.draw.line(self.screen, GRID_C, (ax, ay), (bx, by), 1)

        # ── Patrol corridor dashed line ───────────────────────────────────────
        lx, ly = s(outer_left,  patrol_y)
        rx, ry = s(outer_right, patrol_y)
        dash = 14
        xx = lx
        while xx < rx:
            xe = min(xx + dash, rx)
            pygame.draw.line(self.screen, PATROL_C, (xx, ly), (xe, ly), 2)
            xx += dash * 2

        # ── Spawn points (ghost dots) ─────────────────────────────────────────
        for (px, py) in self.spawn_points:
            sx, sy = s(px, py)
            pygame.draw.circle(self.screen, SPAWN_C, (sx, sy), 5)

        # ── Trash ─────────────────────────────────────────────────────────────
        for (tx, ty) in self.trash_list:
            sx, sy = s(tx, ty)
            pygame.draw.circle(self.screen, TRASH_C, (sx, sy), 9)
            pygame.draw.circle(self.screen, (0, 255, 100), (sx, sy), 9, 2)

        # ── Vertical walls ────────────────────────────────────────────────────
        # Each vertical wall runs from bot_border_y to top_border_y
        # but is OPEN (not drawn) in the corridor gap between sections
        for wx in wall_x_all:
            # Bottom section: bot_border_y → bot_open_y
            ax, ay = s(wx, bot_border_y)
            bx, by = s(wx, bot_open_y)
            pygame.draw.line(self.screen, WALL_COL, (ax, ay), (bx, by), W)
            # Top section: top_open_y → top_border_y
            ax, ay = s(wx, top_open_y)
            bx, by = s(wx, top_border_y)
            pygame.draw.line(self.screen, WALL_COL, (ax, ay), (bx, by), W)

        # ── Horizontal border walls (full width) ──────────────────────────────
        for (x0, x1, wy) in wd['horizontal']:
            ax, ay = s(x0, wy)
            bx, by = s(x1, wy)
            pygame.draw.line(self.screen, WALL_COL, (ax, ay), (bx, by), W + 1)

        # ── Robot ─────────────────────────────────────────────────────────────
        if self.robot_pose:
            rx, ry = s(self.robot_pose.x, self.robot_pose.y)
            pygame.draw.circle(self.screen, ROBOT_C, (rx, ry), 11)
            pygame.draw.circle(self.screen, (150, 200, 255), (rx, ry), 11, 2)

            # Direction indicator
            direction = self.robot_pose.theta  # +1 or -1 stored in theta
            arw_len = 14
            pygame.draw.line(self.screen, (150, 200, 255),
                             (rx, ry),
                             (int(rx + direction * arw_len), ry), 3)

        # ── HUD ───────────────────────────────────────────────────────────────
        lines = [
            "Warehouse Robot Simulation",
        ]
        if self.robot_pose:
            lines.append(
                f"Robot  x={self.robot_pose.x:.2f}  y={self.robot_pose.y:.2f}"
                f"   trash={len(self.trash_list)}"
            )

        for i, line in enumerate(lines):
            color = (255, 255, 255) if i == 0 else TEXT_C
            fnt   = self.font if i == 0 else self.sfont
            surf  = fnt.render(line, True, color)
            self.screen.blit(surf, (PAD, 8 + i * 20))

        # ── Legend ────────────────────────────────────────────────────────────
        legend = [
            (WALL_COL,  "Shelf wall"),
            (PATROL_C,  "Patrol corridor"),
            (TRASH_C,   "Trash"),
            (ROBOT_C,   "Robot"),
            (SPAWN_C,   "Spawn point"),
        ]
        lx2, ly2 = WIN - 165, PAD
        for col, label in legend:
            pygame.draw.circle(self.screen, col, (lx2, ly2), 6)
            surf = self.sfont.render(label, True, TEXT_C)
            self.screen.blit(surf, (lx2 + 12, ly2 - 8))
            ly2 += 22

        pygame.display.flip()


def main(args=None):
    pygame.init()
    screen = pygame.display.set_mode((WIN, WIN))
    pygame.display.set_caption("Warehouse Robot Simulation")
    font  = pygame.font.SysFont("monospace", 15, bold=True)
    sfont = pygame.font.SysFont("monospace", 13)

    rclpy.init(args=args)
    node = VisualizerNode(screen, font, sfont)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    pygame.quit()


if __name__ == '__main__':
    main()