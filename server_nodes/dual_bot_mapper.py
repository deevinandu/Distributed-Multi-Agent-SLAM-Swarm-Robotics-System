"""
Dual-Bot Mission Control Server
================================
- PyGame real-time visualization (no matplotlib)
- Server-side heartbeat with automatic zone lifting
- Occupancy-grid mapping with Bresenham ray-casting
- Pose-graph SLAM with landmark-based loop closure
- Frontier detection, clustering, and dynamic target assignment
- Territory zone sharing between bots

Usage:
    python dual_bot_mapper.py --separation 0.5

    --separation: Initial distance (meters) between the two bots along X-axis.
                  Bot1 starts at (0,0), Bot2 starts at (separation, 0).
"""
import socket
import struct
import math
import os
import sys
import time
import datetime
import csv
import argparse
import numpy as np

# -- Try importing pygame -------------------------------------------------------
try:
    import pygame
    from pygame import gfxdraw
except ImportError:
    print("[ERROR] pygame not installed. Run:  pip install pygame")
    sys.exit(1)

# ==============================================================================
#  PROTOCOL DEFINITIONS
# ==============================================================================

# QuasarPacket v2 from bots  (added landmark_type at the end)
PACKET_FMT  = '<4sBfffiIffffB'
PACKET_SIZE = struct.calcsize(PACKET_FMT)

# Fallback: old QuasarPacket v1 (without landmark_type) for backward compat
PACKET_FMT_V1  = '<4sBfffiIffff'
PACKET_SIZE_V1 = struct.calcsize(PACKET_FMT_V1)

# ZonePacket sent TO bots (territory avoidance)
ZONE_FMT  = '<4sffff'
ZONE_SIZE = struct.calcsize(ZONE_FMT)

# TargetPacket sent TO bots (frontier waypoint)
TARGET_FMT  = '<4sff'
TARGET_SIZE = struct.calcsize(TARGET_FMT)

# -- Trust filter ---------------------------------------------------------------
MAX_DIST_M = 1.20
MIN_DIST_M = 0.05

# -- Sensor ray angles relative to robot forward -------------------------------
SENSOR_ANGLES_RAD = {
    'front': 0.0,
    'left':  math.pi / 2,
    'back':  math.pi,
    'right': -math.pi / 2,
}

# -- Landmark types (must match firmware) --------------------------------------
LM_NONE     = 0
LM_CORNER_L = 1
LM_CORNER_R = 2
LM_CORRIDOR = 3
LM_DEAD_END = 4
LM_OPEN     = 5

LANDMARK_NAMES = {
    LM_NONE: "NONE", LM_CORNER_L: "CORNER_L", LM_CORNER_R: "CORNER_R",
    LM_CORRIDOR: "CORRIDOR", LM_DEAD_END: "DEAD_END", LM_OPEN: "OPEN",
}

# -- Timing constants ----------------------------------------------------------
HEARTBEAT_TIMEOUT    = 5.0    # seconds before declaring bot offline
ZONE_UPDATE_INTERVAL = 2.0    # seconds between zone broadcasts
TARGET_INTERVAL      = 3.0    # seconds between frontier target broadcasts

# -- Occupancy grid ------------------------------------------------------------
GRID_RESOLUTION = 0.05        # meters per cell (5cm)
GRID_SIZE       = 200         # 200x200 cells = 10m x 10m
GRID_ORIGIN_X   = -5.0        # world X of grid cell (0,0)
GRID_ORIGIN_Y   = -5.0        # world Y of grid cell (0,0)

CELL_UNKNOWN  = -1
CELL_FREE     =  0
CELL_OCCUPIED = 100

# -- SLAM constants ------------------------------------------------------------
CLOSURE_RADIUS     = 0.60     # meters -- max distance for loop closure match
MIN_POSES_BETWEEN  = 30       # minimum poses between closures to avoid trivial
CLOSURE_CORRECTION = 0.5      # fraction of error to correct (damping)

# -- Frontier constants --------------------------------------------------------
FRONTIER_MIN_CLUSTER = 3      # minimum cells in a frontier cluster
FRONTIER_SEPARATION  = 1.0    # minimum distance between assigned frontiers (m)


# ==============================================================================
#  OCCUPANCY GRID
# ==============================================================================

class OccupancyGrid:
    """2D occupancy grid for mapping."""

    def __init__(self, size=GRID_SIZE, resolution=GRID_RESOLUTION,
                 origin_x=GRID_ORIGIN_X, origin_y=GRID_ORIGIN_Y):
        self.size = size
        self.res  = resolution
        self.ox   = origin_x
        self.oy   = origin_y
        self.grid = np.full((size, size), CELL_UNKNOWN, dtype=np.int8)

    def world_to_grid(self, wx, wy):
        """Convert world coordinates to grid indices."""
        gx = int((wx - self.ox) / self.res)
        gy = int((wy - self.oy) / self.res)
        return gx, gy

    def grid_to_world(self, gx, gy):
        """Convert grid indices to world coordinates (cell center)."""
        wx = self.ox + (gx + 0.5) * self.res
        wy = self.oy + (gy + 0.5) * self.res
        return wx, wy

    def in_bounds(self, gx, gy):
        return 0 <= gx < self.size and 0 <= gy < self.size

    def update_ray(self, robot_x, robot_y, hit_x, hit_y, hit_valid):
        """
        Cast a ray from robot to hit point.
        Mark cells along the ray as FREE, endpoint as OCCUPIED (if valid hit).
        Uses Bresenham's line algorithm.
        """
        x0, y0 = self.world_to_grid(robot_x, robot_y)
        x1, y1 = self.world_to_grid(hit_x, hit_y)

        cells = self._bresenham(x0, y0, x1, y1)

        # All cells except the last are FREE
        for gx, gy in cells[:-1]:
            if self.in_bounds(gx, gy):
                self.grid[gy, gx] = CELL_FREE

        # Last cell is OCCUPIED if valid hit
        if cells and hit_valid:
            gx, gy = cells[-1]
            if self.in_bounds(gx, gy):
                self.grid[gy, gx] = CELL_OCCUPIED

    def _bresenham(self, x0, y0, x1, y1):
        """Bresenham's line algorithm returning list of (x, y) cells."""
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            cells.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return cells

    def get_frontiers(self):
        """
        Find frontier cells: FREE cells adjacent to at least one UNKNOWN cell.
        Returns list of (gx, gy) grid coordinates.
        """
        frontiers = []
        for y in range(1, self.size - 1):
            for x in range(1, self.size - 1):
                if self.grid[y, x] != CELL_FREE:
                    continue
                # Check 4-connected neighbors for UNKNOWN
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    if self.grid[y + dy, x + dx] == CELL_UNKNOWN:
                        frontiers.append((x, y))
                        break
        return frontiers

    def cluster_frontiers(self, frontier_cells):
        """
        Cluster adjacent frontier cells using flood-fill.
        Returns list of clusters, each cluster is a list of (gx, gy).
        """
        if not frontier_cells:
            return []

        cell_set = set(frontier_cells)
        visited = set()
        clusters = []

        for cell in frontier_cells:
            if cell in visited:
                continue
            # BFS flood fill
            cluster = []
            queue = [cell]
            while queue:
                c = queue.pop(0)
                if c in visited:
                    continue
                visited.add(c)
                cluster.append(c)
                cx, cy = c
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nb = (cx + dx, cy + dy)
                    if nb in cell_set and nb not in visited:
                        queue.append(nb)

            if len(cluster) >= FRONTIER_MIN_CLUSTER:
                clusters.append(cluster)

        return clusters

    def cluster_centroid_world(self, cluster):
        """Compute the world-coordinate centroid of a frontier cluster."""
        avg_x = sum(c[0] for c in cluster) / len(cluster)
        avg_y = sum(c[1] for c in cluster) / len(cluster)
        return self.grid_to_world(avg_x, avg_y)


# ==============================================================================
#  POSE GRAPH SLAM
# ==============================================================================

class PoseNode:
    """A single pose in the pose graph."""
    __slots__ = ('x', 'y', 'yaw', 'agent_id', 'landmark_type', 'timestamp', 'index',
                 'corrected_x', 'corrected_y')

    def __init__(self, x, y, yaw, agent_id, landmark_type, timestamp, index):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.agent_id = agent_id
        self.landmark_type = landmark_type
        self.timestamp = timestamp
        self.index = index
        self.corrected_x = x
        self.corrected_y = y


class PoseGraphSLAM:
    """
    Lightweight pose-graph SLAM with landmark-based loop closure.
    Runs on the server, not on the ESP32.
    """

    def __init__(self):
        self.nodes = []          # list of PoseNode
        self.landmarks = []      # list of (x, y, landmark_type, node_index)
        self.closures = []       # list of (node_i, node_j, dx, dy) loop closure edges
        self.last_closure_idx = {1: -MIN_POSES_BETWEEN, 2: -MIN_POSES_BETWEEN}

    def add_pose(self, x, y, yaw, agent_id, landmark_type, timestamp):
        """Add a new pose node. Returns (closure_detected, correction_dx, correction_dy)."""
        idx = len(self.nodes)
        node = PoseNode(x, y, yaw, agent_id, landmark_type, timestamp, idx)
        self.nodes.append(node)

        closure_detected = False
        corr_dx, corr_dy = 0.0, 0.0

        # Store landmark if meaningful
        if landmark_type != LM_NONE:
            # Check for loop closure against existing landmarks
            closure_detected, corr_dx, corr_dy = self._check_closure(node)

            # Store this landmark for future matching
            self.landmarks.append((x, y, landmark_type, idx))

        return closure_detected, corr_dx, corr_dy

    def _check_closure(self, node):
        """Check if this node's landmark matches a previously seen landmark."""
        for lm_x, lm_y, lm_type, lm_idx in self.landmarks:
            # Must be same landmark type
            if lm_type != node.landmark_type:
                continue

            # Must be far enough in the pose sequence (avoid trivial matches)
            if node.index - lm_idx < MIN_POSES_BETWEEN:
                continue

            # Must not have had a recent closure for this agent
            if node.index - self.last_closure_idx.get(node.agent_id, -999) < MIN_POSES_BETWEEN:
                continue

            # Check spatial proximity
            dist = math.sqrt((node.x - lm_x)**2 + (node.y - lm_y)**2)
            if dist < CLOSURE_RADIUS:
                # Loop closure detected!
                error_x = lm_x - node.x
                error_y = lm_y - node.y

                corr_dx = error_x * CLOSURE_CORRECTION
                corr_dy = error_y * CLOSURE_CORRECTION

                self.closures.append((lm_idx, node.index, corr_dx, corr_dy))
                self.last_closure_idx[node.agent_id] = node.index

                print(f"[SLAM] LOOP CLOSURE! Agent {node.agent_id} | "
                      f"Landmark {LANDMARK_NAMES.get(node.landmark_type, '?')} | "
                      f"Dist: {dist:.2f}m | Correction: ({corr_dx:.3f}, {corr_dy:.3f})")

                return True, corr_dx, corr_dy

        return False, 0.0, 0.0

    def get_correction_for_agent(self, agent_id):
        """
        Get the cumulative drift correction for an agent based on all closures.
        Returns (dx, dy) to add to the agent's odometry.
        """
        total_dx, total_dy = 0.0, 0.0
        for lm_idx, node_idx, cdx, cdy in self.closures:
            if self.nodes[node_idx].agent_id == agent_id:
                total_dx += cdx
                total_dy += cdy
        return total_dx, total_dy


# ==============================================================================
#  PYGAME RENDERER
# ==============================================================================

# Colors (dark theme)
BG_COLOR       = (22, 33, 62)
GRID_COLOR     = (40, 50, 80)
TEXT_COLOR      = (220, 220, 220)
TEXT_DIM        = (120, 130, 150)

BOT_COLORS = {
    1: {
        'main':  (0, 191, 255),   # Deep Sky Blue
        'path':  (0, 120, 180),
        'front': (255, 68, 68),
        'left':  (68, 255, 68),
        'back':  (255, 136, 0),
        'right': (68, 136, 255),
        'name':  'Bot1',
    },
    2: {
        'main':  (255, 105, 180),  # Hot Pink
        'path':  (180, 60, 120),
        'front': (204, 0, 0),
        'left':  (0, 204, 0),
        'back':  (204, 102, 0),
        'right': (0, 68, 204),
        'name':  'Bot2',
    },
}

# Grid cell colors for the occupancy grid overlay
CELL_COLOR_FREE     = (30, 45, 70)
CELL_COLOR_OCCUPIED = (200, 200, 200)
CELL_COLOR_FRONTIER = (255, 255, 0)

OFFLINE_COLOR = (100, 100, 100)


class MapRenderer:
    """PyGame-based real-time map renderer."""

    def __init__(self, width=1000, height=800):
        pygame.init()
        pygame.display.set_caption("Dual-Bot Mission Control")
        self.width  = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        self.clock  = pygame.time.Clock()
        self.font_sm = pygame.font.SysFont("consolas", 12)
        self.font_md = pygame.font.SysFont("consolas", 14)
        self.font_lg = pygame.font.SysFont("consolas", 18, bold=True)

        # View transform: world coords -> screen coords
        self.scale    = 100.0   # pixels per meter (zoom level)
        self.offset_x = width / 2
        self.offset_y = height / 2

        # Dragging
        self._dragging = False
        self._drag_start = (0, 0)
        self._drag_offset_start = (0, 0)

    def world_to_screen(self, wx, wy):
        """Convert world (meters) to screen (pixels)."""
        sx = int(self.offset_x + wx * self.scale)
        sy = int(self.offset_y - wy * self.scale)  # Y is flipped
        return sx, sy

    def handle_events(self):
        """Handle PyGame events. Returns False if window closed."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            elif event.type == pygame.MOUSEWHEEL:
                # Zoom
                factor = 1.15 if event.y > 0 else 1 / 1.15
                self.scale *= factor
                self.scale = max(20, min(500, self.scale))
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                self._dragging = True
                self._drag_start = event.pos
                self._drag_offset_start = (self.offset_x, self.offset_y)
            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                self._dragging = False
            elif event.type == pygame.MOUSEMOTION and self._dragging:
                dx = event.pos[0] - self._drag_start[0]
                dy = event.pos[1] - self._drag_start[1]
                self.offset_x = self._drag_offset_start[0] + dx
                self.offset_y = self._drag_offset_start[1] + dy
        return True

    def render(self, occ_grid, bot_states, point_clouds, paths, zone_boxes,
               frontier_centroids, target_assignments, slam_closures, pkt_counts):
        """Full frame render."""
        self.screen.fill(BG_COLOR)

        # Draw grid lines
        self._draw_grid()

        # Draw occupancy grid overlay
        self._draw_occupancy(occ_grid)

        # Draw frontier centroids
        self._draw_frontiers(frontier_centroids)

        # Draw territory zone boxes
        self._draw_zones(zone_boxes)

        # Draw point clouds per bot
        for bot_id in [1, 2]:
            self._draw_point_cloud(bot_id, point_clouds.get(bot_id, {}))

        # Draw paths
        for bot_id in [1, 2]:
            self._draw_path(bot_id, paths.get(bot_id, ([], [])))

        # Draw robot markers
        for bot_id in [1, 2]:
            state = bot_states.get(bot_id)
            if state:
                self._draw_robot(bot_id, state)

        # Draw target assignment lines
        self._draw_targets(bot_states, target_assignments)

        # Draw loop closure lines
        self._draw_closures(slam_closures)

        # Draw HUD overlay
        self._draw_hud(bot_states, pkt_counts)

        pygame.display.flip()
        self.clock.tick(30)

    def _draw_grid(self):
        """Draw faint meter-grid lines."""
        # Determine visible world bounds
        for world_val in range(-20, 21):
            sx, sy = self.world_to_screen(world_val, 0)
            if 0 <= sx < self.width:
                pygame.draw.line(self.screen, GRID_COLOR, (sx, 0), (sx, self.height), 1)
            sx, sy = self.world_to_screen(0, world_val)
            if 0 <= sy < self.height:
                pygame.draw.line(self.screen, GRID_COLOR, (0, sy), (self.width, sy), 1)

        # Origin crosshair
        ox, oy = self.world_to_screen(0, 0)
        pygame.draw.line(self.screen, (60, 80, 120), (ox - 10, oy), (ox + 10, oy), 2)
        pygame.draw.line(self.screen, (60, 80, 120), (ox, oy - 10), (ox, oy + 10), 2)

    def _draw_occupancy(self, occ_grid):
        """Draw the occupancy grid as small colored cells."""
        cell_px = max(1, int(occ_grid.res * self.scale))
        if cell_px < 2:
            return  # Too zoomed out, skip for performance

        # Only draw cells that are visible and not UNKNOWN
        # Compute visible world bounds
        world_left   = -self.offset_x / self.scale
        world_right  = (self.width - self.offset_x) / self.scale
        world_top    = self.offset_y / self.scale
        world_bottom = -(self.height - self.offset_y) / self.scale

        gx_min = max(0, int((world_left - occ_grid.ox) / occ_grid.res) - 1)
        gx_max = min(occ_grid.size, int((world_right - occ_grid.ox) / occ_grid.res) + 1)
        gy_min = max(0, int((world_bottom - occ_grid.oy) / occ_grid.res) - 1)
        gy_max = min(occ_grid.size, int((world_top - occ_grid.oy) / occ_grid.res) + 1)

        for gy in range(gy_min, gy_max):
            for gx in range(gx_min, gx_max):
                val = occ_grid.grid[gy, gx]
                if val == CELL_UNKNOWN:
                    continue

                wx, wy = occ_grid.grid_to_world(gx, gy)
                sx, sy = self.world_to_screen(wx, wy)

                if val == CELL_OCCUPIED:
                    color = CELL_COLOR_OCCUPIED
                else:
                    color = CELL_COLOR_FREE

                if cell_px <= 2:
                    self.screen.set_at((sx, sy), color)
                else:
                    pygame.draw.rect(self.screen, color,
                                     (sx - cell_px // 2, sy - cell_px // 2, cell_px, cell_px))

    def _draw_frontiers(self, centroids):
        """Draw frontier cluster centroids as yellow diamonds."""
        for wx, wy in centroids:
            sx, sy = self.world_to_screen(wx, wy)
            pts = [(sx, sy - 6), (sx + 6, sy), (sx, sy + 6), (sx - 6, sy)]
            pygame.draw.polygon(self.screen, CELL_COLOR_FRONTIER, pts)
            pygame.draw.polygon(self.screen, (180, 180, 0), pts, 1)

    def _draw_zones(self, zone_boxes):
        """Draw territory bounding boxes as translucent rectangles."""
        for bot_id, box in zone_boxes.items():
            if box is None:
                continue
            color = BOT_COLORS[bot_id]['main']
            sx1, sy1 = self.world_to_screen(box[0], box[3])
            sx2, sy2 = self.world_to_screen(box[2], box[1])
            w = sx2 - sx1
            h = sy2 - sy1
            if w > 0 and h > 0:
                surf = pygame.Surface((w, h), pygame.SRCALPHA)
                surf.fill((*color, 25))  # very translucent
                self.screen.blit(surf, (sx1, sy1))
                pygame.draw.rect(self.screen, color, (sx1, sy1, w, h), 1)

    def _draw_point_cloud(self, bot_id, cloud_dict):
        """Draw sensor point cloud dots."""
        for sensor_name, points in cloud_dict.items():
            if not points:
                continue
            color = BOT_COLORS[bot_id].get(sensor_name, (150, 150, 150))
            # Draw only the last N points for performance
            recent = points[-2000:]
            for wx, wy in recent:
                sx, sy = self.world_to_screen(wx, wy)
                if 0 <= sx < self.width and 0 <= sy < self.height:
                    self.screen.set_at((sx, sy), color)

    def _draw_path(self, bot_id, path):
        """Draw robot trajectory path."""
        px_list, py_list = path
        if len(px_list) < 2:
            return
        color = BOT_COLORS[bot_id]['path']
        # Downsample for performance
        step = max(1, len(px_list) // 500)
        points = []
        for i in range(0, len(px_list), step):
            sx, sy = self.world_to_screen(px_list[i], py_list[i])
            points.append((sx, sy))
        if len(points) >= 2:
            pygame.draw.lines(self.screen, color, False, points, 2)

    def _draw_robot(self, bot_id, state):
        """Draw robot as a directional triangle."""
        x, y, yaw, online = state['x'], state['y'], state['yaw'], state['online']
        sx, sy = self.world_to_screen(x, y)

        size = 10
        color = BOT_COLORS[bot_id]['main'] if online else OFFLINE_COLOR

        # Triangle pointing in yaw direction
        pts = []
        for angle_offset in [0, 2.4, -2.4]:  # front, back-left, back-right
            if angle_offset == 0:
                r = size * 1.5
            else:
                r = size
            a = yaw + angle_offset
            px = sx + int(r * math.cos(a))
            py = sy - int(r * math.sin(a))
            pts.append((px, py))

        pygame.draw.polygon(self.screen, color, pts)
        pygame.draw.polygon(self.screen, (255, 255, 255), pts, 1)

        # Label
        label = self.font_sm.render(BOT_COLORS[bot_id]['name'], True,
                                    color if online else OFFLINE_COLOR)
        self.screen.blit(label, (sx + 14, sy - 6))

    def _draw_targets(self, bot_states, assignments):
        """Draw lines from bots to their assigned frontier targets."""
        for bot_id, (tx, ty) in assignments.items():
            state = bot_states.get(bot_id)
            if not state or not state['online']:
                continue
            sx1, sy1 = self.world_to_screen(state['x'], state['y'])
            sx2, sy2 = self.world_to_screen(tx, ty)
            color = BOT_COLORS[bot_id]['main']
            pygame.draw.line(self.screen, (*color[:3],), (sx1, sy1), (sx2, sy2), 1)
            # Target marker
            pygame.draw.circle(self.screen, color, (sx2, sy2), 5, 2)

    def _draw_closures(self, closures):
        """Draw loop closure connections as green lines."""
        for (x1, y1, x2, y2) in closures:
            sx1, sy1 = self.world_to_screen(x1, y1)
            sx2, sy2 = self.world_to_screen(x2, y2)
            pygame.draw.line(self.screen, (0, 255, 100), (sx1, sy1), (sx2, sy2), 2)

    def _draw_hud(self, bot_states, pkt_counts):
        """Draw head-up display with stats."""
        y = 10
        title = self.font_lg.render("DUAL-BOT MISSION CONTROL", True, TEXT_COLOR)
        self.screen.blit(title, (10, y))
        y += 25

        for bot_id in [1, 2]:
            state = bot_states.get(bot_id)
            bc = BOT_COLORS[bot_id]
            pkts = pkt_counts.get(bot_id, 0)

            if state:
                status = "ONLINE" if state['online'] else "OFFLINE"
                status_color = bc['main'] if state['online'] else OFFLINE_COLOR
                line = f"{bc['name']}: {status} | Pkts:{pkts} | ({state['x']:.2f},{state['y']:.2f}) {math.degrees(state['yaw']):.0f}deg"
            else:
                status_color = OFFLINE_COLOR
                line = f"{bc['name']}: WAITING | Pkts:0"

            surf = self.font_md.render(line, True, status_color)
            self.screen.blit(surf, (10, y))
            y += 18

        # Controls hint
        hint = self.font_sm.render("Scroll=Zoom | Drag=Pan | Close=Exit", True, TEXT_DIM)
        self.screen.blit(hint, (10, self.height - 20))

    def cleanup(self):
        pygame.quit()


# ==============================================================================
#  NETWORK HELPERS
# ==============================================================================

def send_zone_to_bot(sock, bot_addr, zone_box):
    """Send a ZONE packet to a bot telling it about forbidden territory."""
    if bot_addr is None:
        return
    if zone_box is None:
        # Send "no zone" -- values that form an impossible box
        pkt = struct.pack(ZONE_FMT, b'ZONE', 999.0, 999.0, -999.0, -999.0)
    else:
        pkt = struct.pack(ZONE_FMT, b'ZONE',
                          zone_box[0], zone_box[1], zone_box[2], zone_box[3])
    try:
        sock.sendto(pkt, bot_addr)
    except Exception as e:
        print(f"[ZONE-TX] Error: {e}")


def send_target_to_bot(sock, bot_addr, target_x, target_y):
    """Send a TARGET packet to a bot with its assigned frontier waypoint."""
    if bot_addr is None:
        return
    pkt = struct.pack(TARGET_FMT, b'TARG', target_x, target_y)
    try:
        sock.sendto(pkt, bot_addr)
    except Exception as e:
        print(f"[TARGET-TX] Error: {e}")


def compute_bounding_box(points_x, points_y):
    """Compute axis-aligned bounding box from point lists."""
    if not points_x:
        return None
    return (min(points_x), min(points_y), max(points_x), max(points_y))


# ==============================================================================
#  MAIN
# ==============================================================================

def main():
    parser = argparse.ArgumentParser(description="Dual-Bot Mission Control Server")
    parser.add_argument('--separation', type=float, default=0.5,
                        help='Initial separation between bots in meters (default: 0.5)')
    parser.add_argument('--port', type=int, default=8888,
                        help='UDP port to listen on (default: 8888)')
    args = parser.parse_args()

    separation = args.separation

    # -- Logging setup ----------------------------------------------------------
    ts   = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    ldir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "logs", f"dual_session_{ts}")
    os.makedirs(ldir, exist_ok=True)

    f_telem  = open(os.path.join(ldir, "telemetry.csv"),  'w', newline='')
    f_points = open(os.path.join(ldir, "pointcloud.csv"), 'w', newline='')
    w_telem  = csv.writer(f_telem)
    w_points = csv.writer(f_points)
    w_telem.writerow(['time', 'agent', 'x', 'y', 'yaw_deg', 'encoder',
                      'v2v', 'front_cm', 'left_cm', 'back_cm', 'right_cm', 'landmark'])
    w_points.writerow(['time', 'agent', 'sensor', 'x', 'y'])

    print("=" * 70)
    print(f"  DUAL-BOT Mission Control  |  Port {args.port}")
    print(f"  Bot1 origin: (0, 0)   |  Bot2 origin: ({separation}, 0)")
    print(f"  Packet sizes: v2={PACKET_SIZE}B  v1={PACKET_SIZE_V1}B")
    print(f"  Logging -> {ldir}")
    print("=" * 70)

    # -- Socket -----------------------------------------------------------------
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(('0.0.0.0', args.port))
        print(f"[OK] Listening on 0.0.0.0:{args.port}")
    except Exception as e:
        print(f"[ERR] Cannot bind: {e}")
        return
    sock.setblocking(False)

    # -- State ------------------------------------------------------------------
    # Per-bot state
    bot_states = {}       # bot_id -> {x, y, yaw, online, ...}
    bot_addrs  = {1: None, 2: None}
    bot_ports  = {1: 8888, 2: 8889}
    last_packet_time = {1: 0.0, 2: 0.0}
    pkt_counts = {1: 0, 2: 0}

    # Point clouds per bot per sensor
    point_clouds = {
        1: {k: [] for k in SENSOR_ANGLES_RAD},
        2: {k: [] for k in SENSOR_ANGLES_RAD},
    }

    # Robot paths
    paths = {1: ([], []), 2: ([], [])}

    # Territory zones
    zone_boxes = {1: None, 2: None}

    # Frontier targets
    target_assignments = {}   # bot_id -> (target_x, target_y)
    frontier_centroids = []   # list of (wx, wy)

    # SLAM
    slam = PoseGraphSLAM()
    slam_closure_lines = []   # list of (x1, y1, x2, y2) for visualization
    drift_correction = {1: (0.0, 0.0), 2: (0.0, 0.0)}

    # Occupancy grid
    occ_grid = OccupancyGrid()

    # Timing
    last_zone_send   = time.time()
    last_target_send = time.time()

    # -- Renderer ---------------------------------------------------------------
    renderer = MapRenderer(1000, 800)

    # -- Main loop --------------------------------------------------------------
    running = True
    try:
        while running:
            # Handle PyGame events
            if not renderer.handle_events():
                break

            now = time.time()

            # -- Heartbeat check ------------------------------------------------
            for bot_id in [1, 2]:
                if bot_id in bot_states:
                    if last_packet_time[bot_id] > 0 and \
                       now - last_packet_time[bot_id] > HEARTBEAT_TIMEOUT:
                        if bot_states[bot_id]['online']:
                            bot_states[bot_id]['online'] = False
                            print(f"[HEARTBEAT] {BOT_COLORS[bot_id]['name']} is OFFLINE "
                                  f"(no data for {HEARTBEAT_TIMEOUT:.0f}s)")

            # -- Receive packets ------------------------------------------------
            packets_this_frame = 0
            while packets_this_frame < 20:  # process up to 20 packets per frame
                try:
                    data, addr = sock.recvfrom(65535)
                except BlockingIOError:
                    break

                packets_this_frame += 1

                print(f"[UDP-RX] Got {len(data)} bytes from {addr}")

                # Determine packet version
                landmark_type = LM_NONE
                if len(data) == PACKET_SIZE:
                    u = struct.unpack(PACKET_FMT, data)
                    magic, agent_id, rx, ry, ryaw, enc, v2v, \
                        d_front, d_left, d_back, d_right, landmark_type = u
                elif len(data) == PACKET_SIZE_V1:
                    u = struct.unpack(PACKET_FMT_V1, data)
                    magic, agent_id, rx, ry, ryaw, enc, v2v, \
                        d_front, d_left, d_back, d_right = u
                    landmark_type = LM_NONE
                else:
                    continue

                if magic != b'QSRL':
                    continue
                if agent_id not in [1, 2]:
                    continue

                # Store bot address
                bot_addrs[agent_id] = (addr[0], bot_ports[agent_id])
                last_packet_time[agent_id] = now
                pkt_counts[agent_id] += 1

                # Apply Bot2 offset
                if agent_id == 2:
                    rx += separation

                # Apply SLAM drift correction
                cdx, cdy = drift_correction[agent_id]
                rx += cdx
                ry += cdy

                # Update bot state
                bot_states[agent_id] = {
                    'x': rx, 'y': ry, 'yaw': ryaw,
                    'online': True,
                    'landmark': landmark_type,
                }

                # Log telemetry
                w_telem.writerow([
                    f"{now:.3f}", agent_id,
                    f"{rx:.4f}", f"{ry:.4f}",
                    f"{math.degrees(ryaw):.2f}", enc, v2v,
                    f"{d_front*100:.1f}", f"{d_left*100:.1f}",
                    f"{d_back*100:.1f}", f"{d_right*100:.1f}",
                    landmark_type,
                ])
                f_telem.flush()

                # Update path
                paths[agent_id][0].append(rx)
                paths[agent_id][1].append(ry)

                # Project sensor readings -> world coordinates + update grid
                sensors = {
                    'front': d_front, 'left': d_left,
                    'back': d_back, 'right': d_right,
                }
                for name, dist in sensors.items():
                    ray_angle = ryaw + SENSOR_ANGLES_RAD[name]
                    hit_valid = MIN_DIST_M < dist <= MAX_DIST_M
                    if hit_valid:
                        wx = rx + dist * math.cos(ray_angle)
                        wy = ry + dist * math.sin(ray_angle)
                        point_clouds[agent_id][name].append((wx, wy))
                        w_points.writerow([f"{now:.3f}", agent_id, name,
                                           f"{wx:.4f}", f"{wy:.4f}"])

                        # Update occupancy grid
                        occ_grid.update_ray(rx, ry, wx, wy, True)
                    else:
                        # Ray extends to max range -- mark cells as free along the way
                        max_range = min(dist, MAX_DIST_M) if dist > MIN_DIST_M else MAX_DIST_M
                        end_x = rx + max_range * math.cos(ray_angle)
                        end_y = ry + max_range * math.sin(ray_angle)
                        occ_grid.update_ray(rx, ry, end_x, end_y, False)

                f_points.flush()

                # -- SLAM: add pose and check loop closure ----------------------
                closure, cdx_new, cdy_new = slam.add_pose(
                    rx, ry, ryaw, agent_id, landmark_type, now)
                if closure:
                    drift_correction[agent_id] = (
                        drift_correction[agent_id][0] + cdx_new,
                        drift_correction[agent_id][1] + cdy_new,
                    )
                    # Store closure line for visualization
                    slam_closure_lines.append((
                        rx, ry,
                        rx + cdx_new, ry + cdy_new,
                    ))

            # -- Periodic: Zone + Frontier updates ------------------------------
            if now - last_zone_send > ZONE_UPDATE_INTERVAL:
                last_zone_send = now

                for bot_id in [1, 2]:
                    other_id = 2 if bot_id == 1 else 1

                    # Only compute zone if the OTHER bot is online
                    if bot_states.get(other_id, {}).get('online', False):
                        all_pts_x = sum((pts_list
                                         for pts_list in
                                         ([p[0] for p in pts]
                                          for pts in point_clouds[other_id].values())),
                                        []) + paths[other_id][0]
                        all_pts_y = sum((pts_list
                                         for pts_list in
                                         ([p[1] for p in pts]
                                          for pts in point_clouds[other_id].values())),
                                        []) + paths[other_id][1]
                        zone_boxes[other_id] = compute_bounding_box(all_pts_x, all_pts_y)
                        send_zone_to_bot(sock, bot_addrs[bot_id], zone_boxes[other_id])
                    else:
                        # Other bot is offline -- lift the zone
                        zone_boxes[other_id] = None
                        send_zone_to_bot(sock, bot_addrs[bot_id], None)

            # -- Periodic: Frontier detection and target assignment -------------
            if now - last_target_send > TARGET_INTERVAL:
                last_target_send = now

                frontier_cells = occ_grid.get_frontiers()
                clusters = occ_grid.cluster_frontiers(frontier_cells)

                # Compute centroids
                centroids = [occ_grid.cluster_centroid_world(c) for c in clusters]
                frontier_centroids = centroids

                # Assign frontiers to bots (greedy nearest, with separation)
                target_assignments = {}
                # used_centroids = set()

                # online_bots = [bid for bid in [1, 2]
                #                if bot_states.get(bid, {}).get('online', False)]

                # for bot_id in online_bots:
                #     bx = bot_states[bot_id]['x']
                #     by = bot_states[bot_id]['y']

                #     best_dist = float('inf')
                #     best_idx  = -1

                #     for i, (cx, cy) in enumerate(centroids):
                #         if i in used_centroids:
                #             continue

                #         # Check minimum separation from other assigned targets
                #         too_close = False
                #         for other_bid, (tx, ty) in target_assignments.items():
                #             if math.sqrt((cx - tx)**2 + (cy - ty)**2) < FRONTIER_SEPARATION:
                #                 too_close = True
                #                 break
                #         if too_close:
                #             continue

                #         dist = math.sqrt((bx - cx)**2 + (by - cy)**2)
                #         if dist < best_dist:
                #             best_dist = dist
                #             best_idx = i

                #     if best_idx >= 0:
                #         target_assignments[bot_id] = centroids[best_idx]
                #         used_centroids.add(best_idx)

                # Send targets to bots
                # for bot_id, (tx, ty) in target_assignments.items():
                #     send_target_to_bot(sock, bot_addrs[bot_id], tx, ty)

            # -- Render ---------------------------------------------------------
            renderer.render(
                occ_grid, bot_states, point_clouds, paths,
                zone_boxes, frontier_centroids, target_assignments,
                slam_closure_lines, pkt_counts)

    except KeyboardInterrupt:
        total = pkt_counts[1] + pkt_counts[2]
        print(f"\n[EXIT] {total} packets logged "
              f"(Bot1:{pkt_counts[1]}, Bot2:{pkt_counts[2]}). Saving...")

    finally:
        # Save merged point cloud
        all_x = sum(([p[0] for p in pts]
                      for cloud in point_clouds.values()
                      for pts in cloud.values()), [])
        all_y = sum(([p[1] for p in pts]
                      for cloud in point_clouds.values()
                      for pts in cloud.values()), [])
        if all_x:
            np.savetxt(os.path.join(ldir, "pointcloud_merged.csv"),
                       np.column_stack([all_x, all_y]),
                       delimiter=",", header="x,y", comments="")

        # Save per-bot point clouds
        for bot_id in [1, 2]:
            bx = sum(([p[0] for p in pts]
                       for pts in point_clouds[bot_id].values()), [])
            by = sum(([p[1] for p in pts]
                       for pts in point_clouds[bot_id].values()), [])
            if bx:
                np.savetxt(os.path.join(ldir, f"pointcloud_bot{bot_id}.csv"),
                           np.column_stack([bx, by]),
                           delimiter=",", header="x,y", comments="")

        # Save SLAM info
        with open(os.path.join(ldir, "slam_closures.csv"), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['node_i', 'node_j', 'corr_dx', 'corr_dy'])
            for lm_idx, node_idx, cdx, cdy in slam.closures:
                w.writerow([lm_idx, node_idx, f"{cdx:.4f}", f"{cdy:.4f}"])

        print(f"[SAVED] {ldir}")
        f_telem.close()
        f_points.close()
        sock.close()
        renderer.cleanup()


if __name__ == '__main__':
    main()
