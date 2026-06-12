"""
Dual-Bot Session Playback Viewer
==================================
Reads telemetry.csv and pointcloud.csv from a dual-bot session
and replays them as a top-down 2D map using PyGame.

Usage:
  python playback_dual_session.py <session_folder>

Example:
  python playback_dual_session.py server_nodes/logs/dual_session_20260611_062145

Controls:
  Mouse scroll = Zoom
  Mouse drag   = Pan
  SPACE        = Pause/Resume
  +/-          = Speed up/down
  R            = Reset view
  ESC          = Quit
"""
import os
import sys
import csv
import math
import time

try:
    import pygame
    from pygame import gfxdraw
except ImportError:
    print("[ERROR] pygame not installed. Run:  pip install pygame")
    sys.exit(1)

# ==============================================================================
#  COLORS  (dark theme matching dual_bot_mapper.py)
# ==============================================================================
BG_COLOR   = (22, 33, 62)
GRID_COLOR = (40, 50, 80)
TEXT_COLOR  = (220, 220, 220)
TEXT_DIM    = (120, 130, 150)

BOT_COLORS = {
    1: {
        'main':  (0, 191, 255),
        'path':  (0, 120, 180),
        'cloud': (0, 191, 255),
        'name':  'Bot1',
    },
    2: {
        'main':  (255, 105, 180),
        'path':  (180, 60, 120),
        'cloud': (255, 105, 180),
        'name':  'Bot2',
    },
}


def load_session(folder):
    """Load telemetry and pointcloud CSVs."""
    telem_path = os.path.join(folder, "telemetry.csv")
    cloud_path = os.path.join(folder, "pointcloud.csv")

    if not os.path.exists(telem_path):
        print(f"[ERROR] {telem_path} not found!")
        sys.exit(1)

    # Load telemetry
    telemetry = []
    with open(telem_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            telemetry.append({
                'time':   float(row['time']),
                'agent':  int(row['agent']),
                'x':      float(row['x']),
                'y':      float(row['y']),
                'yaw':    math.radians(float(row['yaw_deg'])),
                'enc':    int(row['encoder']),
                'v2v':    int(row['v2v']),
                'front':  float(row['front_cm']),
                'left':   float(row['left_cm']),
                'back':   float(row['back_cm']),
                'right':  float(row['right_cm']),
                'lm':     int(row['landmark']),
            })

    # Load point cloud
    pointcloud = []
    if os.path.exists(cloud_path):
        with open(cloud_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                pointcloud.append({
                    'time':   float(row['time']),
                    'agent':  int(row['agent']),
                    'sensor': row['sensor'],
                    'x':      float(row['x']),
                    'y':      float(row['y']),
                })

    # Sort by time
    telemetry.sort(key=lambda r: r['time'])
    pointcloud.sort(key=lambda r: r['time'])

    return telemetry, pointcloud


def main():
    if len(sys.argv) < 2:
        print("Usage: python playback_dual_session.py <session_folder>")
        print("Example: python playback_dual_session.py server_nodes/logs/dual_session_20260611_062145")
        sys.exit(1)

    folder = sys.argv[1]
    print(f"Loading session from: {folder}")
    telemetry, pointcloud = load_session(folder)
    print(f"  Telemetry: {len(telemetry)} rows")
    print(f"  Pointcloud: {len(pointcloud)} rows")

    if not telemetry:
        print("[ERROR] No telemetry data!")
        sys.exit(1)

    # --- PyGame setup ---
    WIDTH, HEIGHT = 1000, 800
    pygame.init()
    pygame.display.set_caption("Dual-Bot Session Playback")
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    clock = pygame.time.Clock()
    font_sm = pygame.font.SysFont("consolas", 12)
    font_md = pygame.font.SysFont("consolas", 14)
    font_lg = pygame.font.SysFont("consolas", 18, bold=True)

    # View transform
    scale = 100.0
    offset_x = WIDTH / 2 - 2.5 * scale  # center on room
    offset_y = HEIGHT / 2
    dragging = False
    drag_start = (0, 0)
    drag_offset_start = (0, 0)

    def world_to_screen(wx, wy):
        sx = int(offset_x + wx * scale)
        sy = int(offset_y - wy * scale)
        return sx, sy

    # Playback state
    paused = False
    speed = 1.0
    telem_idx = 0
    cloud_idx = 0

    # Accumulated data
    paths = {1: [], 2: []}
    clouds = {1: [], 2: []}
    bot_state = {}
    pkt_counts = {1: 0, 2: 0}

    session_start = telemetry[0]['time']
    playback_start = time.time()

    running = True
    while running:
        # --- Events ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    paused = not paused
                    if not paused:
                        # Reset playback clock
                        playback_start = time.time()
                        if telem_idx < len(telemetry):
                            session_start = telemetry[telem_idx]['time']
                elif event.key == pygame.K_PLUS or event.key == pygame.K_EQUALS:
                    speed = min(20.0, speed * 1.5)
                elif event.key == pygame.K_MINUS:
                    speed = max(0.1, speed / 1.5)
                elif event.key == pygame.K_r:
                    scale = 100.0
                    offset_x = WIDTH / 2 - 2.5 * scale
                    offset_y = HEIGHT / 2
            elif event.type == pygame.MOUSEWHEEL:
                factor = 1.15 if event.y > 0 else 1 / 1.15
                scale *= factor
                scale = max(20, min(500, scale))
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                dragging = True
                drag_start = event.pos
                drag_offset_start = (offset_x, offset_y)
            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                dragging = False
            elif event.type == pygame.MOUSEMOTION and dragging:
                dx = event.pos[0] - drag_start[0]
                dy = event.pos[1] - drag_start[1]
                offset_x = drag_offset_start[0] + dx
                offset_y = drag_offset_start[1] + dy

        # --- Advance playback ---
        if not paused and telem_idx < len(telemetry):
            elapsed = (time.time() - playback_start) * speed
            target_time = session_start + elapsed

            while telem_idx < len(telemetry) and telemetry[telem_idx]['time'] <= target_time:
                row = telemetry[telem_idx]
                agent = row['agent']
                bot_state[agent] = row
                paths[agent].append((row['x'], row['y']))
                pkt_counts[agent] += 1
                telem_idx += 1

            while cloud_idx < len(pointcloud) and pointcloud[cloud_idx]['time'] <= target_time:
                row = pointcloud[cloud_idx]
                agent = row['agent']
                clouds[agent].append((row['x'], row['y']))
                cloud_idx += 1

        # --- Render ---
        screen.fill(BG_COLOR)

        # Grid
        for world_val in range(-5, 15):
            sx, sy = world_to_screen(world_val, 0)
            if 0 <= sx < WIDTH:
                pygame.draw.line(screen, GRID_COLOR, (sx, 0), (sx, HEIGHT), 1)
            sx, sy = world_to_screen(0, world_val)
            if 0 <= sy < HEIGHT:
                pygame.draw.line(screen, GRID_COLOR, (0, sy), (WIDTH, sy), 1)

        # Origin crosshair
        ox, oy = world_to_screen(0, 0)
        pygame.draw.line(screen, (60, 80, 120), (ox - 10, oy), (ox + 10, oy), 2)
        pygame.draw.line(screen, (60, 80, 120), (ox, oy - 10), (ox, oy + 10), 2)

        # Bot2 origin crosshair
        ox2, oy2 = world_to_screen(5, 0)
        pygame.draw.line(screen, (100, 50, 80), (ox2 - 10, oy2), (ox2 + 10, oy2), 2)
        pygame.draw.line(screen, (100, 50, 80), (ox2, oy2 - 10), (ox2, oy2 + 10), 2)

        # Point clouds
        for bot_id in [1, 2]:
            color = BOT_COLORS[bot_id]['cloud']
            faded = (color[0] // 3, color[1] // 3, color[2] // 3)
            for wx, wy in clouds[bot_id][-3000:]:
                sx, sy = world_to_screen(wx, wy)
                if 0 <= sx < WIDTH and 0 <= sy < HEIGHT:
                    screen.set_at((sx, sy), faded)

        # Paths
        for bot_id in [1, 2]:
            pts = paths[bot_id]
            if len(pts) >= 2:
                color = BOT_COLORS[bot_id]['path']
                step = max(1, len(pts) // 500)
                screen_pts = []
                for i in range(0, len(pts), step):
                    screen_pts.append(world_to_screen(pts[i][0], pts[i][1]))
                if len(screen_pts) >= 2:
                    pygame.draw.lines(screen, color, False, screen_pts, 2)

        # Robot markers
        for bot_id in [1, 2]:
            if bot_id not in bot_state:
                continue
            row = bot_state[bot_id]
            sx, sy = world_to_screen(row['x'], row['y'])
            color = BOT_COLORS[bot_id]['main']

            # Triangle
            size = 10
            pts = []
            for angle_offset in [0, 2.4, -2.4]:
                r = size * 1.5 if angle_offset == 0 else size
                a = row['yaw'] + angle_offset
                px = sx + int(r * math.cos(a))
                py = sy - int(r * math.sin(a))
                pts.append((px, py))
            pygame.draw.polygon(screen, color, pts)
            pygame.draw.polygon(screen, (255, 255, 255), pts, 1)

            # Label
            label = font_sm.render(BOT_COLORS[bot_id]['name'], True, color)
            screen.blit(label, (sx + 14, sy - 6))

        # HUD
        y = 10
        title = font_lg.render("DUAL-BOT SESSION PLAYBACK", True, TEXT_COLOR)
        screen.blit(title, (10, y))
        y += 25

        for bot_id in [1, 2]:
            bc = BOT_COLORS[bot_id]
            pkts = pkt_counts[bot_id]
            if bot_id in bot_state:
                st = bot_state[bot_id]
                line = f"{bc['name']}: Pkts:{pkts} | ({st['x']:.2f},{st['y']:.2f}) {math.degrees(st['yaw']):.0f}deg"
                surf = font_md.render(line, True, bc['main'])
            else:
                surf = font_md.render(f"{bc['name']}: WAITING", True, TEXT_DIM)
            screen.blit(surf, (10, y))
            y += 18

        # Progress bar
        progress = telem_idx / max(1, len(telemetry))
        bar_w = 200
        pygame.draw.rect(screen, (50, 60, 90), (10, y + 5, bar_w, 8))
        pygame.draw.rect(screen, (0, 191, 255), (10, y + 5, int(bar_w * progress), 8))
        prog_text = font_sm.render(
            f"Progress: {telem_idx}/{len(telemetry)} | Speed: {speed:.1f}x" +
            (" | PAUSED" if paused else ""),
            True, TEXT_COLOR)
        screen.blit(prog_text, (bar_w + 20, y + 2))
        y += 25

        # Status
        if telem_idx >= len(telemetry):
            done = font_md.render("PLAYBACK COMPLETE - Press ESC to exit", True, (0, 255, 100))
            screen.blit(done, (10, y))

        # Controls hint
        hint = font_sm.render("Scroll=Zoom | Drag=Pan | SPACE=Pause | +/-=Speed | R=Reset | ESC=Quit", True, TEXT_DIM)
        screen.blit(hint, (10, HEIGHT - 20))

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    print(f"\nPlayback finished. {pkt_counts[1] + pkt_counts[2]} total packets displayed.")


if __name__ == "__main__":
    main()
