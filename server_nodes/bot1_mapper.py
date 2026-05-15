"""
Bot 1 Single-Agent Live Mapper
==============================
Listens for QuasarPacket UDP telemetry from Bot 1 (AGENT_ID=1)
and plots its trajectory + 4-sensor point cloud in real time.

Usage:
    python bot1_mapper.py

No territory logic, no zone packets — clean and simple.
Press Ctrl+C to save and exit.
"""
import socket
import struct
import math
import os
import time
import datetime
import csv
import matplotlib
try:
    matplotlib.use('TkAgg')
except Exception:
    pass
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.transforms import Affine2D
import numpy as np

# ── Packet format (must match QuasarPacket in firmware) ───────────────────────
# char[4] magic | uint8 agent_id | float x,y,yaw | int32 enc | uint32 v2v
# | float front, left, back, right
PACKET_FMT  = '<4sBfffiIffff'
PACKET_SIZE = struct.calcsize(PACKET_FMT)

UDP_PORT    = 8888          # must match agent_port in Bot1 firmware

# Sensor ray angles relative to robot heading (yaw=0 → +X axis)
SENSOR_ANGLES = {'front': 0, 'left': 90, 'back': 180, 'right': -90}

# Point filter
MIN_DIST_M = 0.05   # ignore readings < 5 cm (noise)
MAX_DIST_M = 1.50   # ignore readings > 1.5 m (far-field garbage)

TRIANGLE_SIZE = 0.12   # robot marker size in metres

# Colour scheme (dark theme)
BG_DARK   = '#1a1a2e'
BG_PANEL  = '#16213e'
GRID_COL  = '#2a2a4a'

SENSOR_COLORS = {
    'front': '#FF4455',   # red
    'left':  '#44FF88',   # green
    'back':  '#FFAA00',   # amber
    'right': '#4499FF',   # blue
}
PATH_COLOR  = '#00CFFF'
ROBOT_COLOR = '#00FFCC'


def robot_triangle():
    """Triangle pointing in the +Y direction (rotated to heading in-plot)."""
    h = TRIANGLE_SIZE
    return np.array([
        [ 0,        h * 0.7],
        [-h * 0.5, -h * 0.5],
        [ h * 0.5, -h * 0.5],
    ])


def main():
    # ── Logging ───────────────────────────────────────────────────────────────
    ts   = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    ldir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "logs", f"bot1_{ts}")
    os.makedirs(ldir, exist_ok=True)

    f_tel = open(os.path.join(ldir, "telemetry.csv"),  'w', newline='')
    f_pts = open(os.path.join(ldir, "pointcloud.csv"), 'w', newline='')
    w_tel = csv.writer(f_tel)
    w_pts = csv.writer(f_pts)
    w_tel.writerow(['time', 'x', 'y', 'yaw_deg', 'encoder',
                    'front_cm', 'left_cm', 'back_cm', 'right_cm'])
    w_pts.writerow(['time', 'sensor', 'wx', 'wy'])

    print("=" * 55)
    print(f"  Bot 1 Live Mapper  |  UDP port {UDP_PORT}")
    print(f"  Packet size: {PACKET_SIZE} bytes")
    print(f"  Logging to: {ldir}")
    print("=" * 55)

    # ── Socket ────────────────────────────────────────────────────────────────
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(('0.0.0.0', UDP_PORT))
        print(f"[OK] Listening on 0.0.0.0:{UDP_PORT}  (Ctrl+C to stop)")
    except OSError as e:
        print(f"[ERR] Cannot bind port {UDP_PORT}: {e}")
        return
    sock.setblocking(False)

    # ── Figure ────────────────────────────────────────────────────────────────
    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 10))
    fig.patch.set_facecolor(BG_PANEL)
    ax.set_facecolor(BG_DARK)
    ax.set_aspect('equal')
    ax.grid(True, color=GRID_COL, linestyle='--', linewidth=0.6, alpha=0.8)
    ax.tick_params(colors='#aaaacc')
    for sp in ax.spines.values():
        sp.set_color('#333355')

    # Sensor scatter plots
    scatters = {}
    pts_x = {s: [] for s in SENSOR_ANGLES}
    pts_y = {s: [] for s in SENSOR_ANGLES}
    for name, col in SENSOR_COLORS.items():
        scatters[name] = ax.scatter([], [], s=5, c=col, alpha=0.55,
                                    label=name.capitalize(), zorder=3)

    # Robot path trail
    path_x, path_y = [], []
    path_line, = ax.plot([], [], '-', color=PATH_COLOR,
                         linewidth=1.2, alpha=0.6, label='Path', zorder=4)

    # Start marker
    ax.plot(0, 0, 'o', color=ROBOT_COLOR, markersize=10, zorder=6)
    ax.text(0.05, 0.05, 'Start', color=ROBOT_COLOR, fontsize=9)

    # Robot triangle marker
    tri_patch = patches.Polygon(robot_triangle(), closed=True,
                                facecolor=ROBOT_COLOR, edgecolor='white',
                                linewidth=1.2, zorder=10, label='Bot 1')
    ax.add_patch(tri_patch)

    ax.set_xlim(-3, 3)
    ax.set_ylim(-3, 3)
    leg = ax.legend(loc='upper right', fontsize=9,
                    facecolor='#22224a', edgecolor='#444',
                    labelcolor='white')

    # ── Main loop ─────────────────────────────────────────────────────────────
    pkt_count  = 0
    last_title = time.time()

    try:
        while True:
            try:
                data, addr = sock.recvfrom(65535)
            except BlockingIOError:
                plt.pause(0.02)
                continue
            except Exception as e:
                print(f"[SOCK ERR] {e}")
                plt.pause(0.02)
                continue

            # Size check
            if len(data) != PACKET_SIZE:
                print(f"[SKIP] Wrong size: {len(data)} (expected {PACKET_SIZE})")
                continue

            # Unpack
            try:
                magic, agent_id, rx, ry, ryaw, enc, v2v, \
                    d_f, d_l, d_b, d_r = struct.unpack(PACKET_FMT, data)
            except struct.error as e:
                print(f"[UNPACK ERR] {e}")
                continue

            if magic != b'QSRL':
                print(f"[SKIP] Bad magic: {magic}")
                continue

            if agent_id != 1:
                # Silently ignore packets from other bots
                continue

            pkt_count += 1
            now = time.time()

            # ── CSV log ───────────────────────────────────────────────────────
            w_tel.writerow([f"{now:.3f}", f"{rx:.4f}", f"{ry:.4f}",
                            f"{math.degrees(ryaw):.2f}", enc,
                            f"{d_f*100:.1f}", f"{d_l*100:.1f}",
                            f"{d_b*100:.1f}", f"{d_r*100:.1f}"])
            f_tel.flush()

            # ── Path update ───────────────────────────────────────────────────
            path_x.append(rx)
            path_y.append(ry)
            path_line.set_data(path_x, path_y)

            # ── Robot marker ──────────────────────────────────────────────────
            t = (Affine2D().rotate(ryaw - math.pi / 2)
                 + Affine2D().translate(rx, ry)
                 + ax.transData)
            tri_patch.set_transform(t)

            # ── Project sensor rays → world coordinates ───────────────────────
            readings = {'front': d_f, 'left': d_l, 'back': d_b, 'right': d_r}
            for name, dist in readings.items():
                if MIN_DIST_M < dist <= MAX_DIST_M:
                    angle = ryaw + math.radians(SENSOR_ANGLES[name])
                    wx = rx + dist * math.cos(angle)
                    wy = ry + dist * math.sin(angle)
                    pts_x[name].append(wx)
                    pts_y[name].append(wy)
                    w_pts.writerow([f"{now:.3f}", name,
                                    f"{wx:.4f}", f"{wy:.4f}"])
            f_pts.flush()

            # ── Update scatter plots ───────────────────────────────────────────
            for name in SENSOR_ANGLES:
                if pts_x[name]:
                    scatters[name].set_offsets(
                        np.column_stack([pts_x[name], pts_y[name]]))

            # ── Auto-scale ────────────────────────────────────────────────────
            all_x = sum(pts_x.values(), []) + path_x
            all_y = sum(pts_y.values(), []) + path_y
            if len(all_x) > 5:
                pad = 0.5
                ax.set_xlim(min(all_x) - pad, max(all_x) + pad)
                ax.set_ylim(min(all_y) - pad, max(all_y) + pad)

            # ── Title (throttled to avoid flicker) ────────────────────────────
            if time.time() - last_title > 0.2:
                total_pts = sum(len(v) for v in pts_x.values())
                ax.set_title(
                    f"Bot 1 Map  |  Pkts: {pkt_count}  |  Pts: {total_pts}  |  "
                    f"Pos: ({rx:.2f}, {ry:.2f})  {math.degrees(ryaw):.0f}°",
                    color='white', fontsize=11, pad=10)
                last_title = time.time()

            fig.canvas.draw()
            fig.canvas.flush_events()

            # ── Terminal ──────────────────────────────────────────────────────
            print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] "
                  f"Pkt#{pkt_count}  ({rx:.2f},{ry:.2f},{math.degrees(ryaw):.0f}°) "
                  f"F:{d_f*100:.0f}cm L:{d_l*100:.0f}cm "
                  f"B:{d_b*100:.0f}cm R:{d_r*100:.0f}cm")

    except KeyboardInterrupt:
        print(f"\n[EXIT] {pkt_count} packets received. Saving final map...")

        # Save merged point cloud
        all_x = sum(pts_x.values(), [])
        all_y = sum(pts_y.values(), [])
        if all_x:
            np.savetxt(os.path.join(ldir, "pointcloud_all.csv"),
                       np.column_stack([all_x, all_y]),
                       delimiter=',', header='x,y', comments='')
            print(f"[SAVED] pointcloud_all.csv  ({len(all_x)} points)")

        # Save path
        if path_x:
            np.savetxt(os.path.join(ldir, "path.csv"),
                       np.column_stack([path_x, path_y]),
                       delimiter=',', header='x,y', comments='')
            print(f"[SAVED] path.csv  ({len(path_x)} poses)")

        print(f"[SAVED] All logs in: {ldir}")

    finally:
        f_tel.close()
        f_pts.close()
        sock.close()
        plt.close('all')


if __name__ == '__main__':
    main()
