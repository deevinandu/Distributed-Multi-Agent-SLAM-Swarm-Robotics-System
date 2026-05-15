"""
Bot 1 Single-Agent Live Mapper
==============================
Listens for UDP telemetry from Bot 1 (AGENT_ID=1) and plots its
trajectory + 4-sensor point cloud in real time.

Architecture: UDP receive runs in a background thread.
              matplotlib GUI runs on the main thread (required by Qt/Windows).

Usage:
    python bot1_mapper.py

Press Ctrl+C to stop and save.
"""
import socket
import struct
import math
import os
import time
import datetime
import csv
import threading
import numpy as np

# ── Matplotlib — must be set BEFORE importing pyplot ─────────────────────────
import matplotlib
matplotlib.use('Qt5Agg')          # PyQt5 5.15.5 is installed
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.transforms import Affine2D

# ══════════════════════════════════════════════════════════════════════════════
# CONSTANTS
# ══════════════════════════════════════════════════════════════════════════════
PACKET_FMT  = '<4sBfffiIffff'    # must match QuasarPacket in firmware
PACKET_SIZE = struct.calcsize(PACKET_FMT)
UDP_PORT    = 8888

SENSOR_ANGLES = {'front': 0, 'left': 90, 'back': 180, 'right': -90}
MIN_DIST_M    = 0.05
MAX_DIST_M    = 1.50
TRIANGLE_SIZE = 0.12

BG_DARK  = '#12121f'
BG_PANEL = '#0e0e1a'
GRID_COL = '#1e1e34'

SENSOR_COLORS = {
    'front': '#FF4455',
    'left':  '#44FF88',
    'back':  '#FFAA00',
    'right': '#4499FF',
}
PATH_COLOR  = '#00CFFF'
ROBOT_COLOR = '#00FFCC'

REFRESH_MS  = 80    # GUI refresh interval (ms)

# ══════════════════════════════════════════════════════════════════════════════
# SHARED STATE  (written by rx thread, read by GUI thread, protected by lock)
# ══════════════════════════════════════════════════════════════════════════════
_lock      = threading.Lock()
_new_data  = False          # flag: new packet waiting to be drawn
_pkt_count = 0
_rx        = 0.0
_ry        = 0.0
_ryaw      = 0.0
_d_f = _d_l = _d_b = _d_r = 0.0

# Accumulated point clouds & path (written inside lock)
_pts_x  = {s: [] for s in SENSOR_ANGLES}
_pts_y  = {s: [] for s in SENSOR_ANGLES}
_path_x = []
_path_y = []

# ══════════════════════════════════════════════════════════════════════════════
# LOGGING HELPERS
# ══════════════════════════════════════════════════════════════════════════════
def open_logs():
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
    return ldir, f_tel, f_pts, w_tel, w_pts

# ══════════════════════════════════════════════════════════════════════════════
# UDP RECEIVE THREAD
# ══════════════════════════════════════════════════════════════════════════════
def rx_thread(sock, w_tel, f_tel, w_pts, f_pts, stop_event):
    global _new_data, _pkt_count
    global _rx, _ry, _ryaw, _d_f, _d_l, _d_b, _d_r

    while not stop_event.is_set():
        try:
            data, _ = sock.recvfrom(65535)
        except BlockingIOError:
            time.sleep(0.005)
            continue
        except Exception as e:
            print(f"[SOCK ERR] {e}")
            time.sleep(0.01)
            continue

        if len(data) != PACKET_SIZE:
            continue

        try:
            magic, agent_id, rx, ry, ryaw, enc, v2v, \
                d_f, d_l, d_b, d_r = struct.unpack(PACKET_FMT, data)
        except struct.error:
            continue

        if magic != b'QSRL' or agent_id != 1:
            continue

        now = time.time()

        # Project sensor hits into world coordinates
        hits = {}
        readings = {'front': d_f, 'left': d_l, 'back': d_b, 'right': d_r}
        for name, dist in readings.items():
            if MIN_DIST_M < dist <= MAX_DIST_M:
                angle = ryaw + math.radians(SENSOR_ANGLES[name])
                wx = rx + dist * math.cos(angle)
                wy = ry + dist * math.sin(angle)
                hits[name] = (wx, wy)
                w_pts.writerow([f"{now:.3f}", name, f"{wx:.4f}", f"{wy:.4f}"])
        f_pts.flush()

        # Log telemetry
        w_tel.writerow([f"{now:.3f}", f"{rx:.4f}", f"{ry:.4f}",
                        f"{math.degrees(ryaw):.2f}", enc,
                        f"{d_f*100:.1f}", f"{d_l*100:.1f}",
                        f"{d_b*100:.1f}", f"{d_r*100:.1f}"])
        f_tel.flush()

        # Push to shared state (GUI thread will pick this up)
        with _lock:
            _pkt_count += 1
            _rx, _ry, _ryaw = rx, ry, ryaw
            _d_f, _d_l, _d_b, _d_r = d_f, d_l, d_b, d_r
            _path_x.append(rx)
            _path_y.append(ry)
            for name, (wx, wy) in hits.items():
                _pts_x[name].append(wx)
                _pts_y[name].append(wy)
            _new_data = True

        print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] "
              f"Pkt#{_pkt_count}  ({rx:.2f},{ry:.2f},{math.degrees(ryaw):.0f}°) "
              f"F:{d_f*100:.0f}cm L:{d_l*100:.0f}cm "
              f"B:{d_b*100:.0f}cm R:{d_r*100:.0f}cm")

# ══════════════════════════════════════════════════════════════════════════════
# GUI  (runs on main thread)
# ══════════════════════════════════════════════════════════════════════════════
def build_robot_triangle():
    h = TRIANGLE_SIZE
    return np.array([[ 0, h*.7], [-h*.5, -h*.5], [ h*.5, -h*.5]])

def build_figure():
    fig, ax = plt.subplots(figsize=(10, 10))
    fig.patch.set_facecolor(BG_PANEL)
    ax.set_facecolor(BG_DARK)
    ax.set_aspect('equal')
    ax.grid(True, color=GRID_COL, linestyle='--', linewidth=0.6, alpha=0.8)
    ax.tick_params(colors='#aaaacc')
    for sp in ax.spines.values():
        sp.set_color('#333355')

    scatters = {}
    for name, col in SENSOR_COLORS.items():
        scatters[name] = ax.scatter([], [], s=5, c=col, alpha=0.55,
                                    label=name.capitalize(), zorder=3)

    path_line, = ax.plot([], [], '-', color=PATH_COLOR,
                         linewidth=1.2, alpha=0.7, label='Path', zorder=4)

    ax.plot(0, 0, 'o', color=ROBOT_COLOR, markersize=9, zorder=6)
    ax.text(0.05, 0.05, 'Start', color=ROBOT_COLOR, fontsize=9)

    tri_patch = patches.Polygon(build_robot_triangle(), closed=True,
                                facecolor=ROBOT_COLOR, edgecolor='white',
                                linewidth=1.2, zorder=10, label='Bot 1')
    ax.add_patch(tri_patch)
    ax.set_xlim(-3, 3)
    ax.set_ylim(-3, 3)
    ax.set_title("Bot 1 Map  |  Waiting for packets...",
                 color='white', fontsize=11, pad=10)
    ax.legend(loc='upper right', fontsize=9,
              facecolor='#22224a', edgecolor='#444', labelcolor='white')
    return fig, ax, scatters, path_line, tri_patch


def refresh_gui(fig, ax, scatters, path_line, tri_patch):
    """Called every REFRESH_MS ms by matplotlib's timer on the main thread."""
    global _new_data

    with _lock:
        if not _new_data:
            return
        _new_data = False
        # Snapshot current state
        rx, ry, ryaw   = _rx, _ry, _ryaw
        pkt_count      = _pkt_count
        snap_pts_x     = {k: list(v) for k, v in _pts_x.items()}
        snap_pts_y     = {k: list(v) for k, v in _pts_y.items()}
        snap_path_x    = list(_path_x)
        snap_path_y    = list(_path_y)

    # Update path
    path_line.set_data(snap_path_x, snap_path_y)

    # Update robot triangle
    t = (Affine2D().rotate(ryaw - math.pi / 2)
         + Affine2D().translate(rx, ry)
         + ax.transData)
    tri_patch.set_transform(t)

    # Update scatter plots
    for name in SENSOR_ANGLES:
        if snap_pts_x[name]:
            scatters[name].set_offsets(
                np.column_stack([snap_pts_x[name], snap_pts_y[name]]))

    # Auto-scale
    all_x = sum(snap_pts_x.values(), []) + snap_path_x
    all_y = sum(snap_pts_y.values(), []) + snap_path_y
    if len(all_x) > 5:
        pad = 0.5
        ax.set_xlim(min(all_x) - pad, max(all_x) + pad)
        ax.set_ylim(min(all_y) - pad, max(all_y) + pad)

    total_pts = sum(len(v) for v in snap_pts_x.values())
    ax.set_title(
        f"Bot 1 Map  |  Pkts:{pkt_count}  Pts:{total_pts}  "
        f"({rx:.2f},{ry:.2f}) {math.degrees(ryaw):.0f}°",
        color='white', fontsize=11, pad=10)

    fig.canvas.draw_idle()

# ══════════════════════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════════════════════
def main():
    ldir, f_tel, f_pts, w_tel, w_pts = open_logs()

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
        print(f"[OK] Listening on 0.0.0.0:{UDP_PORT}  (close window to stop)")
    except OSError as e:
        print(f"[ERR] Cannot bind port {UDP_PORT}: {e}")
        return
    sock.setblocking(False)

    # ── Build figure (main thread) ────────────────────────────────────────────
    fig, ax, scatters, path_line, tri_patch = build_figure()

    # ── Start rx thread ───────────────────────────────────────────────────────
    stop_event = threading.Event()
    t = threading.Thread(
        target=rx_thread,
        args=(sock, w_tel, f_tel, w_pts, f_pts, stop_event),
        daemon=True)
    t.start()

    # ── matplotlib timer drives GUI refresh on main thread ────────────────────
    timer = fig.canvas.new_timer(interval=REFRESH_MS)
    timer.add_callback(refresh_gui, fig, ax, scatters, path_line, tri_patch)
    timer.start()

    print("[OK] Window open. Close it or press Ctrl+C to stop and save.")

    try:
        plt.show(block=True)      # <-- blocks here; Qt event loop owns main thread
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        timer.stop()
        sock.close()
        f_tel.close()
        f_pts.close()

        # Save merged outputs
        with _lock:
            all_x = sum(_pts_x.values(), [])
            all_y = sum(_pts_y.values(), [])
            px, py = list(_path_x), list(_path_y)

        if all_x:
            np.savetxt(os.path.join(ldir, "pointcloud_all.csv"),
                       np.column_stack([all_x, all_y]),
                       delimiter=',', header='x,y', comments='')
        if px:
            np.savetxt(os.path.join(ldir, "path.csv"),
                       np.column_stack([px, py]),
                       delimiter=',', header='x,y', comments='')

        print(f"\n[SAVED] {_pkt_count} packets → {ldir}")


if __name__ == '__main__':
    main()
