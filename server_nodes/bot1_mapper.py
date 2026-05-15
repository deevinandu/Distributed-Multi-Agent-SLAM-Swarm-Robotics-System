"""
Bot 1 Single-Agent Live Mapper
==============================
Uses TkAgg backend (tkinter 8.6 confirmed available).
UDP receive runs in a background thread.
Tk/matplotlib GUI runs on the main thread.

Usage:
    python bot1_mapper.py

Close the window or press Ctrl+C to save and exit.
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

# ── Backend MUST be set before importing pyplot ───────────────────────────────
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.transforms import Affine2D

# ══════════════════════════════════════════════════════════════════════════════
PACKET_FMT  = '<4sBfffiIffff'
PACKET_SIZE = struct.calcsize(PACKET_FMT)
UDP_PORT    = 8888

SENSOR_ANGLES = {'front': 0, 'left': 90, 'back': 180, 'right': -90}
MIN_DIST_M    = 0.05
MAX_DIST_M    = 1.50
TRIANGLE_SIZE = 0.12
REFRESH_MS    = 100   # GUI refresh interval

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

# ══════════════════════════════════════════════════════════════════════════════
# SHARED STATE
# ══════════════════════════════════════════════════════════════════════════════
_lock      = threading.Lock()
_new_data  = False
_pkt_count = 0
_rx = _ry = _ryaw = 0.0
_d_f = _d_l = _d_b = _d_r = 0.0
_pts_x  = {s: [] for s in SENSOR_ANGLES}
_pts_y  = {s: [] for s in SENSOR_ANGLES}
_path_x = []
_path_y = []
_stop   = False   # signal rx thread to quit

# ══════════════════════════════════════════════════════════════════════════════
# UDP RECEIVE THREAD
# ══════════════════════════════════════════════════════════════════════════════
def rx_thread(sock, w_tel, f_tel, w_pts, f_pts):
    global _new_data, _pkt_count, _rx, _ry, _ryaw
    global _d_f, _d_l, _d_b, _d_r, _stop

    while not _stop:
        try:
            data, _ = sock.recvfrom(65535)
        except BlockingIOError:
            time.sleep(0.005)
            continue
        except Exception:
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

        hits = {}
        for name, dist in {'front': d_f, 'left': d_l,
                           'back': d_b, 'right': d_r}.items():
            if MIN_DIST_M < dist <= MAX_DIST_M:
                angle = ryaw + math.radians(SENSOR_ANGLES[name])
                wx = rx + dist * math.cos(angle)
                wy = ry + dist * math.sin(angle)
                hits[name] = (wx, wy)
                w_pts.writerow([f"{now:.3f}", name, f"{wx:.4f}", f"{wy:.4f}"])
        f_pts.flush()

        w_tel.writerow([f"{now:.3f}", f"{rx:.4f}", f"{ry:.4f}",
                        f"{math.degrees(ryaw):.2f}", enc,
                        f"{d_f*100:.1f}", f"{d_l*100:.1f}",
                        f"{d_b*100:.1f}", f"{d_r*100:.1f}"])
        f_tel.flush()

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
              f"#{_pkt_count} ({rx:.2f},{ry:.2f},{math.degrees(ryaw):.0f}°) "
              f"F:{d_f*100:.0f} L:{d_l*100:.0f} "
              f"B:{d_b*100:.0f} R:{d_r*100:.0f} cm")

# ══════════════════════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════════════════════
def main():
    global _stop

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
    print(f"  Logging  →  {ldir}")
    print("=" * 55)

    # ── Socket ────────────────────────────────────────────────────────────────
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(('0.0.0.0', UDP_PORT))
        print(f"[OK] Listening on 0.0.0.0:{UDP_PORT}")
    except OSError as e:
        print(f"[ERR] Cannot bind port {UDP_PORT}: {e}")
        return
    sock.setblocking(False)

    # ── Start rx thread ───────────────────────────────────────────────────────
    t = threading.Thread(target=rx_thread,
                         args=(sock, w_tel, f_tel, w_pts, f_pts),
                         daemon=True)
    t.start()

    # ── Build figure ──────────────────────────────────────────────────────────
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

    h = TRIANGLE_SIZE
    tri = np.array([[0, h*.7], [-h*.5, -h*.5], [h*.5, -h*.5]])
    tri_patch = patches.Polygon(tri, closed=True,
                                facecolor=ROBOT_COLOR, edgecolor='white',
                                linewidth=1.2, zorder=10, label='Bot 1')
    ax.add_patch(tri_patch)
    ax.set_xlim(-3, 3)
    ax.set_ylim(-3, 3)
    ax.set_title("Bot 1 Map  |  Waiting for packets...",
                 color='white', fontsize=11, pad=10)
    ax.legend(loc='upper right', fontsize=9,
              facecolor='#22224a', edgecolor='#444', labelcolor='white')

    # ── GUI REFRESH CALLBACK ──────────────────────────────────────────────────
    def refresh():
        global _new_data
        with _lock:
            if not _new_data:
                fig.canvas.manager.window.after(REFRESH_MS, refresh)
                return
            _new_data = False
            rx, ry, ryaw = _rx, _ry, _ryaw
            pc = _pkt_count
            spx = {k: list(v) for k, v in _pts_x.items()}
            spy = {k: list(v) for k, v in _pts_y.items()}
            phx = list(_path_x)
            phy = list(_path_y)

        path_line.set_data(phx, phy)

        t = (Affine2D().rotate(ryaw - math.pi / 2)
             + Affine2D().translate(rx, ry) + ax.transData)
        tri_patch.set_transform(t)

        for name in SENSOR_ANGLES:
            if spx[name]:
                scatters[name].set_offsets(
                    np.column_stack([spx[name], spy[name]]))

        all_x = sum(spx.values(), []) + phx
        all_y = sum(spy.values(), []) + phy
        if len(all_x) > 5:
            pad = 0.5
            ax.set_xlim(min(all_x)-pad, max(all_x)+pad)
            ax.set_ylim(min(all_y)-pad, max(all_y)+pad)

        total_pts = sum(len(v) for v in spx.values())
        ax.set_title(f"Bot 1  |  Pkts:{pc}  Pts:{total_pts}  "
                     f"({rx:.2f},{ry:.2f}) {math.degrees(ryaw):.0f}°",
                     color='white', fontsize=11, pad=10)
        fig.canvas.draw()
        fig.canvas.manager.window.after(REFRESH_MS, refresh)

    # Schedule first refresh after window opens
    fig.canvas.manager.window.after(REFRESH_MS, refresh)

    print("[OK] Window opened. Close it to stop and save.")

    try:
        plt.show(block=True)   # Tk mainloop runs here
    except KeyboardInterrupt:
        pass
    finally:
        _stop = True
        sock.close()
        f_tel.close()
        f_pts.close()

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
