"""
Live Top-Down Persistent Mapper with Full Logging
- Matches new 4-sensor QuasarPacket (Front/Left/Back/Right)
- Creates timestamped log folder each session
- Logs telemetry + point cloud to CSV
- Visualizes persistent 2D map in real-time
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

# ── Packet Format ──────────────────────────────────────────────────────────────
# Matches: char[4] magic, uint8 agent_id, float x,y,yaw, int32 enc,
#          uint32 v2v,  float front, left, back, right
UDP_PORT    = 8888
PACKET_FMT  = '<4sBfffiIffff'   # 4s B fff i I ffff
PACKET_SIZE = struct.calcsize(PACKET_FMT)

# Trust filter
MAX_DIST_M = 1.20   # ignore > 1.2m
MIN_DIST_M = 0.05   # ignore < 5cm

# Sensor ray angles relative to robot forward (yaw=0 → +X)
# Front=0°, Left=+90°, Back=180°, Right=-90°
SENSOR_ANGLES_DEG = {'front': 0, 'left': 90, 'back': 180, 'right': -90}

TRIANGLE_SIZE = 0.15

def make_triangle():
    return np.array([
        [ 0,                TRIANGLE_SIZE*0.7],
        [-TRIANGLE_SIZE*0.5, -TRIANGLE_SIZE*0.5],
        [ TRIANGLE_SIZE*0.5, -TRIANGLE_SIZE*0.5],
    ])

def main():
    # ── Logging setup ──────────────────────────────────────────────────────────
    ts   = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    ldir = os.path.join(os.path.dirname(__file__), "logs", f"session_{ts}")
    os.makedirs(ldir, exist_ok=True)

    f_telem  = open(os.path.join(ldir, "telemetry.csv"),  'w', newline='')
    f_points = open(os.path.join(ldir, "pointcloud.csv"), 'w', newline='')
    w_telem  = csv.writer(f_telem)
    w_points = csv.writer(f_points)
    w_telem.writerow( ['time', 'agent', 'x', 'y', 'yaw_deg', 'encoder',
                       'v2v', 'front_cm', 'left_cm', 'back_cm', 'right_cm'])
    w_points.writerow(['time', 'sensor', 'x', 'y'])

    print("=" * 60)
    print(f"  4-Sensor Live Mapper  |  Port {UDP_PORT}")
    print(f"  Packet size expected: {PACKET_SIZE} bytes")
    print(f"  Logging → {ldir}")
    print("=" * 60)

    # ── Socket ─────────────────────────────────────────────────────────────────
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(('0.0.0.0', UDP_PORT))
        print(f"[OK] Listening on 0.0.0.0:{UDP_PORT}")
    except Exception as e:
        print(f"[ERR] Cannot bind: {e}")
        return
    sock.setblocking(False)

    # ── Plot setup ─────────────────────────────────────────────────────────────
    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_title("Live 4-Sensor Map", fontsize=14)
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.35)

    # Sensor point cloud: different colours per sensor
    colors = {'front': 'red', 'left': 'green', 'back': 'orange', 'right': 'blue'}
    scatters = {}
    map_x  = {k: [] for k in colors}
    map_y  = {k: [] for k in colors}
    for k, col in colors.items():
        scatters[k] = ax.scatter([], [], s=6, c=col, alpha=0.6, label=k.capitalize())

    # Robot path
    path_x, path_y = [], []
    path_line, = ax.plot([], [], 'k-', linewidth=1, alpha=0.5, label='Path')

    # Robot marker
    tri  = make_triangle()
    rpatch = patches.Polygon(tri, closed=True,
                             facecolor='lime', edgecolor='black',
                             linewidth=1, zorder=10, label='Robot')
    ax.add_patch(rpatch)

    ax.set_xlim(-3, 3)
    ax.set_ylim(-3, 3)
    ax.legend(loc='upper right', fontsize=9)

    # ── Main loop ──────────────────────────────────────────────────────────────
    pkt_count = 0
    try:
        while True:
            try:
                data, addr = sock.recvfrom(65535)

                if len(data) != PACKET_SIZE:
                    print(f"[SKIP] Bad size: {len(data)} (expected {PACKET_SIZE})")
                    continue

                u = struct.unpack(PACKET_FMT, data)
                magic, agent_id, rx, ry, ryaw, enc, v2v, \
                    d_front, d_left, d_back, d_right = u

                if magic != b'QSRL':
                    print(f"[SKIP] Bad magic: {magic}")
                    continue

                pkt_count += 1
                now = time.time()

                # ── Log telemetry ──────────────────────────────────────────────
                w_telem.writerow([
                    f"{now:.3f}", agent_id,
                    f"{rx:.4f}", f"{ry:.4f}",
                    f"{math.degrees(ryaw):.2f}", enc, v2v,
                    f"{d_front*100:.1f}", f"{d_left*100:.1f}",
                    f"{d_back*100:.1f}",  f"{d_right*100:.1f}",
                ])
                f_telem.flush()

                # ── Update robot path ──────────────────────────────────────────
                path_x.append(rx)
                path_y.append(ry)
                path_line.set_data(path_x, path_y)

                # ── Robot marker ───────────────────────────────────────────────
                t = (Affine2D().rotate(ryaw - math.pi/2)
                     + Affine2D().translate(rx, ry)
                     + ax.transData)
                rpatch.set_transform(t)

                # ── Project sensor readings → world coordinates ────────────────
                sensors = {
                    'front': d_front, 'left': d_left,
                    'back':  d_back,  'right': d_right
                }
                for name, dist in sensors.items():
                    if MIN_DIST_M < dist <= MAX_DIST_M:
                        ray_angle = ryaw + math.radians(SENSOR_ANGLES_DEG[name])
                        wx = rx + dist * math.cos(ray_angle)
                        wy = ry + dist * math.sin(ray_angle)
                        map_x[name].append(wx)
                        map_y[name].append(wy)
                        w_points.writerow([f"{now:.3f}", name, f"{wx:.4f}", f"{wy:.4f}"])
                f_points.flush()

                # ── Update scatter plots ───────────────────────────────────────
                for name in colors:
                    if map_x[name]:
                        scatters[name].set_offsets(
                            np.column_stack([map_x[name], map_y[name]]))

                # ── Auto-scale ─────────────────────────────────────────────────
                all_wx = sum(map_x.values(), []) + path_x
                all_wy = sum(map_y.values(), []) + path_y
                if len(all_wx) > 10:
                    ax.set_xlim(min(all_wx) - 0.5, max(all_wx) + 0.5)
                    ax.set_ylim(min(all_wy) - 0.5, max(all_wy) + 0.5)

                total_pts = sum(len(v) for v in map_x.values())
                ax.set_title(
                    f"Map | Pkts:{pkt_count} | Pts:{total_pts} | "
                    f"({rx:.2f},{ry:.2f}) {math.degrees(ryaw):.0f}°", fontsize=11)

                fig.canvas.draw()
                fig.canvas.flush_events()

                # ── Terminal print ─────────────────────────────────────────────
                print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] "
                      f"Agent {agent_id} | "
                      f"({rx:.2f},{ry:.2f},{math.degrees(ryaw):.0f}°) | "
                      f"F:{d_front*100:.0f}cm L:{d_left*100:.0f}cm "
                      f"B:{d_back*100:.0f}cm R:{d_right*100:.0f}cm")

            except BlockingIOError:
                plt.pause(0.02)
            except Exception as e:
                print(f"[ERR] {e}")

    except KeyboardInterrupt:
        print(f"\n[EXIT] {pkt_count} packets logged. Saving...")
        np.savetxt(os.path.join(ldir, "pointcloud_all.csv"),
                   np.column_stack([
                       sum(map_x.values(), []),
                       sum(map_y.values(), [])
                   ]), delimiter=",", header="x,y", comments="")
        print(f"[SAVED] {ldir}")
    finally:
        f_telem.close()
        f_points.close()
        sock.close()
        plt.close()

if __name__ == '__main__':
    main()
