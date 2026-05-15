"""
Dual-Bot Live Mapper with Territory Sharing
============================================
- Receives telemetry from both Bot1 (AGENT_ID=1) and Bot2 (AGENT_ID=2)
- Plots both bots on the same map with distinct colors
- Maintains per-bot occupancy grids
- Sends territory zone packets back to each bot so they avoid
  areas already mapped by the other bot

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
import matplotlib
try:
    matplotlib.use('TkAgg')
except Exception:
    pass
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.transforms import Affine2D
import numpy as np

# ── Packet Formats ─────────────────────────────────────────────────────────────
# QuasarPacket from bots
PACKET_FMT  = '<4sBfffiIffff'
PACKET_SIZE = struct.calcsize(PACKET_FMT)

# ZonePacket sent TO bots (territory avoidance)
ZONE_FMT    = '<4sffff'
ZONE_SIZE   = struct.calcsize(ZONE_FMT)

# Trust filter
MAX_DIST_M = 1.20
MIN_DIST_M = 0.05

# Sensor ray angles relative to robot forward
SENSOR_ANGLES_DEG = {'front': 0, 'left': 90, 'back': 180, 'right': -90}

TRIANGLE_SIZE = 0.12

# Territory zone update interval (seconds)
ZONE_UPDATE_INTERVAL = 2.0


def make_triangle():
    return np.array([
        [ 0,                TRIANGLE_SIZE * 0.7],
        [-TRIANGLE_SIZE*0.5, -TRIANGLE_SIZE*0.5],
        [ TRIANGLE_SIZE*0.5, -TRIANGLE_SIZE*0.5],
    ])


def compute_bounding_box(points_x, points_y):
    """Compute axis-aligned bounding box from point lists."""
    if not points_x:
        return None
    return (min(points_x), min(points_y), max(points_x), max(points_y))


def send_zone_to_bot(sock, bot_addr, zone_box):
    """Send a ZONE packet to a bot telling it about forbidden territory."""
    if zone_box is None or bot_addr is None:
        return
    pkt = struct.pack(ZONE_FMT, b'ZONE',
                      zone_box[0], zone_box[1],
                      zone_box[2], zone_box[3])
    try:
        sock.sendto(pkt, bot_addr)
    except Exception as e:
        print(f"[ZONE-TX] Error: {e}")


def main():
    parser = argparse.ArgumentParser(description="Dual-Bot Live Mapper")
    parser.add_argument('--separation', type=float, default=0.5,
                        help='Initial separation between bots in meters (default: 0.5)')
    parser.add_argument('--port', type=int, default=8888,
                        help='UDP port to listen on (default: 8888)')
    args = parser.parse_args()

    separation = args.separation

    # ── Logging setup ──────────────────────────────────────────────────────────
    ts   = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    ldir = os.path.join(os.path.dirname(__file__), "logs", f"dual_session_{ts}")
    os.makedirs(ldir, exist_ok=True)

    f_telem  = open(os.path.join(ldir, "telemetry.csv"),  'w', newline='')
    f_points = open(os.path.join(ldir, "pointcloud.csv"), 'w', newline='')
    w_telem  = csv.writer(f_telem)
    w_points = csv.writer(f_points)
    w_telem.writerow(['time', 'agent', 'x', 'y', 'yaw_deg', 'encoder',
                      'v2v', 'front_cm', 'left_cm', 'back_cm', 'right_cm'])
    w_points.writerow(['time', 'agent', 'sensor', 'x', 'y'])

    print("=" * 70)
    print(f"  DUAL-BOT Live Mapper  |  Port {args.port}")
    print(f"  Bot1 origin: (0, 0)   |  Bot2 origin: ({separation}, 0)")
    print(f"  Packet size: {PACKET_SIZE} bytes")
    print(f"  Logging → {ldir}")
    print("=" * 70)

    # ── Socket ─────────────────────────────────────────────────────────────────
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(('0.0.0.0', args.port))
        print(f"[OK] Listening on 0.0.0.0:{args.port}")
    except Exception as e:
        print(f"[ERR] Cannot bind: {e}")
        return
    sock.setblocking(False)

    # ── Per-bot state ──────────────────────────────────────────────────────────
    bot_colors = {
        1: {'path': '#00BFFF', 'front': '#FF4444', 'left': '#44FF44',
            'back': '#FF8800', 'right': '#4488FF', 'robot': '#00FFFF', 'name': 'Bot1'},
        2: {'path': '#FF69B4', 'front': '#CC0000', 'left': '#00CC00',
            'back': '#CC6600', 'right': '#0044CC', 'robot': '#FF00FF', 'name': 'Bot2'},
    }

    # Point clouds per bot per sensor
    map_x = {1: {k: [] for k in SENSOR_ANGLES_DEG},
             2: {k: [] for k in SENSOR_ANGLES_DEG}}
    map_y = {1: {k: [] for k in SENSOR_ANGLES_DEG},
             2: {k: [] for k in SENSOR_ANGLES_DEG}}

    # Robot paths
    path_x = {1: [], 2: []}
    path_y = {1: [], 2: []}

    # Bot addresses (for sending zone packets back)
    bot_addrs   = {1: None, 2: None}
    bot_ports   = {1: 8888, 2: 8889}

    # ── Plot setup ─────────────────────────────────────────────────────────────
    plt.ion()
    fig, ax = plt.subplots(figsize=(12, 10))
    ax.set_title("Dual-Bot Live Mapper", fontsize=14, fontweight='bold')
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.3)
    ax.set_facecolor('#1a1a2e')
    fig.patch.set_facecolor('#16213e')
    ax.tick_params(colors='white')
    ax.xaxis.label.set_color('white')
    ax.yaxis.label.set_color('white')
    ax.title.set_color('white')
    for spine in ax.spines.values():
        spine.set_color('#333')

    # Scatter plots per bot per sensor
    scatters = {}
    for bot_id in [1, 2]:
        scatters[bot_id] = {}
        bc = bot_colors[bot_id]
        for sensor in SENSOR_ANGLES_DEG:
            scatters[bot_id][sensor] = ax.scatter(
                [], [], s=6, c=bc[sensor], alpha=0.5,
                label=f"{bc['name']} {sensor}" if sensor == 'front' else None)

    # Path lines
    path_lines = {}
    for bot_id in [1, 2]:
        bc = bot_colors[bot_id]
        line, = ax.plot([], [], '-', color=bc['path'], linewidth=1.5,
                        alpha=0.7, label=f"{bc['name']} Path")
        path_lines[bot_id] = line

    # Robot triangles
    rpatches = {}
    for bot_id in [1, 2]:
        bc = bot_colors[bot_id]
        tri = make_triangle()
        p = patches.Polygon(tri, closed=True,
                            facecolor=bc['robot'], edgecolor='white',
                            linewidth=1.5, zorder=10,
                            label=f"{bc['name']}")
        ax.add_patch(p)
        rpatches[bot_id] = p

    # Territory zone rectangles (shown as shaded areas)
    zone_patches = {}
    for bot_id in [1, 2]:
        bc = bot_colors[bot_id]
        rect = patches.Rectangle((0, 0), 0, 0,
                                 facecolor=bc['robot'], alpha=0.1,
                                 edgecolor=bc['robot'], linestyle='--',
                                 linewidth=1, zorder=1)
        ax.add_patch(rect)
        zone_patches[bot_id] = rect

    # Starting positions marker
    ax.plot(0, 0, 'o', color='#00FFFF', markersize=10, zorder=11)
    ax.text(0.05, 0.05, 'Bot1 Start', color='#00FFFF', fontsize=8)
    ax.plot(separation, 0, 'o', color='#FF00FF', markersize=10, zorder=11)
    ax.text(separation + 0.05, 0.05, 'Bot2 Start', color='#FF00FF', fontsize=8)

    ax.set_xlim(-3, 3)
    ax.set_ylim(-3, 3)
    ax.legend(loc='upper right', fontsize=8, facecolor='#1a1a2e',
              edgecolor='#333', labelcolor='white')

    # ── Main loop ──────────────────────────────────────────────────────────────
    pkt_count = {1: 0, 2: 0}
    last_zone_send = time.time()

    try:
        while True:
            try:
                data, addr = sock.recvfrom(65535)

                if len(data) != PACKET_SIZE:
                    continue

                u = struct.unpack(PACKET_FMT, data)
                magic, agent_id, rx, ry, ryaw, enc, v2v, \
                    d_front, d_left, d_back, d_right = u

                if magic != b'QSRL':
                    continue

                if agent_id not in [1, 2]:
                    print(f"[SKIP] Unknown agent_id: {agent_id}")
                    continue

                # Store bot address for zone responses
                bot_addrs[agent_id] = (addr[0], bot_ports[agent_id])

                # Apply Bot2 offset
                if agent_id == 2:
                    rx += separation

                pkt_count[agent_id] += 1
                now = time.time()

                # Log telemetry
                w_telem.writerow([
                    f"{now:.3f}", agent_id,
                    f"{rx:.4f}", f"{ry:.4f}",
                    f"{math.degrees(ryaw):.2f}", enc, v2v,
                    f"{d_front*100:.1f}", f"{d_left*100:.1f}",
                    f"{d_back*100:.1f}", f"{d_right*100:.1f}",
                ])
                f_telem.flush()

                # Update robot path
                path_x[agent_id].append(rx)
                path_y[agent_id].append(ry)
                path_lines[agent_id].set_data(path_x[agent_id], path_y[agent_id])

                # Robot marker
                t = (Affine2D().rotate(ryaw - math.pi/2)
                     + Affine2D().translate(rx, ry)
                     + ax.transData)
                rpatches[agent_id].set_transform(t)

                # Project sensor readings → world coordinates
                sensors = {
                    'front': d_front, 'left': d_left,
                    'back':  d_back,  'right': d_right
                }
                for name, dist in sensors.items():
                    if MIN_DIST_M < dist <= MAX_DIST_M:
                        ray_angle = ryaw + math.radians(SENSOR_ANGLES_DEG[name])
                        wx = rx + dist * math.cos(ray_angle)
                        wy = ry + dist * math.sin(ray_angle)
                        map_x[agent_id][name].append(wx)
                        map_y[agent_id][name].append(wy)
                        w_points.writerow([f"{now:.3f}", agent_id, name,
                                           f"{wx:.4f}", f"{wy:.4f}"])
                f_points.flush()

                # Update scatter plots
                for sensor in SENSOR_ANGLES_DEG:
                    if map_x[agent_id][sensor]:
                        scatters[agent_id][sensor].set_offsets(
                            np.column_stack([map_x[agent_id][sensor],
                                             map_y[agent_id][sensor]]))

                # ── Send territory zones to bots ──────────────────────────────
                if now - last_zone_send > ZONE_UPDATE_INTERVAL:
                    last_zone_send = now

                    # Bot1's territory → send to Bot2
                    all_pts_x_1 = sum(map_x[1].values(), []) + path_x[1]
                    all_pts_y_1 = sum(map_y[1].values(), []) + path_y[1]
                    zone1 = compute_bounding_box(all_pts_x_1, all_pts_y_1)

                    # Bot2's territory → send to Bot1
                    all_pts_x_2 = sum(map_x[2].values(), []) + path_x[2]
                    all_pts_y_2 = sum(map_y[2].values(), []) + path_y[2]
                    zone2 = compute_bounding_box(all_pts_x_2, all_pts_y_2)

                    # Send zone1 (Bot1's area) to Bot2
                    send_zone_to_bot(sock, bot_addrs[2], zone1)
                    # Send zone2 (Bot2's area) to Bot1
                    send_zone_to_bot(sock, bot_addrs[1], zone2)

                    # Update zone visualization
                    if zone1:
                        zone_patches[1].set_xy((zone1[0], zone1[1]))
                        zone_patches[1].set_width(zone1[2] - zone1[0])
                        zone_patches[1].set_height(zone1[3] - zone1[1])
                    if zone2:
                        zone_patches[2].set_xy((zone2[0], zone2[1]))
                        zone_patches[2].set_width(zone2[2] - zone2[0])
                        zone_patches[2].set_height(zone2[3] - zone2[1])

                # ── Auto-scale ─────────────────────────────────────────────────
                all_wx = (sum(map_x[1].values(), []) + sum(map_x[2].values(), [])
                          + path_x[1] + path_x[2])
                all_wy = (sum(map_y[1].values(), []) + sum(map_y[2].values(), [])
                          + path_y[1] + path_y[2])
                if len(all_wx) > 10:
                    ax.set_xlim(min(all_wx) - 0.5, max(all_wx) + 0.5)
                    ax.set_ylim(min(all_wy) - 0.5, max(all_wy) + 0.5)

                total_pts = sum(len(v) for b in map_x.values() for v in b.values())
                ax.set_title(
                    f"Dual-Bot Map | Bot1:{pkt_count[1]}pkts | Bot2:{pkt_count[2]}pkts | "
                    f"Points:{total_pts}", fontsize=11, color='white')

                fig.canvas.draw()
                fig.canvas.flush_events()

                # Terminal print
                bc = bot_colors[agent_id]
                print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] "
                      f"{bc['name']} | "
                      f"({rx:.2f},{ry:.2f},{math.degrees(ryaw):.0f}°) | "
                      f"F:{d_front*100:.0f}cm L:{d_left*100:.0f}cm "
                      f"B:{d_back*100:.0f}cm R:{d_right*100:.0f}cm")

            except BlockingIOError:
                plt.pause(0.02)
            except Exception as e:
                print(f"[ERR] {e}")

    except KeyboardInterrupt:
        total = pkt_count[1] + pkt_count[2]
        print(f"\n[EXIT] {total} packets logged "
              f"(Bot1:{pkt_count[1]}, Bot2:{pkt_count[2]}). Saving...")

        # Save merged point cloud
        all_x = sum(map_x[1].values(), []) + sum(map_x[2].values(), [])
        all_y = sum(map_y[1].values(), []) + sum(map_y[2].values(), [])
        if all_x:
            np.savetxt(os.path.join(ldir, "pointcloud_merged.csv"),
                       np.column_stack([all_x, all_y]),
                       delimiter=",", header="x,y", comments="")

        # Save per-bot point clouds
        for bot_id in [1, 2]:
            bx = sum(map_x[bot_id].values(), [])
            by = sum(map_y[bot_id].values(), [])
            if bx:
                np.savetxt(os.path.join(ldir, f"pointcloud_bot{bot_id}.csv"),
                           np.column_stack([bx, by]),
                           delimiter=",", header="x,y", comments="")

        print(f"[SAVED] {ldir}")
    finally:
        f_telem.close()
        f_points.close()
        sock.close()
        plt.close()


if __name__ == '__main__':
    main()
