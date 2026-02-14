"""
Live Top-Down Persistent Mapper with Full Logging
- Creates a new log folder for each session
- Logs raw UDP packets, telemetry, and point cloud data to CSVs
- Visualizes real-time map
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
except:
    pass
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.transforms import Affine2D
import numpy as np

# Configuration
UDP_PORT = 8888
PACKET_FMT = '<4sBfffiIH181f'
PACKET_SIZE = struct.calcsize(PACKET_FMT)

# Filtering threshold
MAX_TRUST_DIST_M = 1.2  # Ignore > 1.2m
MIN_TRUST_DIST_M = 0.05 # Ignore < 5cm

# Triangle robot marker
TRIANGLE_SIZE = 0.12

def create_robot_triangle():
    return np.array([
        [0, TRIANGLE_SIZE * 0.7],
        [-TRIANGLE_SIZE * 0.5, -TRIANGLE_SIZE * 0.5],
        [TRIANGLE_SIZE * 0.5, -TRIANGLE_SIZE * 0.5]
    ])

def main():
    # --- SETUP LOGGING ---
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_dir = os.path.join("logs", f"session_{timestamp}")
    os.makedirs(log_dir, exist_ok=True)
    
    csv_telemetry_path = os.path.join(log_dir, "telemetry.csv")
    csv_points_path = os.path.join(log_dir, "pointcloud.csv")
    
    # Open CSV files
    f_telem = open(csv_telemetry_path, 'w', newline='')
    writer_telem = csv.writer(f_telem)
    writer_telem.writerow(['timestamp', 'agent_id', 'x', 'y', 'yaw', 'encoder', 'v2v_count', 'ranges_181'])
    
    f_points = open(csv_points_path, 'w', newline='')
    writer_points = csv.writer(f_points)
    writer_points.writerow(['timestamp', 'x', 'y'])

    print("=" * 60)
    print(f"  Live Mapper & Logger")
    print(f"  Listening on UDP Port: {UDP_PORT}")
    print(f"  Logging to: {log_dir}")
    print("=" * 60)

    # --- NETWORKING ---
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(('0.0.0.0', UDP_PORT))
    except Exception as e:
        print(f"[ERROR] Could not bind to port {UDP_PORT}: {e}")
        return
    sock.setblocking(False)

    # --- VISUALIZATION ---
    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_title(f"Live Map - Session {timestamp}", fontsize=14)
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.4)
    
    # Persistent Point Cloud (Global Map)
    cloud_scatter = ax.scatter([], [], s=4, c='black', alpha=0.6, label='Global Map (Persistent)')
    # Current Scan (Dynamic)
    scan_scatter = ax.scatter([], [], s=15, c='red', alpha=0.9, label='Latest Scan')
    # Robot Path
    robot_path, = ax.plot([], [], 'g-', linewidth=1, label='Path', alpha=0.5)
    
    # Robot Marker
    tri_verts = create_robot_triangle()
    robot_tri = patches.Polygon(tri_verts, closed=True, 
                                facecolor='lime', edgecolor='black', 
                                linewidth=1, zorder=10, label='Robot')
    ax.add_patch(robot_tri)
    
    # Data storage for plotting
    global_map_x = []
    global_map_y = []
    path_x, path_y = [], []
    
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.legend(loc='upper right')

    try:
        while True:
            try:
                data, addr = sock.recvfrom(65535)
                
                if len(data) != PACKET_SIZE:
                    continue

                recv_time = time.time()
                unpacked = struct.unpack(PACKET_FMT, data)
                magic = unpacked[0]
                if magic != b'QSRL': continue

                agent_id = unpacked[1]
                rx, ry, ryaw = unpacked[2], unpacked[3], unpacked[4]
                enc = unpacked[5]
                v2v_count = unpacked[6]
                ranges = list(unpacked[8:])

                # --- LOG TELEMETRY ---
                # Save ranges as a long string or individual columns? String is cleaner for now.
                writer_telem.writerow([recv_time, agent_id, rx, ry, ryaw, enc, v2v_count, str(ranges)])
                f_telem.flush()

                # --- UPDATE PATH ---
                path_x.append(rx)
                path_y.append(ry)
                robot_path.set_data(path_x, path_y)
                
                # --- UPDATE ROBOT MARKER ---
                t = Affine2D().rotate(ryaw - np.pi/2) + Affine2D().translate(rx, ry) + ax.transData
                robot_tri.set_transform(t)

                # --- PROCESS SCAN DATA ---
                current_x = []
                current_y = []
                
                for i, r in enumerate(ranges):
                    # Filter: 5cm < d < 1.2m
                    if MIN_TRUST_DIST_M < r <= MAX_TRUST_DIST_M:
                        # Angle logic: 
                        # Servo 0 (Right, -90 rel) ... 90 (Front, 0 rel) ... 180 (Left, +90 rel)
                        # Relative angle = (i - 90) degrees
                        rel_angle_rad = math.radians(i - 90)
                        
                        # Global angle = RobotYaw + RelativeAngle
                        # (Note: RobotYaw 0 points +X usually, but map logic assumes +Y is forward?
                        # Let's check AgentFirmware: 
                        # x += dist * cos(yaw), y += dist * sin(yaw).
                        # So Yaw=0 is +X (East).
                        
                        # BUT, scan logic usually treats "Forward" as aligned with Yaw.
                        # So Forward (servo 90) should be at global angle = Yaw.
                        # Right (servo 0) should be at global angle = Yaw - 90.
                        # Left (servo 180) should be at global angle = Yaw + 90.
                        # So: global_angle = Yaw + (i - 90) degrees.
                        
                        global_angle = ryaw + rel_angle_rad
                        
                        px = rx + r * math.cos(global_angle)
                        py = ry + r * math.sin(global_angle)
                        
                        current_x.append(px)
                        current_y.append(py)
                        
                        # Add to persistent map
                        global_map_x.append(px)
                        global_map_y.append(py)
                        
                        # Log point
                        writer_points.writerow([recv_time, px, py])
                
                f_points.flush() # Ensure data is saved immediately

                # --- UPDATE PLOTS ---
                if global_map_x:
                    cloud_scatter.set_offsets(np.column_stack([global_map_x, global_map_y]))
                
                if current_x:
                    scan_scatter.set_offsets(np.column_stack([current_x, current_y]))
                else:
                    scan_scatter.set_offsets(np.empty((0, 2)))

                # Auto-scale view
                if len(global_map_x) > 10:
                    all_x = global_map_x + path_x
                    all_y = global_map_y + path_y
                    ax.set_xlim(min(all_x)-0.5, max(all_x)+0.5)
                    ax.set_ylim(min(all_y)-0.5, max(all_y)+0.5)

                ax.set_title(f"Map | Pts: {len(global_map_x)} | Pose: ({rx:.2f},{ry:.2f})", fontsize=12)
                
                fig.canvas.draw()
                fig.canvas.flush_events()
                
                # Debug print to console (EVERY PACKET)
                valid_points = len(current_x)
                if ranges:
                    # Print simplified ranges for quick check: [Left (180), Front (90), Right (0)]
                    # Remember ranges[0] is Right, ranges[90] is Front, ranges[180] is Left
                    r_right = ranges[0]
                    r_front = ranges[90] if len(ranges) > 90 else 0
                    r_left = ranges[-1]
                    
                    print(f"[{datetime.datetime.now().strftime('%H:%M:%S')}] Agent {agent_id} | "
                          f"Pose: ({rx:.2f}, {ry:.2f}, {math.degrees(ryaw):.0f}°) | "
                          f"Enc: {enc} | "
                          f"Scan: {valid_points} valid pts | "
                          f"R: {r_right*100:.0f}cm F: {r_front*100:.0f}cm L: {r_left*100:.0f}cm")

            except BlockingIOError:
                plt.pause(0.01)
            except Exception as e:
                print(f"[ERROR] Loop error: {e}")

    except KeyboardInterrupt:
        print("\n[EXIT] Closing...")
    finally:
        f_telem.close()
        f_points.close()
        sock.close()
        plt.close()

if __name__ == '__main__':
    main()
