"""
Live Top-Down Cartesian Mapper
Visualizes the robot's path and environment in absolute X,Y coordinates.

Requirements: pip install matplotlib numpy
"""
import socket
import struct
import math
import matplotlib.pyplot as plt
import numpy as np
import matplotlib

# Force TkAgg for Windows stability
try:
    matplotlib.use('TkAgg')
except:
    pass

# Configuration
UDP_PORT = 8888
# QSRL (4s), AgentID (B), X/Y/Yaw (3f), Encoder (i), V2V_Count (I), ScanCount (H), Ranges (181f)
PACKET_FMT = '<4sBfffiIH181f'
PACKET_SIZE = struct.calcsize(PACKET_FMT)

def main():
    print("=" * 50)
    print("Live Top-Down Mapper - Waiting for Agent...")
    print(f"Listening on UDP Port: {UDP_PORT}")
    print("=" * 50)

    # Setup UDP
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(('0.0.0.0', UDP_PORT))
    except Exception as e:
        print(f"[ERROR] Could not bind to port {UDP_PORT}: {e}")
        return
    sock.setblocking(False)

    # Setup Plot (Cartesian)
    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_title("Live 2D Floor Plan", fontsize=14)
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.set_aspect('equal') # Maintain proportions
    
    # Visualization elements
    robot_path, = ax.plot([], [], 'r-', label='Robot Path', alpha=0.8)
    robot_pos, = ax.plot([], [], 'ro', markersize=8, label='Current Pos')
    wall_points = ax.scatter([], [], s=2, c='blue', alpha=0.5, label='Detected Walls')
    
    # Buffers
    path_x, path_y = [], []
    all_walls_x, all_walls_y = [], []
    
    # Set initial limit (will auto-scale)
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.legend(loc='upper right')

    try:
        while True:
            try:
                data, addr = sock.recvfrom(65535)
                
                if len(data) != PACKET_SIZE:
                    continue

                # Unpack
                unpacked = struct.unpack(PACKET_FMT, data)
                magic = unpacked[0]
                if magic != b'QSRL': continue

                agent_id = unpacked[1]
                rx, ry, ryaw = unpacked[2], unpacked[3], unpacked[4]
                v2v_count = unpacked[6]
                ranges = list(unpacked[8:])

                # 1. Update Path
                path_x.append(rx)
                path_y.append(ry)
                robot_path.set_data(path_x, path_y)
                robot_pos.set_data([rx], [ry])

                # 2. Update Map (Polar -> Cartesian Transformation)
                for i, dist in enumerate(ranges):
                    # TRUST FILTER: Only plot points between 10cm and 1.2m
                    if 0.1 < dist <= 1.2:
                        # World Angle = Robot Yaw + Scan Angle (-90 to +90)
                        angle_world = ryaw + math.radians(i - 90)
                        
                        # Calculate absolute coordinate of the wall hit
                        wx = rx + dist * math.cos(angle_world)
                        wy = ry + dist * math.sin(angle_world)
                        
                        all_walls_x.append(wx)
                        all_walls_y.append(wy)

                # Update scatter plot (sampling to prevent slowdown)
                if len(all_walls_x) > 0:
                    wall_points.set_offsets(np.column_stack([all_walls_x, all_walls_y]))

                # 3. Dynamic Title & Scaling
                ax.set_title(f"Live Map | Link: {v2v_count} | Pos: ({rx:.2f}, {ry:.2f})", fontsize=14)
                
                # Auto-scale view to fit robot movement
                if len(path_x) > 5:
                    ax.set_xlim(min(path_x)-1.5, max(path_x)+1.5)
                    ax.set_ylim(min(path_y)-1.5, max(path_y)+1.5)

                fig.canvas.draw()
                fig.canvas.flush_events()
                
                print(f"Agent {agent_id} | Link: {v2v_count} | Path Points: {len(path_x)} | Wall Points: {len(all_walls_x)}")

            except BlockingIOError:
                plt.pause(0.01)
            except Exception as e:
                print(f"[ERROR] Loop error: {e}")
                
    except KeyboardInterrupt:
        print("\n[EXIT] Closing Live Mapper...")
    
    sock.close()
    plt.close()

if __name__ == '__main__':
    main()
