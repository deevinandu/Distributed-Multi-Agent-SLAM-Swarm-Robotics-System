"""
Room Mapper Visualizer
Shows ultrasonic scan data as a 2D polar plot (like radar)

Requirements: pip install matplotlib numpy
"""
import socket
import struct
import math
import matplotlib.pyplot as plt
import numpy as np
import matplotlib

# Try to force a standard backend for Windows
try:
    matplotlib.use('TkAgg')
except:
    pass

# QSRL (4s), AgentID (B), X/Y/Yaw (3f), Encoder (i), V2V_Count (I), ScanCount (H), Ranges (181f)
PACKET_FMT = '<4sBfffiIH181f'
PACKET_SIZE = struct.calcsize(PACKET_FMT)

def main():
    try:
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        print("[ERROR] Missing libraries. Run: pip install matplotlib numpy")
        return

    print("=" * 50)
    print("Room Mapper - Real-time Visualizer")
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

    print("[INFO] Opening Window...")
    # Setup Plot (Polar coordinates)
    plt.ion()  # Interactive mode
    try:
        fig, ax = plt.subplots(subplot_kw={'projection': 'polar'}, figsize=(8, 8))
    except Exception as e:
        print(f"[ERROR] Matplotlib window failed: {e}")
        print("Try: pip install PyQt5")
        return
    ax.set_title("Room Map (Ultrasonic Scan)", fontsize=14)
    ax.set_theta_zero_location('N')  # 0 degrees at top
    ax.set_theta_direction(-1)  # Clockwise
    ax.set_rlabel_position(45)
    ax.set_ylim(0, 4)  # Max range 4 meters
    
    # Angles for 181 points (0 to 180 degrees)
    # Offset by 90 degrees so 0 is forward, 90 is right, -90 is left
    angles_deg = np.linspace(-90, 90, 181)
    angles_rad = np.radians(angles_deg)
    
    # Initial empty plot
    line, = ax.plot(angles_rad, np.zeros(181), 'b-', linewidth=2)
    points = ax.scatter([], [], c='red', s=20)
    
    print("\nWaiting for scan data...")
    print("Press Ctrl+C to exit\n")

    try:
        while True:
            try:
                data, addr = sock.recvfrom(65535)
                
                if len(data) != PACKET_SIZE:
                    continue

                # Unpack
                unpacked = struct.unpack(PACKET_FMT, data)
                magic = unpacked[0]
                if magic != b'QSRL':
                    continue

                agent_id = unpacked[1]
                odom_x = unpacked[2]
                odom_y = unpacked[3]
                odom_yaw = unpacked[4]
                encoder_total = unpacked[5]
                v2v_count = unpacked[6]
                ranges = list(unpacked[8:])

                # Convert readings to NaN if they are outside the 1.2m trust zone
                # (Robot still uses them to drive, but we don't 'ink' them on the map)
                ranges_clean = [r if 0.01 < r <= 1.2 else np.nan for r in ranges]

                # Update plot
                line.set_ydata(ranges_clean)
                
                # Update scatter points for valid readings
                valid_angles = [angles_rad[i] for i, r in enumerate(ranges_clean) if not np.isnan(r)]
                valid_ranges = [r for r in ranges_clean if not np.isnan(r)]
                points.set_offsets(np.column_stack([valid_angles, valid_ranges]))

                ax.set_title(f"Map | Yaw: {math.degrees(odom_yaw):.1f}° | Swarm Link: {v2v_count}", fontsize=14)
                
                fig.canvas.draw()
                fig.canvas.flush_events()
                
                print(f"Agent {agent_id} | Link: {v2v_count} | Enc: {encoder_total} | Points: {len(valid_ranges)}")

            except BlockingIOError:
                plt.pause(0.01)  # Small pause when no data
            except Exception as e:
                print(f"[ERROR] Loop error: {e}")
                
    except KeyboardInterrupt:
        print("\n[EXIT] Closing visualizer...")
    
    sock.close()
    plt.close()

if __name__ == '__main__':
    main()
