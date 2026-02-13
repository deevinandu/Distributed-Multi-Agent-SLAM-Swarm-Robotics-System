import socket
import struct
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import math

# ================= CONFIGURATION =================
UDP_IP = "0.0.0.0" # Listen on all interfaces
UDP_PORT = 8888
BUFFER_SIZE = 4096

# ================= DATA STRUCTURES =================
# Must match the struct in AgentFirmware.ino
# struct QuasarPacket {
#     char magic[4];       // 4 bytes
#     uint8_t agent_id;    // 1 byte
#     float odom_x;        // 4 bytes
#     float odom_y;        // 4 bytes
#     float odom_yaw;      // 4 bytes
#     int32_t encoder_total; // 4 bytes
#     uint32_t v2v_count;  // 4 bytes
#     uint16_t scan_count; // 2 bytes
#     float ranges[181];   // 181 * 4 bytes
# };
RANGE_COUNT = 181
PACKET_FORMAT = f"<4sBfffIiH{RANGE_COUNT}f"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)

print(f"Expecting Packet Size: {PACKET_SIZE} bytes")

# ================= STATE =================
walls_x = []
walls_y = []
robot_path_x = []
robot_path_y = []

# ================= NETWORKING =================
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

print(f"Listening for UDP packets on {UDP_PORT}...")

# ================= VISUALIZATION =================
fig, ax = plt.subplots(figsize=(10, 10))
ax.set_title("Live Swarm Connectivity & Map (cm)")
ax.set_xlabel("X (cm)")
ax.set_ylabel("Y (cm)")
ax.axis('equal')
ax.grid(True)

# Plot elements
wall_scatter = ax.scatter([], [], c='black', s=5, label='Walls (Ultrasonic)')
path_plot, = ax.plot([], [], 'b-', linewidth=1, label='Path')
robot_marker, = ax.plot([], [], 'ro', markersize=10, label='Robot')
heading_line, = ax.plot([], [], 'r-', linewidth=2)

def update(frame):
    try:
        data, addr = sock.recvfrom(BUFFER_SIZE)
        
        if len(data) == PACKET_SIZE:
            unpacked = struct.unpack(PACKET_FORMAT, data)
            
            magic = unpacked[0]
            if magic != b'QSRL':
                return
                
            # Extract Telemetry (Convert to CM)
            agent_id = unpacked[1]
            odom_x = unpacked[2] * 100.0
            odom_y = unpacked[3] * 100.0
            odom_yaw = unpacked[4]
            # ... skip others ...
            scan_count = unpacked[9]
            ranges = unpacked[10:]
            
            # Update Path
            robot_path_x.append(odom_x)
            robot_path_y.append(odom_y)
            
            # Process Scan Data -> Wall Points
            # Sensor is mounted on the servo.
            # Servo 0-180. 
            # 90 is usually "Forward".
            # 0 is Right, 180 is Left (depending on servo install)
            
            for i in range(len(ranges)):
                dist_m = ranges[i]
                if dist_m > 0.05 and dist_m < 3.0: # Filter noise
                    dist_cm = dist_m * 100.0
                    
                    # Angle logic:
                    # If Servo 90 is Forward (0 relative to robot)
                    # Servo Angle 'i' (0-180)
                    # Sensor Angle Relative to Robot = (i - 90) * (PI/180)
                    # Global Angle = Robot Yaw + Sensor Angle
                    
                    sensor_angle_deg = i - 90
                    sensor_angle_rad = math.radians(sensor_angle_deg)
                    global_angle = odom_yaw + sensor_angle_rad
                    
                    # Convert to Cartesian
                    wx = odom_x + dist_cm * math.cos(global_angle)
                    wy = odom_y + dist_cm * math.sin(global_angle)
                    
                    walls_x.append(wx)
                    walls_y.append(wy)
            
            # Update Plots
            wall_scatter.set_offsets(np.c_[walls_x, walls_y])
            path_plot.set_data(robot_path_x, robot_path_y)
            robot_marker.set_data([odom_x], [odom_y])
            
            # Heading Indicator (30cm magnitude)
            hf_x = odom_x + 30.0 * math.cos(odom_yaw)
            hf_y = odom_y + 30.0 * math.sin(odom_yaw)
            heading_line.set_data([odom_x, hf_x], [odom_y, hf_y])
            
            # Dynamic Zoom?
            # ax.set_xlim(odom_x - 500, odom_x + 500)
            # ax.set_ylim(odom_y - 500, odom_y + 500)
            
    except BlockingIOError:
        pass
    except Exception as e:
        print(f"Error: {e}")

    return wall_scatter, path_plot, robot_marker, heading_line

anim = FuncAnimation(fig, update, interval=100) # 10Hz update
plt.legend(loc='upper right')
plt.show()
