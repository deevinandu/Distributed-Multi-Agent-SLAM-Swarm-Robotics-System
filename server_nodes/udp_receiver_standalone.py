"""
Standalone Quasar-Lite Receiver
No ROS 2 Required - Just Python 3
"""
import socket
import struct

# Configuration
UDP_PORT = 8888
PACKET_FMT = '<4sBfffH181f'  # Little Endian: 4+1+4+4+4+2+724 = 743 bytes
PACKET_SIZE = struct.calcsize(PACKET_FMT)  # Should be 743

def main():
    print("=" * 50)
    print("Quasar-Lite Standalone Receiver")
    print(f"Listening on UDP Port: {UDP_PORT}")
    print(f"Expected Packet Size: {PACKET_SIZE} bytes")
    print("=" * 50)
    print("Waiting for packets from ESP32...\n")

    # Create UDP Socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', UDP_PORT))

    clients = {}

    while True:
        try:
            data, addr = sock.recvfrom(65535)
            
            if len(data) != PACKET_SIZE:
                print(f"[WARN] Invalid packet size: {len(data)} bytes (expected {PACKET_SIZE})")
                continue

            # Unpack Binary Data
            unpacked = struct.unpack(PACKET_FMT, data)
            
            magic = unpacked[0]
            if magic != b'QSRL':
                print(f"[WARN] Invalid magic bytes: {magic}")
                continue

            agent_id = unpacked[1]
            odom_x = unpacked[2]
            odom_y = unpacked[3]
            odom_yaw = unpacked[4]
            scan_count = unpacked[5]
            ranges = unpacked[6:]

            # Track new agents
            if agent_id not in clients:
                clients[agent_id] = addr
                print(f"[NEW] Agent {agent_id} connected from {addr}")

            # Print Summary
            valid_ranges = [r for r in ranges if r > 0.01]
            avg_range = sum(valid_ranges) / len(valid_ranges) if valid_ranges else 0
            
            # Convert yaw to degrees
            import math
            yaw_deg = math.degrees(odom_yaw)
            
            print(f"Agent {agent_id} | Pos: ({odom_x:+.3f}, {odom_y:+.3f}) | Yaw: {yaw_deg:+.1f}° | Avg Range: {avg_range:.2f}m")

            # --- NEW: SAVE TO CSV ---
            import os, csv, time
            if not os.path.exists('logs'): os.makedirs('logs')
            log_file = f"logs/agent_{agent_id}_log.csv"
            file_exists = os.path.isfile(log_file)
            
            with open(log_file, 'a', newline='') as f:
                writer = csv.writer(f)
                if not file_exists:
                    # Write Header
                    header = ['timestamp', 'idx', 'x', 'y', 'yaw'] + [f'r_{i}' for i in range(181)]
                    writer.writerow(header)
                
                # Write Data Row
                row = [time.time(), len(valid_ranges), odom_x, odom_y, odom_yaw] + list(ranges)
                writer.writerow(row)

        except KeyboardInterrupt:
            print("\n[EXIT] Shutting down...")
            break
        except Exception as e:
            print(f"[ERROR] {e}")

    sock.close()

if __name__ == '__main__':
    main()
