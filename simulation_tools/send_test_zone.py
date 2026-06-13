import socket
import struct
import sys

TARGET_IP = "10.131.166.56"
TARGET_PORT = 8888

# Format: magic "ZONE", min_x, min_y, max_x, max_y
ZONE_FMT = '<4sffff'
pkt = struct.pack(ZONE_FMT, b'ZONE', 1.0, 2.0, 3.0, 4.0)

print(f"Sending test ZONE packet to Bot 1 at {TARGET_IP}:{TARGET_PORT}...")
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    sock.sendto(pkt, (TARGET_IP, TARGET_PORT))
    print("Packet sent successfully.")
except Exception as e:
    print(f"Error sending packet: {e}")
finally:
    sock.close()
