"""
EMERGENCY DEBUG RECEIVER
Run this to test if packets are arriving
"""
import socket

print("=" * 50)
print("DEBUG MODE: Listening for ANY UDP on port 8888")
print("=" * 50)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(30)  # 30 second timeout

try:
    sock.bind(('0.0.0.0', 8888))
    print("[OK] Socket bound to port 8888")
except Exception as e:
    print(f"[FAIL] Cannot bind: {e}")
    print("       Another program may be using this port!")
    exit(1)

print("\nWaiting for packets (30 second timeout)...")
print("If nothing happens, check:")
print("  1. Firewall is disabled or allows UDP 8888")
print("  2. ESP32 and PC are on SAME WiFi")
print("  3. Try exact IP instead of broadcast\n")

try:
    data, addr = sock.recvfrom(65535)
    print(f"\n[SUCCESS!] Received {len(data)} bytes from {addr}")
    print(f"First 20 bytes: {data[:20]}")
    print(f"Magic bytes: {data[:4]}")
except socket.timeout:
    print("\n[TIMEOUT] No packets received in 30 seconds.")
    print("\nTROUBLESHOOTING:")
    print("1. DISABLE Windows Firewall temporarily:")
    print("   > Open 'Windows Security' > Firewall > Turn off")
    print("2. Or run PowerShell as Admin:")
    print("   > netsh advfirewall set allprofiles state off")
except Exception as e:
    print(f"[ERROR] {e}")

sock.close()
