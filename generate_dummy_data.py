import csv
import math
import random
import time
import os

# Configuration matching the real system
LOG_DIR = "server_nodes/logs"
LOG_FILE = f"{LOG_DIR}/dummy_agent_log.csv"

def generate_square_room_scan(robot_x, robot_y, robot_yaw):
    """
    Simulates a scan in a 4mx4m room (centered at 0,0)
    """
    ranges = []
    for degree in range(181):
        # Convert scan angle to world angle
        angle_rad = robot_yaw + math.radians(degree - 90)
        
        # Simple Ray-Box Intersection for a 4m x 4m room
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        dist = 4.0 # Default max
        
        if cos_a != 0:
            tx1 = (-2 - robot_x) / cos_a
            tx2 = (2 - robot_x) / cos_a
            if tx1 > 0: dist = min(dist, tx1)
            if tx2 > 0: dist = min(dist, tx2)
            
        if sin_a != 0:
            ty1 = (-2 - robot_y) / sin_a
            ty2 = (2 - robot_y) / sin_a
            if ty1 > 0: dist = min(dist, ty1)
            if ty2 > 0: dist = min(dist, ty2)
            
        dist += random.uniform(-0.02, 0.02) # Noise
        ranges.append(dist)
    return ranges

def main():
    if not os.path.exists(LOG_DIR):
        os.makedirs(LOG_DIR)
        
    print(f"Generating dummy data in {LOG_FILE}...")
    
    with open(LOG_FILE, 'w', newline='') as f:
        writer = csv.writer(f)
        # Added 'encoder' column
        header = ['timestamp', 'idx', 'x', 'y', 'yaw', 'encoder'] + [f'r_{i}' for i in range(181)]
        writer.writerow(header)
        
        for step in range(20):
            t = time.time()
            x = (step / 20.0)
            y = (step / 20.0)
            yaw = (step / 20.0) * math.pi
            encoder = step * 15 # Simulate encoder counts
            
            ranges = generate_square_room_scan(x, y, yaw)
            
            row = [t, 181, x, y, yaw, encoder] + ranges
            writer.writerow(row)
            print(f"  Step {step+1}/20 saved.")
            
    print("\nDummy map data generation complete!")

if __name__ == "__main__":
    main()
