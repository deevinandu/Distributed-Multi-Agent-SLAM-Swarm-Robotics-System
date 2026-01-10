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
        # Boundaries: x = -2, x = 2, y = -2, y = 2
        
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        
        # Possible intersection distances
        dist = 4.0 # Default max
        
        # Intersect with x = -2 and x = 2
        if cos_a != 0:
            tx1 = (-2 - robot_x) / cos_a
            tx2 = (2 - robot_x) / cos_a
            if tx1 > 0: dist = min(dist, tx1)
            if tx2 > 0: dist = min(dist, tx2)
            
        # Intersect with y = -2 and y = 2
        if sin_a != 0:
            ty1 = (-2 - robot_y) / sin_a
            ty2 = (2 - robot_y) / sin_a
            if ty1 > 0: dist = min(dist, ty1)
            if ty2 > 0: dist = min(dist, ty2)
            
        # Add some noise
        dist += random.uniform(-0.02, 0.02)
        ranges.append(dist)
        
    return ranges

def main():
    if not os.path.exists(LOG_DIR):
        os.makedirs(LOG_DIR)
        
    print(f"Generating dummy data in {LOG_FILE}...")
    
    with open(LOG_FILE, 'w', newline='') as f:
        writer = csv.writer(f)
        header = ['timestamp', 'idx', 'x', 'y', 'yaw'] + [f'r_{i}' for i in range(181)]
        writer.writerow(header)
        
        # Simulate a path: moving from (0,0) to (1,1)
        for step in range(20):
            t = time.time()
            x = (step / 20.0)
            y = (step / 20.0)
            # Slowly rotating
            yaw = (step / 20.0) * math.pi
            
            ranges = generate_square_room_scan(x, y, yaw)
            
            row = [t, 181, x, y, yaw] + ranges
            writer.writerow(row)
            print(f"  Step {step+1}/20 saved.")
            
    print("\nDummy map data generation complete!")
    print("You can now open this CSV or run 'python server_nodes/room_mapper.py' to see it (if you modify the mapper to read files).")

if __name__ == "__main__":
    main()
