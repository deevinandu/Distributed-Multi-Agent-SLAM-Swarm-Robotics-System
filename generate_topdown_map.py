import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import math

def draw_topdown_map(file_path):
    if not os.path.exists(file_path):
        print(f"[ERROR] File not found: {file_path}")
        return

    df = pd.read_csv(file_path)
    print(f"Mapping {len(df)} frames from {file_path}...")

    plt.figure(figsize=(10, 10))
    
    # Store all "hits" (X, Y coordinates of detected walls)
    all_x = []
    all_y = []

    range_cols = [f'r_{i}' for i in range(181)]

    for _, row in df.iterrows():
        rx, ry, ryaw = row['x'], row['y'], row['yaw']
        
        # Plot robot position
        plt.plot(rx, ry, 'ro', markersize=5) 
        
        # Convert each scan point to world X, Y
        for i in range(181):
            dist = row[f'r_{i}']
            if 0.1 < dist < 3.9: # Filter out noise and max range
                # World Angle = Robot Yaw + Scan Angle (-90 to +90)
                angle_rad = ryaw + math.radians(i - 90)
                
                # Trig: Polar to Cartesian
                world_x = rx + dist * math.cos(angle_rad)
                world_y = ry + dist * math.sin(angle_rad)
                
                all_x.append(world_x)
                all_y.append(world_y)

    # Plot the "Walls"
    plt.scatter(all_x, all_y, s=1, c='blue', alpha=0.5)
    
    plt.title("2D Floor Plan (Top-Down Map)")
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.grid(True)
    plt.axis('equal') # Keep the proportions correct
    
    print("Map generated. Opening window...")
    plt.show()

if __name__ == "__main__":
    draw_topdown_map("server_nodes/logs/dummy_agent_log.csv")
