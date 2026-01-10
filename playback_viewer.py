"""
Quasar-Lite Log Playback Viewer
Reads agent_log.csv and displays it as a 2D scan
"""
import pandas as pd
import matplotlib
# Try to force a standard backend
try:
    matplotlib.use('TkAgg')
except:
    pass
import matplotlib.pyplot as plt
import numpy as np
import time
import sys
import os

def playback_csv(file_path):
    if not os.path.exists(file_path):
        print(f"[ERROR] File not found: {file_path}")
        return

    print(f"Loading data from {file_path}...")
    try:
        df = pd.read_csv(file_path)
    except Exception as e:
        print(f"[ERROR] Failed to read CSV: {e}")
        return
    
    if df.empty:
        print("[ERROR] File is empty!")
        return

    # Setup Plot
    try:
        plt.ion()
        fig, ax = plt.subplots(subplot_kw={'projection': 'polar'}, figsize=(8, 8))
    except Exception as e:
        print(f"[ERROR] Could not create plot window: {e}")
        return
    ax.set_theta_zero_location('N')
    ax.set_theta_direction(-1)
    ax.set_ylim(0, 4)

    angles_deg = np.linspace(-90, 90, 181)
    angles_rad = np.radians(angles_deg)
    range_cols = [f'r_{i}' for i in range(181)]
    
    line, = ax.plot(angles_rad, np.zeros(181), 'b-', alpha=0.5)
    points = ax.scatter([], [], c='red', s=10)

    print(f"Starting playback of {len(df)} frames...")
    
    for idx, row in df.iterrows():
        ranges = row[range_cols].values
        ranges_clean = np.where(ranges > 0.01, ranges, np.nan)
        
        line.set_ydata(ranges_clean)
        valid_mask = ~np.isnan(ranges_clean)
        points.set_offsets(np.column_stack([angles_rad[valid_mask], ranges_clean[valid_mask]]))
        
        # Display encoder if available
        enc_str = f" | Enc: {int(row['encoder'])}" if 'encoder' in row else ""
        ax.set_title(f"Frame {idx+1}/{len(df)} | Pos: ({row['x']:.2f}, {row['y']:.2f}){enc_str}", fontsize=12)
        
        fig.canvas.draw()
        fig.canvas.flush_events()
        time.sleep(0.1)

    print("\nPlayback finished. Keeping window open...")
    plt.ioff()
    plt.show()

if __name__ == "__main__":
    target = "server_nodes/logs/dummy_agent_log.csv"
    if len(sys.argv) > 1:
        target = sys.argv[1]
    playback_csv(target)
