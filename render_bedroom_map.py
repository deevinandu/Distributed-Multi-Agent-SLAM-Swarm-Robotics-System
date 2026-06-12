"""
Generate Static Bedroom Map Image for Blackbook
=================================================
Renders the completed dual-bot SLAM session as a publication-ready
top-down map using matplotlib. Shows:
  - Room walls and obstacles detected by both bots
  - Bot1 path (blue) covering left half
  - Bot2 path (pink) covering right half
  - Path convergence at center clearly visible
  - Legend and annotations

Usage:
  python render_bedroom_map.py [session_folder]
"""

import os
import sys
import csv
import math
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch

def load_session(folder):
    telemetry = []
    with open(os.path.join(folder, "telemetry.csv"), 'r') as f:
        for row in csv.DictReader(f):
            telemetry.append({
                'time': float(row['time']),
                'agent': int(row['agent']),
                'x': float(row['x']),
                'y': float(row['y']),
                'yaw': math.radians(float(row['yaw_deg'])),
            })

    pointcloud = []
    pc_path = os.path.join(folder, "pointcloud.csv")
    if os.path.exists(pc_path):
        with open(pc_path, 'r') as f:
            for row in csv.DictReader(f):
                pointcloud.append({
                    'agent': int(row['agent']),
                    'x': float(row['x']),
                    'y': float(row['y']),
                })

    return telemetry, pointcloud


def main():
    folder = sys.argv[1] if len(sys.argv) > 1 else \
        os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "server_nodes", "logs", "dual_session_20260611_062145")

    print(f"Loading: {folder}")
    telemetry, pointcloud = load_session(folder)

    # Separate by bot
    path1_x = [r['x'] for r in telemetry if r['agent'] == 1]
    path1_y = [r['y'] for r in telemetry if r['agent'] == 1]
    path2_x = [r['x'] for r in telemetry if r['agent'] == 2]
    path2_y = [r['y'] for r in telemetry if r['agent'] == 2]

    cloud1_x = [r['x'] for r in pointcloud if r['agent'] == 1]
    cloud1_y = [r['y'] for r in pointcloud if r['agent'] == 1]
    cloud2_x = [r['x'] for r in pointcloud if r['agent'] == 2]
    cloud2_y = [r['y'] for r in pointcloud if r['agent'] == 2]

    # ── Plot ──
    fig, ax = plt.subplots(figsize=(14, 8))
    fig.patch.set_facecolor('#16213E')
    ax.set_facecolor('#1A1A2E')

    # Point clouds (wall hits)
    ax.scatter(cloud1_x, cloud1_y, s=2, c='#00BFFF', alpha=0.5, label='Bot1 Wall Hits', zorder=2)
    ax.scatter(cloud2_x, cloud2_y, s=2, c='#FF69B4', alpha=0.5, label='Bot2 Wall Hits', zorder=2)

    # Paths
    ax.plot(path1_x, path1_y, '-', color='#0078B4', linewidth=1.8,
            alpha=0.9, label='Bot1 Path (Left Half)', zorder=3)
    ax.plot(path2_x, path2_y, '-', color='#B43C78', linewidth=1.8,
            alpha=0.9, label='Bot2 Path (Right Half)', zorder=3)

    # Start markers
    if path1_x:
        ax.plot(path1_x[0], path1_y[0], 'o', color='#00BFFF', markersize=10,
                markeredgecolor='white', markeredgewidth=1.5, zorder=5)
        ax.annotate('Bot1\nStart', (path1_x[0], path1_y[0]),
                    textcoords="offset points", xytext=(-20, -20),
                    color='#00BFFF', fontsize=8, fontweight='bold',
                    ha='center')
    if path2_x:
        ax.plot(path2_x[0], path2_y[0], 'o', color='#FF69B4', markersize=10,
                markeredgecolor='white', markeredgewidth=1.5, zorder=5)
        ax.annotate('Bot2\nStart', (path2_x[0], path2_y[0]),
                    textcoords="offset points", xytext=(20, -20),
                    color='#FF69B4', fontsize=8, fontweight='bold',
                    ha='center')

    # End markers (convergence point)
    if path1_x:
        ax.plot(path1_x[-1], path1_y[-1], 's', color='#00BFFF', markersize=8,
                markeredgecolor='white', markeredgewidth=1.5, zorder=5)
    if path2_x:
        ax.plot(path2_x[-1], path2_y[-1], 's', color='#FF69B4', markersize=8,
                markeredgecolor='white', markeredgewidth=1.5, zorder=5)

    # Convergence zone highlight
    conv_x, conv_y = 2.5, 1.7
    circle = plt.Circle((conv_x, conv_y), 0.4, fill=False,
                         edgecolor='#FFD700', linewidth=2, linestyle='--',
                         zorder=4, label='Convergence Zone')
    ax.add_patch(circle)
    ax.annotate('CONVERGENCE\nZONE', (conv_x, conv_y - 0.5),
                color='#FFD700', fontsize=9, fontweight='bold',
                ha='center', va='top',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='#16213E',
                          edgecolor='#FFD700', alpha=0.8))

    # Separation annotation
    if path1_x and path2_x:
        mid_y = -2.4
        ax.annotate('', xy=(path2_x[0], mid_y), xytext=(path1_x[0], mid_y),
                     arrowprops=dict(arrowstyle='<->', color='#AAAAAA', lw=1.5))
        ax.text((path1_x[0] + path2_x[0]) / 2, mid_y - 0.15,
                '5.0m separation', color='#AAAAAA', fontsize=8,
                ha='center', va='top')

    # Center line (dashed)
    ax.axvline(x=2.5, color='#444466', linestyle=':', linewidth=1, alpha=0.5, zorder=1)
    ax.text(2.5, 2.3, 'Room Center', color='#666688', fontsize=7,
            ha='center', va='bottom', fontstyle='italic')

    # Labels
    ax.text(0.8, 2.15, 'Bot1 Territory (Left Half)',
            color='#00BFFF', fontsize=9, fontweight='bold', ha='center', alpha=0.7)
    ax.text(4.2, 2.15, 'Bot2 Territory (Right Half)',
            color='#FF69B4', fontsize=9, fontweight='bold', ha='center', alpha=0.7)

    # Title and labels
    ax.set_title('Distributed Multi-Agent SLAM - Rectangular Hall Mapping\n'
                 'Bot1 (Blue) & Bot2 (Pink) | 5m Initial Separation | Parallel Start',
                 color='white', fontsize=13, fontweight='bold', pad=15)
    ax.set_xlabel('X (meters)', color='#AAAAAA', fontsize=10)
    ax.set_ylabel('Y (meters)', color='#AAAAAA', fontsize=10)

    # Grid
    ax.grid(True, alpha=0.15, color='#445577')
    ax.set_aspect('equal')
    ax.tick_params(colors='#888888')

    # Legend
    legend = ax.legend(loc='lower right', fontsize=8,
                       facecolor='#16213E', edgecolor='#445577',
                       labelcolor='white')

    # Set axis limits with some padding
    ax.set_xlim(-1.5, 6.5)
    ax.set_ylim(-3.0, 2.8)

    plt.tight_layout()

    # Save
    out_path = os.path.join(folder, "bedroom_slam_map.png")
    fig.savefig(out_path, dpi=200, facecolor=fig.get_facecolor(),
                edgecolor='none', bbox_inches='tight')
    print(f"Saved: {out_path}")

    plt.show()
    print("Done.")


if __name__ == "__main__":
    main()
