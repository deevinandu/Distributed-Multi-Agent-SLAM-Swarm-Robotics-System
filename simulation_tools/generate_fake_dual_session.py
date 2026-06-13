"""
Generate Fake Dual-Bot Bedroom SLAM Session
=============================================
Simulates two bots mapping a bedroom with a bed in the center.

Room layout (top-down view):
  ┌──────────────────────────────┐  y = 2.0
  │                              │
  │      ┌──────────────┐        │
  │      │              │        │
  │      │     BED      │        │
  │      │              │        │
  │      └──────────────┘        │
  │                              │
  │  Bot1(0,0)      Bot2(5,0)    │
  └──────D───────────────────────┘  y = -2.0
         (door)

  x = -0.5                x = 5.5

Both bots start at the bottom of the room, 5 meters apart.
They initially move PARALLEL (both head north).
Bot1 maps the LEFT half, Bot2 maps the RIGHT half.
Their paths converge near x ≈ 2.5 (room center) to prove
distributed SLAM with map merging.

Usage:
  python generate_fake_dual_session.py
"""

import os
import csv
import math
import random
import numpy as np

# ==============================================================================
#  BEDROOM GEOMETRY  (walls as line segments)
# ==============================================================================

# Room: 6m wide (x: -0.5 to 5.5) x 4m deep (y: -2.0 to 2.0)
# Bot1 at (0, 0), Bot2 at (5, 0), door on bottom wall near Bot1

WALLS = [
    # ── Outer room walls ──
    # Bottom wall (solid rectangular hall)
    ((-0.5, -2.0), (5.5, -2.0)),
    # Right wall
    ((5.5, -2.0),  (5.5,  2.0)),
    # Top wall
    ((5.5,  2.0),  (-0.5, 2.0)),
    # Left wall
    ((-0.5, 2.0),  (-0.5, -2.0)),
]

MAX_SENSOR_RANGE = 1.20  # meters
MIN_SENSOR_RANGE = 0.05

SENSOR_ANGLES = {
    'front': 0.0,
    'left':  math.pi / 2,
    'back':  math.pi,
    'right': -math.pi / 2,
}


def ray_segment_intersect(ox, oy, angle, sx1, sy1, sx2, sy2):
    """Find distance from (ox,oy) along ray at `angle` to segment."""
    dx = math.cos(angle)
    dy = math.sin(angle)
    dsx = sx2 - sx1
    dsy = sy2 - sy1
    denom = dx * dsy - dy * dsx
    if abs(denom) < 1e-10:
        return None
    t = ((sx1 - ox) * dsy - (sy1 - oy) * dsx) / denom
    u = ((sx1 - ox) * dy  - (sy1 - oy) * dx)  / denom
    if t > 0.001 and 0.0 <= u <= 1.0:
        return t
    return None


def cast_ray(ox, oy, angle):
    """Cast a ray and return distance to nearest wall/obstacle."""
    min_dist = float('inf')
    for (sx1, sy1), (sx2, sy2) in WALLS:
        d = ray_segment_intersect(ox, oy, angle, sx1, sy1, sx2, sy2)
        if d is not None and d < min_dist:
            min_dist = d
    return min_dist if min_dist < 50.0 else 99.0


def get_sensor_readings(x, y, yaw):
    """Get 4 sensor distances with realistic noise."""
    readings = {}
    for name, rel_angle in SENSOR_ANGLES.items():
        world_angle = yaw + rel_angle
        true_dist = cast_ray(x, y, world_angle)

        # Realistic ultrasonic sensor noise (~35mm std dev)
        noise = random.gauss(0, 0.035)
        noisy_dist = true_dist + noise

        # Occasional spurious reading/reflection (6% chance)
        if random.random() < 0.06:
            noisy_dist = random.uniform(0.02, 2.5)

        noisy_dist = max(0.01, noisy_dist)
        readings[name] = noisy_dist
    return readings


def get_landmark_type(readings):
    """Determine landmark type. 0=NONE, 1=CORNER_L, 2=CORNER_R, 3=CORRIDOR, 4=DEAD_END, 5=OPEN"""
    f = readings['front']
    l = readings['left']
    r = readings['right']
    close = 0.30
    if f < close and l < close and r > close:
        return 1
    if f < close and r < close and l > close:
        return 2
    if l < close and r < close and f > close:
        return 3
    if f < close and l < close and r < close:
        return 4
    if f > MAX_SENSOR_RANGE and l > MAX_SENSOR_RANGE and r > MAX_SENSOR_RANGE:
        return 5
    return 0


# ==============================================================================
#  BOT TRAJECTORIES
# ==============================================================================

# Bot1: starts at (0, 0), maps LEFT half of room
BOT1_WAYPOINTS = [
    # Start
    (0.0,   0.0,    90),    # start, facing north
    (0.0,   0.4,    90),    # move north
    (0.0,   0.9,    90),    # continue north
    (0.0,   1.3,    90),    # approaching top wall area

    # Turn west to left wall
    (0.0,   1.3,   180),    # turn to face west
    (-0.2,  1.3,   180),    # drive toward left wall
    (-0.2,  1.3,    90),    # turn north
    (-0.2,  1.7,    90),    # drive north near left wall

    # Turn east, sweep along top wall to x=2.45
    (-0.2,  1.7,     0),    # turn east
    (0.4,   1.7,     0),    # drive east along top wall
    (0.9,   1.7,     0),
    (1.4,   1.7,     0),
    (1.9,   1.7,     0),
    (2.45,  1.7,     0),    # stop sweeping top wall at x=2.45

    # Move down along x=2.45
    (2.45,   1.7,   -90),    # turn south
    (2.45,   1.1,   -90),    # drive south
    (2.45,   0.5,   -90),
    (2.45,  -0.1,   -90),
    (2.45,  -0.7,   -90),
    (2.45,  -1.3,   -90),
    (2.45,  -1.7,   -90),    # reach bottom-center-left

    # Turn west, sweep along bottom wall
    (2.45,  -1.7,   180),    # turn west
    (1.8,  -1.7,   180),    # drive west along bottom wall
    (1.2,  -1.7,   180),
    (0.6,  -1.7,   180),
    (0.0,  -1.7,   180),    # reach bottom-left

    # Turn north, go back to start
    (0.0,  -1.7,    90),    # turn north
    (0.0,  -1.1,    90),    # drive north
    (0.0,  -0.5,    90),
    (0.0,   0.0,    90),    # back to start!
]

BOT2_WAYPOINTS = [
    # Start
    (5.0,   0.0,    90),    # start, facing north
    (5.0,   0.4,    90),    # move north
    (5.0,   0.9,    90),    # continue north
    (5.0,   1.3,    90),    # same Y as Bot1's parallel segment

    # Turn east to right wall
    (5.0,   1.3,     0),    # turn to face east
    (5.2,   1.3,     0),    # drive toward right wall
    (5.2,   1.3,    90),    # turn north
    (5.2,   1.7,    90),    # drive north near right wall

    # Turn west, sweep along top wall to x=3.0
    (5.2,   1.7,   180),    # turn west
    (4.6,   1.7,   180),    # drive west along top wall
    (4.0,   1.7,   180),
    (3.5,   1.7,   180),
    (3.0,   1.7,   180),    # stop sweeping top wall at x=3.0

    # Move down along x=3.0
    (3.0,   1.7,   -90),    # turn south
    (3.0,   1.1,   -90),    # drive south
    (3.0,   0.5,   -90),
    (3.0,  -0.1,   -90),
    (3.0,  -0.7,   -90),
    (3.0,  -1.3,   -90),
    (3.0,  -1.7,   -90),    # reach bottom-center-right

    # Turn east, sweep along bottom wall
    (3.0,  -1.7,     0),    # turn east
    (3.5,  -1.7,     0),    # drive east along bottom wall
    (4.0,  -1.7,     0),
    (4.5,  -1.7,     0),
    (5.0,  -1.7,     0),    # reach bottom-right

    # Turn north, go back to start
    (5.0,  -1.7,    90),    # turn north
    (5.0,  -1.1,    90),    # drive north
    (5.0,  -0.5,    90),
    (5.0,   0.0,    90),    # back to start!
]


def interpolate_waypoints(waypoints, steps_per_meter=25):
    """Interpolate between waypoints with realistic wobbly physical steering when near a wall."""
    poses = []
    random_state = random.Random(42)  # Use local random for repeatability
    
    # State for feedback-driven wall-following wiggles
    lat_offset = 0.0
    steer_dir = 1.0  # +1.0 = steering away from wall, -1.0 = steering towards wall
    
    for i in range(len(waypoints) - 1):
        x1, y1, yaw1 = waypoints[i]
        x2, y2, yaw2 = waypoints[i + 1]
        yaw1_r = math.radians(yaw1)
        yaw2_r = math.radians(yaw2)

        dx = x2 - x1
        dy = y2 - y1
        dist = math.sqrt(dx**2 + dy**2)

        # More steps for longer segments, fewer for turns
        if dist < 0.05:
            n_steps = 4  # pure rotation
            for j in range(n_steps):
                t = j / n_steps
                # Interpolate yaw with wraparound
                dyaw = yaw2_r - yaw1_r
                while dyaw > math.pi:   dyaw -= 2 * math.pi
                while dyaw < -math.pi:  dyaw += 2 * math.pi
                yaw = yaw1_r + t * dyaw + random_state.gauss(0, 0.03)
                poses.append((x1, y1, yaw))
        else:
            n_steps = max(5, int(dist * steps_per_meter))
            
            # Unit direction and normal
            ux = dx / dist
            uy = dy / dist
            nx = -uy
            ny = ux
            
            # Compute segment yaw
            seg_yaw = math.atan2(dy, dx)
            
            for j in range(n_steps):
                t = j / n_steps
                # Clean interpolated position
                px = x1 + t * dx
                py = y1 + t * dy
                
                # Check if we are close to a wall (top wall, bottom wall, left wall, or right wall)
                is_following_wall = (py > 1.3) or (py < -1.3) or (px < -0.1) or (px > 5.1)
                
                if is_following_wall:
                    # Hysteresis controller modeling wall too close-far threshold logic.
                    # Clean path is 25-30cm from the wall. We wiggle between thresholds.
                    # -0.15m corresponds to being too far from the wall.
                    # +0.15m corresponds to being too close to the wall.
                    threshold_close = -0.15
                    threshold_far = 0.15
                    if lat_offset < threshold_close:
                        steer_dir = 1.0  # Drift away / steer away from wall
                    elif lat_offset > threshold_far:
                        steer_dir = -1.0 # Drift towards / steer towards wall
                    
                    # Proportional-like steering updates with noise
                    lat_offset += steer_dir * 0.012 + random_state.gauss(0, 0.003)
                    lat_offset = max(-0.20, min(0.20, lat_offset))
                    
                    # Yaw error corresponds to the steering correction angle
                    yaw_err = -steer_dir * 0.22 + random_state.gauss(0, 0.03)
                    long_offset = random_state.gauss(0, 0.004)
                else:
                    # Straight trajectory with minor execution noise in open space.
                    # Smoothly decay any lingering lateral offset back to 0.
                    lat_offset = lat_offset * 0.9 + random_state.gauss(0, 0.002)
                    yaw_err = random_state.gauss(0, 0.005)
                    long_offset = random_state.gauss(0, 0.002)
                
                x = px + lat_offset * nx + long_offset * ux
                y = py + lat_offset * ny + long_offset * uy
                yaw = seg_yaw + yaw_err
                
                poses.append((x, y, yaw))

    # Final pose
    xf, yf, yawf = waypoints[-1]
    poses.append((xf, yf, math.radians(yawf)))
    return poses


# ==============================================================================
#  MAIN
# ==============================================================================

def main():
    random.seed(42)
    np.random.seed(42)

    ts = "20260611_062145"
    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "server_nodes", "logs", f"dual_session_{ts}")
    os.makedirs(out_dir, exist_ok=True)

    # Generate trajectories
    bot1_poses = interpolate_waypoints(BOT1_WAYPOINTS)
    bot2_poses_clean = interpolate_waypoints(BOT2_WAYPOINTS)

    # Insert stuck states for Bot 2 (e.g. wall too far / seeking wall loop in top-right corner)
    bot2_poses = []
    stuck_inserted = False
    for p in bot2_poses_clean:
        bot2_poses.append(p)
        # Check if we are near (5.2, 1.7) heading west, and haven't stuck yet
        if not stuck_inserted and len(bot2_poses) > 40:
            x, y, yaw = p
            # Near top-right corner, heading west (around math.pi)
            if abs(x - 5.2) < 0.25 and abs(y - 1.7) < 0.25 and abs(yaw - math.pi) < 0.4:
                stuck_inserted = True
                curr_yaw = yaw
                # Insert 40 stuck wiggling steps (wall lost/seeking wall loop)
                for step_idx in range(40):
                    # Wiggle in the top-right corner (tight scan, minimal translation drift)
                    wx = 5.2 + 0.01 * math.sin(step_idx * 0.35) + random.gauss(0, 0.002)
                    wy = 1.7 + 0.01 * math.cos(step_idx * 0.25) + random.gauss(0, 0.002)
                    # Spin yaw around seeking wall
                    curr_yaw = math.pi + 0.6 * math.sin(step_idx * 0.3) + random.gauss(0, 0.05)
                    bot2_poses.append((wx, wy, curr_yaw))

    len_bot1 = len(bot1_poses)
    len_bot2 = len(bot2_poses)

    # Pad to equal length for index safety in the loop
    max_len = max(len_bot1, len_bot2)
    while len(bot1_poses) < max_len:
        bot1_poses.append(bot1_poses[-1])
    while len(bot2_poses) < max_len:
        bot2_poses.append(bot2_poses[-1])

    print(f"Generated {len_bot1} poses for Bot 1, {len_bot2} poses for Bot 2")

    # Open output files
    f_telem = open(os.path.join(out_dir, "telemetry.csv"), 'w', newline='')
    f_points = open(os.path.join(out_dir, "pointcloud.csv"), 'w', newline='')
    w_telem = csv.writer(f_telem)
    w_points = csv.writer(f_points)
    w_telem.writerow(['time', 'agent', 'x', 'y', 'yaw_deg', 'encoder',
                      'v2v', 'front_cm', 'left_cm', 'back_cm', 'right_cm', 'landmark'])
    w_points.writerow(['time', 'agent', 'sensor', 'x', 'y'])

    base_time = 1781121500.000
    t = base_time

    enc1 = enc2 = 0
    
    # Initialize estimated poses at the starting poses of the ground-truth trajectories
    x_est1, y_est1, yaw_est1 = bot1_poses[0]
    x_est2, y_est2, yaw_est2 = bot2_poses[0]
    
    prev_est1 = (x_est1, y_est1)
    prev_est2 = (x_est2, y_est2)
    
    count1 = count2 = 0

    for i in range(max_len):
        x1, y1, yaw1 = bot1_poses[i]
        x2, y2, yaw2 = bot2_poses[i]

        # Uniform simulation clock update once per step
        dt = random.uniform(0.45, 0.65)
        t += dt

        # Integrate odometry drift for Bot 1 (starts at 2nd step, stops when bot stops)
        if i > 0 and i < len_bot1:
            x1_prev, y1_prev, yaw1_prev = bot1_poses[i-1]
            dx1 = x1 - x1_prev
            dy1 = y1 - y1_prev
            d_trans1 = math.sqrt(dx1**2 + dy1**2)
            d_rot1 = yaw1 - yaw1_prev
            while d_rot1 > math.pi:  d_rot1 -= 2 * math.pi
            while d_rot1 < -math.pi: d_rot1 += 2 * math.pi

            # Scale error (under-reports by 0.2%) and add Gaussian noise
            d_trans_noisy1 = d_trans1 * 0.998
            if d_trans1 > 0.001:
                d_trans_noisy1 += random.gauss(0, 0.003)
            d_trans_noisy1 = max(0.0, d_trans_noisy1)

            # Yaw drift bias (drifts rightward) and random noise
            d_rot_noisy1 = d_rot1
            if d_trans1 > 0.001:
                d_rot_noisy1 += d_trans1 * -0.008 + random.gauss(0, 0.002)
            elif abs(d_rot1) > 0.01:
                d_rot_noisy1 += random.gauss(0, 0.005)

            yaw_est1 += d_rot_noisy1
            while yaw_est1 > math.pi:  yaw_est1 -= 2 * math.pi
            while yaw_est1 < -math.pi: yaw_est1 += 2 * math.pi

            x_est1 += d_trans_noisy1 * math.cos(yaw_est1 - d_rot_noisy1 / 2.0)
            y_est1 += d_trans_noisy1 * math.sin(yaw_est1 - d_rot_noisy1 / 2.0)

        # Integrate odometry drift for Bot 2 (starts at 2nd step, stops when bot stops)
        if i > 0 and i < len_bot2:
            x2_prev, y2_prev, yaw2_prev = bot2_poses[i-1]
            dx2 = x2 - x2_prev
            dy2 = y2 - y2_prev
            d_trans2 = math.sqrt(dx2**2 + dy2**2)
            d_rot2 = yaw2 - yaw2_prev
            while d_rot2 > math.pi:  d_rot2 -= 2 * math.pi
            while d_rot2 < -math.pi: d_rot2 += 2 * math.pi

            # Scale error (over-reports by 0.2%) and add Gaussian noise
            d_trans_noisy2 = d_trans2 * 1.002
            if d_trans2 > 0.001:
                d_trans_noisy2 += random.gauss(0, 0.003)
            d_trans_noisy2 = max(0.0, d_trans_noisy2)

            # Yaw drift bias (drifts leftward) and random noise
            d_rot_noisy2 = d_rot2
            if d_trans2 > 0.001:
                d_rot_noisy2 += d_trans2 * 0.008 + random.gauss(0, 0.002)
            elif abs(d_rot2) > 0.01:
                d_rot_noisy2 += random.gauss(0, 0.005)

            yaw_est2 += d_rot_noisy2
            while yaw_est2 > math.pi:  yaw_est2 -= 2 * math.pi
            while yaw_est2 < -math.pi: yaw_est2 += 2 * math.pi

            x_est2 += d_trans_noisy2 * math.cos(yaw_est2 - d_rot_noisy2 / 2.0)
            y_est2 += d_trans_noisy2 * math.sin(yaw_est2 - d_rot_noisy2 / 2.0)

        # ── BOT 1 Readings ──
        if i < len_bot1:
            s1 = get_sensor_readings(x1, y1, yaw1)  # physical readings cast from TRUE pose
            lm1 = get_landmark_type(s1)

            # Encoder based on estimated pose movement
            d1_est = math.sqrt((x_est1 - prev_est1[0])**2 + (y_est1 - prev_est1[1])**2)
            enc1 += max(0, int(d1_est / 0.0107))
            prev_est1 = (x_est1, y_est1)

            # V2V (approximate physical distance between bots based on TRUE poses)
            v2v = int(math.sqrt((x1 - x2)**2 + (y1 - y2)**2) * 100)

            yaw_est1_deg = round(math.degrees(yaw_est1) / 15) * 15

            # Occasional duplicate packet
            n_dup = 2 if random.random() < 0.05 else 1
            for d in range(n_dup):
                tp = t + (random.uniform(-0.01, 0.01) if d > 0 else 0)
                w_telem.writerow([
                    f"{tp:.3f}", 1,
                    f"{x_est1:.4f}", f"{y_est1:.4f}",
                    f"{yaw_est1_deg:.2f}", enc1, v2v,
                    f"{s1['front']*100:.1f}", f"{s1['left']*100:.1f}",
                    f"{s1['back']*100:.1f}", f"{s1['right']*100:.1f}",
                    lm1,
                ])
                count1 += 1
                for name, dist in s1.items():
                    if MIN_SENSOR_RANGE < dist <= MAX_SENSOR_RANGE:
                        ra = yaw_est1 + SENSOR_ANGLES[name]
                        wx = x_est1 + dist * math.cos(ra)
                        wy = y_est1 + dist * math.sin(ra)
                        w_points.writerow([f"{tp:.3f}", 1, name, f"{wx:.4f}", f"{wy:.4f}"])

        # ── BOT 2 Readings ──
        if i < len_bot2:
            s2 = get_sensor_readings(x2, y2, yaw2)  # physical readings cast from TRUE pose
            lm2 = get_landmark_type(s2)

            # Encoder based on estimated pose movement
            d2_est = math.sqrt((x_est2 - prev_est2[0])**2 + (y_est2 - prev_est2[1])**2)
            enc2 += max(0, int(d2_est / 0.0107))
            prev_est2 = (x_est2, y_est2)

            # V2V (approximate physical distance between bots based on TRUE poses)
            v2v = int(math.sqrt((x1 - x2)**2 + (y1 - y2)**2) * 100)

            yaw_est2_deg = round(math.degrees(yaw_est2) / 15) * 15

            t2 = t + random.uniform(-0.08, 0.08)
            n_dup2 = 2 if random.random() < 0.05 else 1
            for d in range(n_dup2):
                tp2 = t2 + (random.uniform(-0.01, 0.01) if d > 0 else 0)
                w_telem.writerow([
                    f"{tp2:.3f}", 2,
                    f"{x_est2:.4f}", f"{y_est2:.4f}",
                    f"{yaw_est2_deg:.2f}", enc2, v2v,
                    f"{s2['front']*100:.1f}", f"{s2['left']*100:.1f}",
                    f"{s2['back']*100:.1f}", f"{s2['right']*100:.1f}",
                    lm2,
                ])
                count2 += 1
                for name, dist in s2.items():
                    if MIN_SENSOR_RANGE < dist <= MAX_SENSOR_RANGE:
                        ra = yaw_est2 + SENSOR_ANGLES[name]
                        wx = x_est2 + dist * math.cos(ra)
                        wy = y_est2 + dist * math.sin(ra)
                        w_points.writerow([f"{tp2:.3f}", 2, name, f"{wx:.4f}", f"{wy:.4f}"])

    f_telem.flush(); f_points.flush()
    f_telem.close(); f_points.close()

    # ── Generate derived files ──
    import pandas as pd
    pc = pd.read_csv(os.path.join(out_dir, "pointcloud.csv"))
    np.savetxt(os.path.join(out_dir, "pointcloud_merged.csv"),
               np.column_stack([pc['x'].values, pc['y'].values]),
               delimiter=",", header="x,y", comments="")
    for bid in [1, 2]:
        bdf = pc[pc['agent'] == bid]
        if not bdf.empty:
            np.savetxt(os.path.join(out_dir, f"pointcloud_bot{bid}.csv"),
                       np.column_stack([bdf['x'].values, bdf['y'].values]),
                       delimiter=",", header="x,y", comments="")

    with open(os.path.join(out_dir, "slam_closures.csv"), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['node_i', 'node_j', 'corr_dx', 'corr_dy'])

    print(f"\n{'='*60}")
    print(f"  BEDROOM SLAM SESSION GENERATED")
    print(f"{'='*60}")
    print(f"  Output: {out_dir}")
    print(f"  Bot1 rows: {count1}   Bot2 rows: {count2}")
    print(f"  Pointcloud: {len(pc)} points")
    print(f"  Bot1 start: (0, 0)  ->  maps LEFT half")
    print(f"  Bot2 start: (5, 0)  ->  maps RIGHT half")
    print(f"  Separation: 5.0m")
    print(f"  Paths converge at: x ~ 2.5 (room center)")
    print(f"{'='*60}")
    print(f"\n  To view:")
    print(f"    python playback_dual_session.py \"{out_dir}\"")


if __name__ == "__main__":
    main()
