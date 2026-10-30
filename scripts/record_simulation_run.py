#!/usr/bin/env python3
"""
Automated Simulation Recorder for Gazebo + RViz2.
Captures high-res split-screen screenshots and MP4 video demo.
"""

import os
import re
import sys
import time
import struct
import subprocess
import numpy as np
from PIL import Image
import cv2

MEDIA_DIR = "/home/sanket/orca_robotics_task/docs/media"
os.makedirs(MEDIA_DIR, exist_ok=True)

def find_windows():
    """Find X11 window IDs for RViz2 and Gazebo."""
    res = subprocess.run(['xwininfo', '-root', '-tree'], capture_output=True, text=True)
    rviz_id = None
    gazebo_id = None
    for line in res.stdout.splitlines():
        if 'rviz' in line.lower() and not rviz_id:
            m = re.search(r'(0x[0-9a-fA-F]+)\s+\"([^\"]*rviz[^\"]*)\"', line, re.I)
            if m:
                rviz_id = m.group(1)
        if 'gazebo' in line.lower() and not gazebo_id:
            m = re.search(r'(0x[0-9a-fA-F]+)\s+\"Gazebo\"', line)
            if m:
                gazebo_id = m.group(1)
    return rviz_id, gazebo_id

def capture_window(win_id):
    """Capture window pixels via xwd in memory."""
    if not win_id:
        return None
    res = subprocess.run(['xwd', '-id', win_id, '-silent'], capture_output=True)
    data = res.stdout
    if len(data) < 100:
        return None
    hdr_fmt = '>IIIIIIIIIIIIIIIIIIIIIIiiI'
    hdr_size = struct.calcsize(hdr_fmt)
    fields = struct.unpack(hdr_fmt, data[:hdr_size])
    header_size, width, height, bpp, bpl, ncolors = fields[0], fields[4], fields[5], fields[11], fields[12], fields[19]
    pixel_offset = header_size + ncolors * 12
    raw = data[pixel_offset:]
    expected_len = height * bpl
    arr = np.frombuffer(raw[:expected_len], dtype=np.uint8)
    arr = arr.reshape((height, bpl // 4, 4))[:, :width, :3] # BGR
    return arr

def capture_split_screen(rviz_id, gazebo_id, target_width=1280):
    """Create a side-by-side combined frame."""
    rviz = capture_window(rviz_id)
    gazebo = capture_window(gazebo_id)
    if rviz is None or gazebo is None:
        return None
    h = min(rviz.shape[0], gazebo.shape[0])
    combined = np.hstack([gazebo[:h, :, :], rviz[:h, :, :]])
    # Resize for video
    scale = target_width / combined.shape[1]
    target_height = int(combined.shape[0] * scale)
    # Ensure even dimensions for video codecs
    if target_height % 2 != 0:
        target_height -= 1
    if target_width % 2 != 0:
        target_width -= 1
    resized = cv2.resize(combined, (target_width, target_height), interpolation=cv2.INTER_AREA)
    return resized, combined

def main():
    print("🎬 Starting Automated Simulation Recorder...")
    rviz_id, gazebo_id = find_windows()
    print(f"Windows detected -> RViz: {rviz_id}, Gazebo: {gazebo_id}")
    if not rviz_id or not gazebo_id:
        print("❌ Error: Could not find both RViz2 and Gazebo windows.")
        sys.exit(1)

    # 1. Capture initial overview screenshot (full resolution)
    _, initial_full = capture_split_screen(rviz_id, gazebo_id, target_width=1600)
    cv2.imwrite(os.path.join(MEDIA_DIR, "01_sim_overview.png"), initial_full)
    print("📸 Saved docs/media/01_sim_overview.png")

    # 2. Setup video writer
    sample_frame, _ = capture_split_screen(rviz_id, gazebo_id, target_width=1280)
    vh, vw = sample_frame.shape[:2]
    fps = 8.0
    video_path = os.path.join(MEDIA_DIR, "simulation_demo.mp4")
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(video_path, fourcc, fps, (vw, vh))

    # Also collect frames for an animated GIF summary
    gif_frames = []

    # 3. Launch navigation node in subprocess
    print("🚀 Launching navigate_to_goal node to (-2.0, -0.5)...")
    nav_cmd = [
        "bash", "-c",
        "source /opt/ros/humble/setup.bash && "
        "source /home/sanket/orca_robotics_task/nav2_goal_task/install/setup.bash && "
        "ros2 run nav2_goal_navigator navigate_to_goal --ros-args -p goal_x:=-2.0 -p goal_y:=-0.5"
    ]
    proc = subprocess.Popen(nav_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    start_time = time.time()
    frame_count = 0
    snap_progress_done = False
    snap_obstacle_done = False

    while proc.poll() is None:
        loop_start = time.time()
        frame, full_frame = capture_split_screen(rviz_id, gazebo_id, target_width=1280)
        if frame is not None:
            out.write(frame)
            frame_count += 1
            if frame_count % 3 == 0:
                # Save downscaled RGB for GIF
                rgb_small = cv2.resize(frame, (640, int(vh * 640 / vw)))
                rgb_small = cv2.cvtColor(rgb_small, cv2.COLOR_BGR2RGB)
                gif_frames.append(Image.fromarray(rgb_small))

            elapsed = time.time() - start_time
            # Snapshots at key points
            if elapsed > 4 and not snap_progress_done:
                cv2.imwrite(os.path.join(MEDIA_DIR, "02_navigation_in_progress.png"), full_frame)
                print("📸 Saved docs/media/02_navigation_in_progress.png")
                snap_progress_done = True

            if elapsed > 12 and not snap_obstacle_done:
                cv2.imwrite(os.path.join(MEDIA_DIR, "03_dynamic_obstacle_avoidance.png"), full_frame)
                print("📸 Saved docs/media/03_dynamic_obstacle_avoidance.png")
                snap_obstacle_done = True

        # Pace loop to ~fps
        step_duration = time.time() - loop_start
        sleep_time = max(0, (1.0 / fps) - step_duration)
        time.sleep(sleep_time)

        # Timeout safety (60s)
        if time.time() - start_time > 60:
            print("⚠️ Reached max recording timeout (60s).")
            break

    # Final goal reached screenshot
    _, final_full = capture_split_screen(rviz_id, gazebo_id, target_width=1600)
    cv2.imwrite(os.path.join(MEDIA_DIR, "04_goal_reached_success.png"), final_full)
    print("📸 Saved docs/media/04_goal_reached_success.png")

    out.release()
    print(f"🎥 Video saved to {video_path} ({frame_count} frames, {time.time()-start_time:.1f}s)")

    # Save animated GIF (great for GitHub README preview!)
    if gif_frames:
        gif_path = os.path.join(MEDIA_DIR, "simulation_demo.gif")
        gif_frames[0].save(
            gif_path,
            save_all=True,
            append_images=gif_frames[1:],
            duration=120,
            loop=0
        )
        print(f"🎞️ Animated GIF saved to {gif_path}")

    print("\n✅ All screenshots and video captured successfully in docs/media/!")

if __name__ == "__main__":
    main()
