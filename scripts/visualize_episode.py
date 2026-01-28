#!/usr/bin/env python3
"""
Episode Visualization Script
----------------------------
Visualizes collected episode data.

Usage:
    python3 visualize_episode.py <episode_file.jsonl>
    
Example:
    python3 visualize_episode.py ../data/omy_l100/session_20260123_143022/episode_001.jsonl
"""

import json
import sys
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path


def load_episode(jsonl_path):
    """Load episode data from JSONL file"""
    samples = []
    with open(jsonl_path, 'r') as f:
        for line in f:
            sample = json.loads(line)
            samples.append(sample)
    return samples


def visualize_episode(episode_path):
    """Visualize episode data"""
    # Load data
    samples = load_episode(episode_path)
    print(f"Loaded {len(samples)} samples from {episode_path.name}")
    
    # Extract data
    timestamps = np.array([s['timestamp'] for s in samples])
    timestamps = (timestamps - timestamps[0]) * 1e-9  # Convert to seconds from start
    
    ee_positions = np.array([s['state']['ee_pose'][:3] for s in samples])
    ee_orientations = np.array([s['state']['ee_pose'][3:] for s in samples])
    joints = np.array([s['state']['joints'] for s in samples])
    actions = np.array([s['action']['delta_twist'] for s in samples])
    
    # Load metadata
    meta_path = episode_path.parent / f"{episode_path.stem}_meta.json"
    target = None
    if meta_path.exists():
        with open(meta_path, 'r') as f:
            meta = json.load(f)
        target = np.array(meta.get('target_position', []))
    
    # Create figure
    fig = plt.figure(figsize=(16, 10))
    
    # 1. 3D trajectory
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    ax1.plot(ee_positions[:, 0], ee_positions[:, 1], ee_positions[:, 2], 'b-', linewidth=2, label='EE Trajectory')
    ax1.scatter(ee_positions[0, 0], ee_positions[0, 1], ee_positions[0, 2], c='g', s=100, marker='o', label='Start')
    ax1.scatter(ee_positions[-1, 0], ee_positions[-1, 1], ee_positions[-1, 2], c='r', s=100, marker='x', label='End')
    if target is not None:
        ax1.scatter(target[0], target[1], target[2], c='r', s=200, marker='*', label='Target')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('End-Effector 3D Trajectory')
    ax1.legend()
    ax1.grid(True)
    
    # 2. EE position over time
    ax2 = fig.add_subplot(2, 3, 2)
    ax2.plot(timestamps, ee_positions[:, 0], label='X')
    ax2.plot(timestamps, ee_positions[:, 1], label='Y')
    ax2.plot(timestamps, ee_positions[:, 2], label='Z')
    if target is not None:
        ax2.axhline(y=target[0], color='r', linestyle='--', alpha=0.3, label='Target X')
        ax2.axhline(y=target[1], color='g', linestyle='--', alpha=0.3, label='Target Y')
        ax2.axhline(y=target[2], color='b', linestyle='--', alpha=0.3, label='Target Z')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Position (m)')
    ax2.set_title('End-Effector Position vs Time')
    ax2.legend()
    ax2.grid(True)
    
    # 3. Distance to target over time
    if target is not None:
        ax3 = fig.add_subplot(2, 3, 3)
        distances = np.linalg.norm(ee_positions - target, axis=1)
        ax3.plot(timestamps, distances * 100, 'b-', linewidth=2)
        ax3.axhline(y=2.0, color='r', linestyle='--', label='Reach threshold (2cm)')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Distance (cm)')
        ax3.set_title('Distance to Target')
        ax3.legend()
        ax3.grid(True)
    
    # 4. Action (linear velocity)
    ax4 = fig.add_subplot(2, 3, 4)
    ax4.plot(timestamps, actions[:, 0], label='vx')
    ax4.plot(timestamps, actions[:, 1], label='vy')
    ax4.plot(timestamps, actions[:, 2], label='vz')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Linear Velocity (m/s)')
    ax4.set_title('Action: Linear Velocity')
    ax4.legend()
    ax4.grid(True)
    
    # 5. Action (angular velocity)
    ax5 = fig.add_subplot(2, 3, 5)
    ax5.plot(timestamps, actions[:, 3], label='wx')
    ax5.plot(timestamps, actions[:, 4], label='wy')
    ax5.plot(timestamps, actions[:, 5], label='wz')
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Angular Velocity (rad/s)')
    ax5.set_title('Action: Angular Velocity')
    ax5.legend()
    ax5.grid(True)
    
    # 6. Joint angles
    ax6 = fig.add_subplot(2, 3, 6)
    for i in range(7):
        ax6.plot(timestamps, joints[:, i], label=f'Joint {i+1}')
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('Joint Angle (rad)')
    ax6.set_title('Joint Angles')
    ax6.legend(loc='upper right', fontsize=8)
    ax6.grid(True)
    
    plt.tight_layout()
    
    # Add overall title
    fig.suptitle(f'Episode: {episode_path.name} ({len(samples)} samples, {timestamps[-1]:.2f}s)', 
                 fontsize=14, y=1.00)
    
    plt.show()


def main():
    if len(sys.argv) != 2:
        print("Usage: python3 visualize_episode.py <episode_file.jsonl>")
        print("Example: python3 visualize_episode.py ../data/omy_l100/session_20260123_143022/episode_001.jsonl")
        sys.exit(1)
    
    episode_file = Path(sys.argv[1])
    
    if not episode_file.exists():
        print(f"❌ Episode file not found: {episode_file}")
        sys.exit(1)
    
    visualize_episode(episode_file)


if __name__ == '__main__':
    main()
