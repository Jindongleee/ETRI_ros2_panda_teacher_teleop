#!/usr/bin/env python3
"""
Dataset Validation Script
-------------------------
Validates collected imitation learning datasets.

Usage:
    python3 validate_dataset.py <session_dir>
    
Example:
    python3 validate_dataset.py ../data/omy_l100/session_20260123_143022/
"""

import json
import sys
import os
from pathlib import Path
import numpy as np


def load_episode(jsonl_path):
    """Load episode data from JSONL file"""
    samples = []
    with open(jsonl_path, 'r') as f:
        for line in f:
            sample = json.loads(line)
            samples.append(sample)
    return samples


def validate_sample(sample, seq):
    """Validate a single sample"""
    errors = []
    
    # Check required fields
    required_fields = ['timestamp', 'seq', 'episode_id', 'state', 'action']
    for field in required_fields:
        if field not in sample:
            errors.append(f"Missing field: {field}")
    
    # Check sequence number
    if sample.get('seq') != seq:
        errors.append(f"Sequence mismatch: expected {seq}, got {sample.get('seq')}")
    
    # Check state structure
    state = sample.get('state', {})
    if 'ee_pose' not in state or len(state.get('ee_pose', [])) != 7:
        errors.append(f"Invalid state.ee_pose: expected 7 elements")
    if 'joints' not in state or len(state.get('joints', [])) != 7:
        errors.append(f"Invalid state.joints: expected 7 elements")
    
    # Check action structure
    action = sample.get('action', {})
    if 'delta_twist' not in action or len(action.get('delta_twist', [])) != 6:
        errors.append(f"Invalid action.delta_twist: expected 6 elements")
    
    # Check for NaN or Inf
    for field_name, values in [
        ('state.ee_pose', state.get('ee_pose', [])),
        ('state.joints', state.get('joints', [])),
        ('action.delta_twist', action.get('delta_twist', []))
    ]:
        if any(np.isnan(v) or np.isinf(v) for v in values):
            errors.append(f"NaN or Inf detected in {field_name}")
    
    return errors


def validate_episode(episode_path):
    """Validate an episode"""
    print(f"\n{'='*60}")
    print(f"Validating: {episode_path.name}")
    print('='*60)
    
    # Load data
    try:
        samples = load_episode(episode_path)
    except Exception as e:
        print(f"❌ Failed to load episode: {e}")
        return False
    
    print(f"✅ Loaded {len(samples)} samples")
    
    # Validate each sample
    total_errors = 0
    for i, sample in enumerate(samples):
        errors = validate_sample(sample, i)
        if errors:
            print(f"\n❌ Sample {i} errors:")
            for error in errors:
                print(f"   - {error}")
            total_errors += len(errors)
    
    if total_errors == 0:
        print(f"\n✅ All samples valid!")
    else:
        print(f"\n❌ Total errors: {total_errors}")
    
    # Statistics
    print(f"\n📊 Statistics:")
    print(f"   - Total samples: {len(samples)}")
    
    if len(samples) > 0:
        # Timestamps
        timestamps = [s['timestamp'] for s in samples]
        dt_values = np.diff(timestamps) * 1e-9  # Convert to seconds
        print(f"   - Duration: {(timestamps[-1] - timestamps[0]) * 1e-9:.2f}s")
        print(f"   - Average dt: {np.mean(dt_values):.4f}s ({1/np.mean(dt_values):.1f} Hz)")
        print(f"   - Dt std: {np.std(dt_values):.4f}s")
        
        # Action statistics
        actions = [s['action']['delta_twist'] for s in samples]
        actions = np.array(actions)
        print(f"   - Linear velocity range: [{np.min(actions[:,:3]):.4f}, {np.max(actions[:,:3]):.4f}] m/s")
        print(f"   - Angular velocity range: [{np.min(actions[:,3:]):.4f}, {np.max(actions[:,3:]):.4f}] rad/s")
    
    # Load metadata
    meta_path = episode_path.parent / f"{episode_path.stem}_meta.json"
    if meta_path.exists():
        with open(meta_path, 'r') as f:
            meta = json.load(f)
        print(f"\n📋 Metadata:")
        print(f"   - Target: {meta.get('target_position')}")
        print(f"   - Duration: {meta.get('duration'):.2f}s")
        print(f"   - Success: {meta.get('success')}")
    
    return total_errors == 0


def validate_session(session_dir):
    """Validate entire session"""
    session_path = Path(session_dir)
    
    if not session_path.exists():
        print(f"❌ Session directory not found: {session_dir}")
        return False
    
    print(f"\n{'='*60}")
    print(f"Validating Session: {session_path.name}")
    print('='*60)
    
    # Find all episode files
    episode_files = sorted(session_path.glob('episode_*.jsonl'))
    
    if not episode_files:
        print(f"❌ No episode files found in {session_dir}")
        return False
    
    print(f"Found {len(episode_files)} episodes")
    
    # Validate each episode
    results = []
    for episode_file in episode_files:
        result = validate_episode(episode_file)
        results.append(result)
    
    # Summary
    print(f"\n{'='*60}")
    print("SUMMARY")
    print('='*60)
    print(f"Total episodes: {len(results)}")
    print(f"Valid episodes: {sum(results)}")
    print(f"Invalid episodes: {len(results) - sum(results)}")
    
    if all(results):
        print(f"\n✅ All episodes valid!")
        return True
    else:
        print(f"\n❌ Some episodes have errors")
        return False


def main():
    if len(sys.argv) != 2:
        print("Usage: python3 validate_dataset.py <session_dir>")
        print("Example: python3 validate_dataset.py ../data/omy_l100/session_20260123_143022/")
        sys.exit(1)
    
    session_dir = sys.argv[1]
    success = validate_session(session_dir)
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
