#!/usr/bin/env python3
"""
Quick performance test to demonstrate optimization improvements.
Simulates the control loop operations without connecting to robot.
"""

import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import config

def test_old_implementation(iterations=10000):
    """Simulate the old, unoptimized control loop."""
    print("Testing old implementation...")
    
    start_pos = np.random.rand(3)
    start_rot_matrix = R.random().as_matrix()
    
    start_time = time.perf_counter()
    
    for i in range(iterations):
        # Old approach: create new arrays every iteration
        phone_pose = np.identity(4)  # New array allocation
        phone_pose[:3, :3] = R.random().as_matrix()
        phone_pose[:3, 3] = np.random.rand(3)
        
        new_pos = start_pos + phone_pose[:3, 3] * 0.8  # New array allocation
        new_pos[2] = max(0.3, new_pos[2])
        new_pos[2] = min(0.9, new_pos[2])
        new_pos[1] = max(-0.5, new_pos[1])
        new_pos[1] = min(0.38, new_pos[1])
        new_pos[0] = 0.29
        
        transformed_matrix = phone_pose[:3, :3] @ start_rot_matrix
        transformed_rot_vec = R.from_matrix(transformed_matrix).as_rotvec()
        updated_pose = np.concatenate([new_pos, transformed_rot_vec])  # New allocation
        
        # Simulate processing
        _ = updated_pose.sum()
    
    end_time = time.perf_counter()
    return end_time - start_time

def test_new_implementation(iterations=10000):
    """Simulate the new, optimized control loop."""
    print("Testing optimized implementation...")
    
    start_pos = np.random.rand(3)
    start_rot_matrix = R.random().as_matrix()
    
    # Pre-allocate arrays (optimization)
    phone_pose = config.IDENTITY_4x4.copy()
    new_pos = config.ZERO_VECTOR_3.copy()
    updated_pose = config.ZERO_VECTOR_6.copy()
    
    start_time = time.perf_counter()
    
    for i in range(iterations):
        # New approach: reuse pre-allocated arrays
        phone_pose[:3, :3] = R.random().as_matrix()
        phone_pose[:3, 3] = np.random.rand(3)
        
        np.copyto(new_pos, start_pos)  # Reuse array
        new_pos += phone_pose[:3, 3] * config.POSITION_SCALE_FACTOR
        
        # Use numpy clipping (optimized)
        new_pos[2] = np.clip(new_pos[2], config.MIN_Z_HEIGHT, config.MAX_Z_HEIGHT)
        new_pos[1] = np.clip(new_pos[1], config.MIN_Y_HEIGHT, config.MAX_Y_HEIGHT)
        new_pos[0] = config.FIXED_X_POSITION
        
        transformed_matrix = phone_pose[:3, :3] @ start_rot_matrix
        transformed_rot_vec = R.from_matrix(transformed_matrix).as_rotvec()
        
        # Reuse pre-allocated array
        updated_pose[:3] = new_pos
        updated_pose[3:] = transformed_rot_vec
        
        # Simulate processing
        _ = updated_pose.sum()
    
    end_time = time.perf_counter()
    return end_time - start_time

def main():
    iterations = 10000
    print(f"Performance Test - {iterations} iterations")
    print("=" * 50)
    
    # Test old implementation
    old_time = test_old_implementation(iterations)
    
    # Test new implementation  
    new_time = test_new_implementation(iterations)
    
    # Calculate improvements
    improvement = (old_time - new_time) / old_time * 100
    speedup = old_time / new_time
    
    print("\nResults:")
    print("=" * 50)
    print(f"Old implementation: {old_time:.4f} seconds")
    print(f"New implementation: {new_time:.4f} seconds")
    print(f"Performance improvement: {improvement:.1f}%")
    print(f"Speedup factor: {speedup:.2f}x")
    
    print(f"\nAt {config.CONTROL_FREQUENCY} Hz:")
    old_max_freq = iterations / old_time
    new_max_freq = iterations / new_time
    print(f"Old max frequency: {old_max_freq:.0f} Hz")
    print(f"New max frequency: {new_max_freq:.0f} Hz")
    
    if new_max_freq >= config.CONTROL_FREQUENCY:
        print(f"✅ Target frequency ({config.CONTROL_FREQUENCY} Hz) achievable")
    else:
        print(f"⚠️  Target frequency ({config.CONTROL_FREQUENCY} Hz) may be challenging")

if __name__ == "__main__":
    main()