#!/usr/bin/env python3
"""
Performance monitoring and benchmarking tool for Franka AR application.
Helps identify bottlenecks and measure optimization improvements.
"""

import time
import psutil
import numpy as np
import threading
import argparse
import signal
import sys
from collections import deque
import matplotlib.pyplot as plt
from FrankaClient import FrankaClient
import config

class PerformanceMonitor:
    def __init__(self, sample_interval=0.1):
        self.sample_interval = sample_interval
        self.running = False
        self.monitor_thread = None
        
        # Metrics storage
        self.cpu_usage = deque(maxlen=1000)
        self.memory_usage = deque(maxlen=1000)
        self.network_latency = deque(maxlen=1000)
        self.control_loop_timing = deque(maxlen=1000)
        self.timestamps = deque(maxlen=1000)
        
        # Robot client for latency testing
        self.robot_client = None
        
    def start_monitoring(self):
        """Start the performance monitoring thread."""
        if not self.running:
            self.running = True
            self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
            self.monitor_thread.start()
            print("Performance monitoring started...")
    
    def stop_monitoring(self):
        """Stop the performance monitoring thread."""
        self.running = False
        if self.monitor_thread:
            self.monitor_thread.join()
        print("Performance monitoring stopped.")
    
    def _monitor_loop(self):
        """Main monitoring loop."""
        while self.running:
            timestamp = time.time()
            
            # CPU and Memory usage
            cpu_percent = psutil.cpu_percent(interval=None)
            memory_info = psutil.virtual_memory()
            
            # Network latency (robot communication)
            latency = self._measure_network_latency()
            
            # Store metrics
            self.cpu_usage.append(cpu_percent)
            self.memory_usage.append(memory_info.percent)
            self.network_latency.append(latency)
            self.timestamps.append(timestamp)
            
            time.sleep(self.sample_interval)
    
    def _measure_network_latency(self):
        """Measure network latency to robot server."""
        if not self.robot_client:
            try:
                self.robot_client = FrankaClient(
                    server_ip=config.SERVER_IP,
                    port=config.SERVER_PORT,
                    connection_timeout=1.0
                )
            except:
                return -1.0  # Connection failed
        
        try:
            start_time = time.perf_counter()
            # Simple ping - get joint positions (lightweight call)
            self.robot_client.get_joint_positions()
            end_time = time.perf_counter()
            return (end_time - start_time) * 1000  # Convert to milliseconds
        except:
            return -1.0  # Call failed
    
    def measure_control_loop_performance(self, num_iterations=1000):
        """Measure control loop timing performance."""
        print(f"Measuring control loop performance ({num_iterations} iterations)...")
        
        # Simulate control loop operations
        start_rot_matrix = np.random.rand(3, 3)
        phone_poses = [np.random.rand(4, 4) for _ in range(num_iterations)]
        start_pos = np.random.rand(3)
        
        timings = []
        
        for i, phone_pose in enumerate(phone_poses):
            start_time = time.perf_counter()
            
            # Simulate the main operations from mujocoar_teleop.py
            new_pos = start_pos + phone_pose[:3, 3] * config.POSITION_SCALE_FACTOR
            new_pos[2] = np.clip(new_pos[2], config.MIN_Z_HEIGHT, config.MAX_Z_HEIGHT)
            new_pos[1] = np.clip(new_pos[1], config.MIN_Y_HEIGHT, config.MAX_Y_HEIGHT)
            new_pos[0] = config.FIXED_X_POSITION
            
            transformed_matrix = phone_pose[:3, :3] @ start_rot_matrix
            # transformed_rot_vec = R.from_matrix(transformed_matrix).as_rotvec()
            updated_pose = np.concatenate([new_pos, [0, 0, 0]])  # Simplified
            
            end_time = time.perf_counter()
            timings.append((end_time - start_time) * 1000000)  # Convert to microseconds
            
            if i % 100 == 0:
                print(f"Progress: {i/num_iterations*100:.1f}%")
        
        self.control_loop_timing = deque(timings, maxlen=1000)
        return timings
    
    def generate_report(self):
        """Generate a performance report."""
        if not self.timestamps:
            print("No performance data available. Start monitoring first.")
            return
        
        print("\n" + "="*50)
        print("PERFORMANCE REPORT")
        print("="*50)
        
        # CPU Usage Statistics
        if self.cpu_usage:
            cpu_avg = np.mean(self.cpu_usage)
            cpu_max = np.max(self.cpu_usage)
            print(f"CPU Usage - Avg: {cpu_avg:.1f}%, Max: {cpu_max:.1f}%")
        
        # Memory Usage Statistics
        if self.memory_usage:
            mem_avg = np.mean(self.memory_usage)
            mem_max = np.max(self.memory_usage)
            print(f"Memory Usage - Avg: {mem_avg:.1f}%, Max: {mem_max:.1f}%")
        
        # Network Latency Statistics
        valid_latencies = [l for l in self.network_latency if l > 0]
        if valid_latencies:
            lat_avg = np.mean(valid_latencies)
            lat_max = np.max(valid_latencies)
            lat_min = np.min(valid_latencies)
            print(f"Network Latency - Avg: {lat_avg:.2f}ms, Min: {lat_min:.2f}ms, Max: {lat_max:.2f}ms")
        
        # Control Loop Timing Statistics
        if self.control_loop_timing:
            loop_avg = np.mean(self.control_loop_timing)
            loop_max = np.max(self.control_loop_timing)
            loop_min = np.min(self.control_loop_timing)
            effective_freq = 1000000 / loop_avg if loop_avg > 0 else 0  # Hz
            print(f"Control Loop Timing - Avg: {loop_avg:.1f}μs, Min: {loop_min:.1f}μs, Max: {loop_max:.1f}μs")
            print(f"Effective Max Frequency: {effective_freq:.1f} Hz")
        
        print("="*50)
    
    def plot_metrics(self, save_path=None):
        """Plot performance metrics."""
        if not self.timestamps:
            print("No performance data available.")
            return
        
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        fig.suptitle('Franka AR Performance Metrics')
        
        # Convert timestamps to relative time
        if self.timestamps:
            start_time = self.timestamps[0]
            rel_times = [(t - start_time) for t in self.timestamps]
        
        # CPU Usage
        if self.cpu_usage and rel_times:
            axes[0, 0].plot(rel_times[:len(self.cpu_usage)], list(self.cpu_usage))
            axes[0, 0].set_title('CPU Usage (%)')
            axes[0, 0].set_xlabel('Time (s)')
            axes[0, 0].grid(True)
        
        # Memory Usage
        if self.memory_usage and rel_times:
            axes[0, 1].plot(rel_times[:len(self.memory_usage)], list(self.memory_usage))
            axes[0, 1].set_title('Memory Usage (%)')
            axes[0, 1].set_xlabel('Time (s)')
            axes[0, 1].grid(True)
        
        # Network Latency
        valid_latencies = [(i, l) for i, l in enumerate(self.network_latency) if l > 0]
        if valid_latencies:
            indices, latencies = zip(*valid_latencies)
            times = [rel_times[i] for i in indices if i < len(rel_times)]
            axes[1, 0].plot(times, latencies)
            axes[1, 0].set_title('Network Latency (ms)')
            axes[1, 0].set_xlabel('Time (s)')
            axes[1, 0].grid(True)
        
        # Control Loop Timing Histogram
        if self.control_loop_timing:
            axes[1, 1].hist(list(self.control_loop_timing), bins=50, alpha=0.7)
            axes[1, 1].set_title('Control Loop Timing Distribution (μs)')
            axes[1, 1].set_xlabel('Time (μs)')
            axes[1, 1].grid(True)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Performance plots saved to {save_path}")
        else:
            plt.show()

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully."""
    print('\nShutting down performance monitor...')
    sys.exit(0)

def main():
    parser = argparse.ArgumentParser(description='Franka AR Performance Monitor')
    parser.add_argument('--duration', type=int, default=30, 
                       help='Monitoring duration in seconds (default: 30)')
    parser.add_argument('--interval', type=float, default=0.1,
                       help='Sampling interval in seconds (default: 0.1)')
    parser.add_argument('--control-loop-test', action='store_true',
                       help='Run control loop performance test')
    parser.add_argument('--plot', action='store_true',
                       help='Show performance plots')
    parser.add_argument('--save-plot', type=str,
                       help='Save performance plot to file')
    
    args = parser.parse_args()
    
    # Set up signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Create performance monitor
    monitor = PerformanceMonitor(sample_interval=args.interval)
    
    try:
        # Start monitoring
        monitor.start_monitoring()
        
        # Run control loop test if requested
        if args.control_loop_test:
            monitor.measure_control_loop_performance()
        
        # Monitor for specified duration
        if args.duration > 0:
            print(f"Monitoring for {args.duration} seconds...")
            time.sleep(args.duration)
        else:
            print("Monitoring indefinitely. Press Ctrl+C to stop.")
            while True:
                time.sleep(1)
                
    except KeyboardInterrupt:
        pass
    finally:
        # Stop monitoring and generate report
        monitor.stop_monitoring()
        monitor.generate_report()
        
        # Show or save plots if requested
        if args.plot or args.save_plot:
            monitor.plot_metrics(save_path=args.save_plot)

if __name__ == "__main__":
    main()