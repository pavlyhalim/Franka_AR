# Franka\_AR

## Overview

**Franka\_AR** enables augmented reality (AR) teleoperation of a Franka Emika Panda robot using the [MuJoCo AR mobile app](https://apps.apple.com/ae/app/mujoco-ar/id6612039501). This project integrates [Polymetis](https://github.com/facebookresearch/fairo) and a custom AR extension to provide real-time control via an iPhone.

## Recent Optimizations

This application has been optimized for better real-time performance:

- **Reduced CPU usage** by optimizing the main control loop (200 Hz instead of 1000 Hz)
- **Improved network reliability** with automatic reconnection and error recovery
- **Cached calculations** to avoid repeated matrix operations
- **Centralized configuration** for easy parameter tuning
- **Performance monitoring** tools for benchmarking and profiling
- **Cleaner error handling** to prevent system crashes

## Prerequisites

Before getting started, ensure the following are installed and configured:

* **libfranka v0.15**
* **ROS Noetic**
* **[Polymetis](https://github.com/facebookresearch/fairo)** (from FAIR)
* **[mujoco_ar](https://github.com/omarrayyann/MujocoAR)** (custom AR package)

> Make sure all dependencies are correctly linked and sourced, especially libfranka with ROS.

## Setup Instructions

1. **Install Dependencies**
   Clone and install Polymetis and the `mujoco_ar` package. Follow their respective installation guides.

2. **Launch the System in Order**
   Open new terminal windows/tabs for each of the following commands:

   ```bash
   bash launch_gripper.sh
   bash launch_robot.sh
   bash launch_server.sh
   ```

3. **Start the Control Script**
   Run the following script to enable teleoperation:

   ```bash
   python mujocoar_teleop.py
   ```

4. **Control via iPhone**
   Download and install the [MuJoCo AR app](https://apps.apple.com/ae/app/mujoco-ar/id6612039501) from the App Store. Connect it to your local setup to begin controlling the robot in AR.

## Configuration and Tuning

All system parameters are now centralized in `config.py`:

- **Control frequency**: Adjustable from 1-1000 Hz
- **Safety bounds**: Configurable workspace limits
- **Network timeouts**: Tunable connection parameters
- **Cartesian gains**: Easy impedance controller tuning

Edit `config.py` to optimize performance for your specific setup.

## Performance Monitoring

Use the built-in performance monitoring tools:

```bash
# Monitor system performance for 60 seconds
python performance_monitor.py --duration 60 --plot

# Run control loop benchmarks
python performance_monitor.py --control-loop-test

# Analyze code for optimization opportunities
python optimize_analyzer.py
```

## Testing

Test robot movements with predefined poses:

```bash
python test.py
```

## Files Overview

- `server.py` - ZeroRPC server interfacing with Polymetis
- `mujocoar_teleop.py` - Main AR teleoperation control loop (optimized)
- `FrankaClient.py` - Robot communication client with auto-reconnection
- `random_points.cpp` - Direct libfranka control for scripted movements
- `config.py` - Centralized configuration parameters
- `performance_monitor.py` - Performance analysis and benchmarking
- `optimize_analyzer.py` - Code optimization analysis tool
- `test.py` - Robot testing with predefined poses

---

Special thanks to Omar Rayan for his work on MuJoCo-AR, which made this project possible.
His contributions to AR-based robotics control are greatly appreciated.
