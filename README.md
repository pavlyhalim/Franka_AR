Here's a clearer, more professional version of your README file:

---

# Franka\_AR

## Overview

**Franka\_AR** enables augmented reality (AR) teleoperation of a Franka Emika Panda robot using the [MuJoCo AR mobile app](https://apps.apple.com/ae/app/mujoco-ar/id6612039501). This project integrates [Polymetis](https://github.com/facebookresearch/fairo) and a custom AR extension to provide real-time control via an iPhone.

## Prerequisites

Before getting started, ensure the following are installed and configured:

* **libfranka v0.15**
* **ROS Noetic**
* **[Polymetis](https://github.com/facebookresearch/fairo)** (from FAIR)
* **`mujoco_ar`** (custom AR package)

> Make sure all dependencies are correctly linked and sourced, especially libfranka with ROS.

## Setup Instructions

1. **Install Dependencies**
   Clone and install Polymetis and the `mujoco_ar` package. Follow their respective installation guides.

2. **Launch the System in Order**
   Open new terminal windows/tabs for each of the following commands:

   ```bash
   bash launch_gripper.launch
   bash launch_robot.launch
   bash launch_server.launch
   ```

3. **Start the Control Script**
   Run the following script to enable teleoperation:

   ```bash
   python tele_random.py
   ```

4. **Control via iPhone**
   Download and install the [MuJoCo AR app](https://apps.apple.com/ae/app/mujoco-ar/id6612039501) from the App Store. Connect it to your local setup to begin controlling the robot in AR.

---

Special thanks to Omar Rayan for his work on MuJoCo-AR, which made this project possible.
His contributions to AR-based robotics control are greatly appreciated.
