# ReemcWalker REEM-C Docker

This repository contains a Dockerized environment for simulating the REEM-C humanoid robot's walking controller based on the reemc_walker framework.

## Overview

The REEM-C robot is a full-size humanoid robot platform, and this simulation environment focuses on its walking controller implementation. The controller enables stable bipedal locomotion through advanced control algorithms and real-time balance management.

## Walking Controller Architecture

The walking controller consists of several key components working together:

1. **State Estimation**
   - Joint encoder readings for kinematic state
   - IMU data for body orientation
   - Force/torque sensors in feet for ground contact detection
   - Fusion of sensor data for robust state estimation

2. **Pattern Generation**
   - Generation of reference Center of Mass (CoM) trajectory
   - Foot placement planning based on desired walking parameters
   - Zero Moment Point (ZMP) trajectory generation
   - Preview control for maintaining dynamic balance

3. **Balance Control**
   - Real-time ZMP tracking
   - Whole-body motion control
   - Compliant control for foot-ground interaction
   - Stabilizer for disturbance rejection

## Controller Operation

The walking controller operates through the following steps:

1. **Initialization Phase**
   - Robot model loading and parameter initialization
   - Sensor calibration and state estimation setup
   - Controller gains configuration

2. **Planning Phase**
   - User input processing for walking parameters
   - Generation of reference trajectories
   - Footstep planning based on desired path

3. **Control Loop**
   - Real-time state estimation (500Hz)
   - ZMP and CoM tracking
   - Joint trajectory computation
   - Balance maintenance through feedback control

4. **Execution**
   - Joint command publishing
   - Real-time monitoring and adjustment
   - Safety checks and emergency stops if needed

## Key Features

- Real-time balance control
- Adaptive step planning
- Disturbance rejection
- Compliant ground interaction
- Multiple walking modes (forward, sideways, turning)

## Implementation Details

The controller implementation follows these principles:

1. **Hierarchical Control**
   ```
   High Level: Walking pattern generation
      ↓
   Mid Level: ZMP and CoM control
      ↓
   Low Level: Joint position/torque control
   ```

2. **State Machine**
   ```
   IDLE → INITIALIZE → STANDING → WALKING → STOPPING
   ```

3. **Key Parameters**
   - Step length and width
   - Walking speed
   - ZMP safety margins
   - Balance control gains

## Usage in Simulation

1. **Environment Setup**
   ```bash
   cd ~/reemc_docker
   docker build -t reemc .
   ```
   ```bash
   docker run -it --gpus all --rm --user ros --network=host --ipc=host -e DISPLAY=$DISPLAY -e NVIDIA_DRIVER_CAPABILITIES=all -e NVIDIA_VISIBLE_DEVICES=all -v /tmp/.X11-unix:/tmp/.X11-unix:rw reemc:latest
    ```

2. **Launch Simulation**
   ```bash
   # Inside container
   source /home/ros/reemc_ws/devel/setup.bash
   roslaunch ow_reemc gazebo.launch
   ```

3. **Start Controller**
   ```bash
   source /home/ros/reemc_ws/devel/setup.bash
   roslaunch ow_reemc sim.launch
   ```

4. **Command Walking**
   ```bash
   rosservice call /reemc_walker/plan_fixed_line
   ```

## Safety Features

- Automatic stopping on excessive ZMP deviation
- Joint limit monitoring
- Ground contact force thresholds
- Emergency stop triggers

## Performance Metrics

The controller achieves:
- Stable walking at 0.2 m/s
- Step length up to 0.15m
- Turning capability up to 30 degrees
- Balance recovery from pushes up to 10N

## Troubleshooting

Common issues and solutions:
1. Controller not starting
   - Check ROS master status
   - Verify sensor data publication
   - Review controller parameters

2. Unstable walking
   - Adjust control gains
   - Check ground friction parameters
   - Verify state estimation accuracy

## Credits
[TUM-ICS/openwalker](https://github.com/TUM-ICS/openwalker)
