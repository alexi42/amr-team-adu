# AMR FINAL PROJECT

Assignment description in [ASSIGNMENT.md](ASSIGNMENT.md).

## How to use this code

The project was developed and tested using ROS 2 Humble.

### 1. Build the workspace

```bash
cd ~/amr_ws
colcon build --packages-select amr_adu_robile
source install/setup.bash
```

### 2. Start the robot or simulation
```bash
ros2 launch robile_gazebo gazebo_4_wheel.launch.py
```

Make sure the robot is publishing the required ROS 2 topics:

- `/scan`
- `/odom`
- `/tf`
- `/tf_static`

### 3. Run path planning

```bash
ros2 run amr_adu_robile state_machine
```

### 4. Run localisation

```bash
ros2 run amr_adu_robile localisation
```

### 4. Run SLAM

```bash
ros2 run amr_adu_robile localisation
```
