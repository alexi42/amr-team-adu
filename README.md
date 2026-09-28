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

### 3. Run localisation

```bash
ros2 run amr_adu_robile localisation
```

The localisation node loads a saved occupancy grid, initialises particles across the map and estimates the robot's position using odometry and LiDAR measurements.

For real robot experiments, `use_sim_time` must be set to `false`.

The following topics can be visualised in RViz:

- `/map` – Saved occupancy grid
- `/particle_cloud` – Particle distribution
- `/estimated_pose` – Estimated robot position
- `/scan` – LiDAR measurements

<img width="1915" height="1091" alt="localisation" src="https://github.com/user-attachments/assets/72af27c5-153d-4fda-82a6-bc5e0d70f2b5" />


## Approach to solving the tasks

### Path Planning

The A* algorithm was implemented for global path planning. It searches the occupancy grid to find a path from the robot's current position to the goal.

The generated path is used by the robot controller to navigate towards the target.

### Localisation

Monte Carlo Localisation (MCL) was implemented without using AMCL.

First, a saved occupancy grid is loaded, and 1500 particles are randomly distributed across the free areas of the map.

The motion update uses robot odometry. Gaussian noise is added to the odometry measurements to represent motion uncertainty.

For the sensor update, 24 LiDAR beams are selected and compared with simulated LiDAR measurements generated through ray casting from each particle.

Particles whose predicted measurements are closer to the real LiDAR measurements receive higher weights.

The particles are then resampled according to their weights. After each resampling step, 5% of the particles are replaced with randomly generated particles to help the filter recover from incorrect localisation hypotheses.

The estimated robot pose is calculated from a concentrated cluster of particles.

### Mapping and Exploration

SLAM Toolbox was used to create an occupancy grid of the real environment.

LiDAR and odometry measurements were used to build the map while the robot moved through the environment.

The resulting map can be saved and used by the localisation and navigation components.

## Challenges we faced

### Global Localisation

One of the main challenges was achieving reliable global localisation when the particles were initially distributed across the entire map.

Our first sensor model used a likelihood field, but it did not always converge to the correct robot position.

We therefore replaced it with a beam-based ray casting approach. This allowed the predicted LiDAR measurements of each particle to be compared directly with the actual measurements.

### Particle Recovery

Another challenge was recovering from incorrect localisation hypotheses. If particles around an incorrect position became dominant, the filter could struggle to recover.

To address this problem, 5% of the particles are replaced with randomly generated particles after every resampling step.

### Real Robot Integration

Testing on the real Robile robot required some adjustments to the ROS 2 configuration.

The real robot uses `base_link` as its base frame, whereas some simulation and SLAM configurations expected `base_footprint`.

We also had to disable simulation time, verify the LiDAR transformations and synchronise the robot's clock with the laptop to avoid TF timestamp problems.
