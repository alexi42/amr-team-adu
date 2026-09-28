# AMR FINAL PROJECT

This project was developed for the Autonomous Mobile Robots course at Hochschule Bonn-Rhein-Sieg.

The project consists of three main tasks:

1. Path and Motion Planning
2. Monte Carlo Localisation
3. SLAM and Autonomous Exploration

The implementations were developed using ROS 2 Humble and tested in simulation and on the real Robile robot.


## How to use this code

### Requirements

- Ubuntu 22.04
- ROS 2 Humble

### 1. Build the workspace

```bash
cd ~/(your_workspace)
colcon build --packages-select amr_adu_robile
source install/setup.bash
```

### 2. Start the simulation
```bash
ros2 launch robile_gazebo gazebo_4_wheel.launch.py
```

### 3. Start the robot
```bash
ros2 launch robile_bringup robot.launch.py
```

Make sure the robot is publishing the required ROS 2 topics:

- `/scan`
- `/odom`
- `/tf`
- `/tf_static`


### 4. Run path planning node

Start the path-planning node after starting the robot.

The node uses A* to find a path to the goal. It uses a potential field controller to follow the path and avoid obstacles.

```bash
ros2 run amr_adu_robile pot_field_path_planning
```


### 5. Execute localisation node

The localisation node needs an existing map.

Run the node with the path to your map:

```bash
ros2 launch amr_adu_robile localisation.py
```

For the real robot, set `use_sim_time` to `false`.

The localisation node loads a saved occupancy grid, initialises particles across the map and estimates the robot's position using odometry and LiDAR measurements.

The following topics can be visualised in RViz:

- `/map` – Saved occupancy grid
- `/particle_cloud` – Particle distribution
- `/estimated_pose` – Estimated robot position
- `/scan` – LiDAR measurements


### 6. Run SLAM

The exploration code uses wall following to explore the environment. It also creates an occupancy grid from LiDAR and odometry data.

We also used SLAM Toolbox to create a map with the real robot.

For Robile4, SLAM Toolbox needs these frame settings:

```yaml
odom_frame: odom
map_frame: map
base_frame: base_link
scan_topic: /scan
```

```bash
ros2 run amr_adu_robile wall_follower
```

## Approach to Solving the Tasks

## 1. Path and Motion Planning

We combined a global path planner with a local potential field controller.

### Potential Field Controller

The potential field controller uses attractive and repulsive forces.

The attractive force moves the robot towards its goal. The repulsive force pushes it away from nearby obstacles.

The controller uses LiDAR measurements to calculate the repulsive force.

We also added movement in the y-direction so the robot could move sideways. However, the real robot's LiDAR mainly detects obstacles in front of it. Therefore, we used a lower speed for sideways movement.

The parameters `ka`, `kr` and `rho0` were adjusted during testing. We also changed the robot's speed because it moved much faster on the real robot than in simulation.

### Global Path Planner

We used A* to find a path from the robot's current position to the goal.

The planner uses an occupancy grid. At the beginning, all grid cells are considered free. Obstacles are added using LiDAR measurements.

The robot's starting position is placed in the middle of the grid. This allows the planner to use both positive and negative world coordinates.

A* calculates a path through the free cells. The path is then divided into waypoints.

Our A* implementation was adapted from the [GeeksforGeeks A* example](https://www.geeksforgeeks.org/python/a-search-algorithm-in-python/).

### State Machine

We used a state machine to control the planning process. It contains four states.

**CREATE WAYPOINTS**

The robot uses A* to find a path to the goal. This state is used at the beginning and when a new obstacle blocks the current path.

**FOLLOW WAYPOINTS**

The potential field controller moves the robot towards the next waypoint.

A waypoint is removed from the list when the robot is within 0.05 m of it.

If an obstacle blocks the next waypoint, the robot stops and returns to CREATE WAYPOINTS.

**GOAL REACHED**

The robot enters this state when it reaches the destination grid cell.

It stops moving forward and turns to the required final orientation.

**GOAL UNREACHABLE**

The robot enters this state when A* cannot find a path to the destination.

The robot stops completely.

Our occupancy grid does not support dynamic obstacles. Once a cell is marked as occupied, it stays occupied.

### Replanning

If a new obstacle blocks the current path, the robot stops before calculating new waypoints.

We added this because the robot previously continued moving while the new path was being calculated.

We also added a check for the final goal. An empty waypoint list does not always mean that the robot has reached the destination.

If the waypoint list is empty but the robot has not reached the goal, the planner calculates a new path.

The occupancy grid is created once and updated when new obstacles are detected. This keeps information about obstacles found earlier.

<!-- Add your path-planning screenshot here. -->
<!-- ![Path planning](images/path_planning.png) -->

## 2. Monte Carlo Localisation

We implemented our own Monte Carlo Localisation (MCL) algorithm without using AMCL.

The algorithm uses an existing occupancy grid, odometry and LiDAR measurements to estimate the robot's position.

### Particle Initialisation

We first tested the algorithm with 300 particles. Later, we increased this number to 1500.

The particles are placed at random positions in the free areas of the map. Each particle also has a random orientation.

This allows the robot to estimate its position without knowing its initial location.

### Motion Update

We use odometry data from `/odom` to update the particles.

The robot's translation and rotation are applied to every particle.

Gaussian noise is added to the movement. The noise increases with the travelled distance and rotation.

### Sensor Update

We first implemented a likelihood-field sensor model. However, it did not always find the correct position when the particles were concentrated far from the real robot.

We replaced it with a beam-based sensor model.

For each update, we select up to 24 valid LiDAR beams. We use ray casting to calculate the expected LiDAR distances for each particle.

We compare these expected distances with the real LiDAR measurements.

Particles with smaller measurement errors receive higher weights.

### Resampling

After the sensor update, particles are resampled according to their weights.

Particles with higher weights have a greater chance of being selected.

We also replace 5% of the resampled particles with new random particles. This helps the filter recover from incorrect position estimates.

### Pose Estimation

We select a group of particles that have similar positions and orientations.

The group must contain enough particles and have a high enough total weight.

We calculate the weighted average position and orientation of this group to estimate the robot's pose.

The result is published on `/estimated_pose`.

We also implemented the transformation between the `map` and `odom` frames. It is calculated using the estimated pose and the robot's odometry.

This allows the robot and the map to be shown in the same coordinate frame in RViz.

<img width="1915" height="1091" alt="localisation" src="https://github.com/user-attachments/assets/72af27c5-153d-4fda-82a6-bc5e0d70f2b5" />


## 3. Environment Exploration and Mapping

For this task, we implemented wall following together with a potential field controller.

The robot explores the environment while creating an occupancy grid.

### Occupancy Grid

The occupancy grid stores three types of cells:

- `-1`: Unknown
- `0`: Free
- `100`: Occupied

We update the grid using LiDAR measurements and the robot's position from odometry.

Bresenham's line algorithm is used to find the cells along each LiDAR beam.

Cells along the beam are marked as free. If the beam detects an obstacle, its endpoint is marked as occupied.

The map is created around the robot's starting position.

### Wall Following

The robot starts by searching for a wall on its right side.

It moves towards the wall until it reaches a distance of about one metre. It then follows the wall while keeping a suitable distance.

If the robot approaches a corner or an obstacle in front of it, it turns left.

If the wall on its right side disappears, it turns right until it can follow the wall again.

The next goal is usually placed in front of the robot. This goal is passed to the potential field controller.

The attractive force moves the robot towards the goal. The repulsive force helps it avoid nearby obstacles.

### Searching for a Wall

If the robot starts in the middle of a large room, it may not detect any walls.

For this situation, we implemented `move_around()`.

The robot moves slowly until it detects a wall.

Our wall detection method looks for at least five LiDAR points that are no more than five centimetres apart.

This is a simple method for our test environment. It may not work reliably in larger environments.

### Map Coverage

We calculate map coverage using the area explored by the robot.

The first goal is to cover at least 80% of the environment.

After reaching this value, the robot should use A* to find a path towards an unknown cell.

However, this part did not work as expected. The algorithm could not always find a path to an unknown cell, even when the path appeared to be free.

The robot can finish exploration when more than 95% of the map is covered. Reaching 100% is not always possible because some areas can be detected by LiDAR but cannot be reached by the robot.

When exploration finishes or the robot cannot continue, the map can be saved using `nav2_map_server`.

### Mapping Results

The custom mapping method produced an occupancy grid during simulation.

However, the published map appeared mirrored because of a problem with coordinate conversion.

We tested exchanging the x and y coordinates, which corrected the displayed map. We did not keep this change in the shared conversion methods because the other tasks used them.

During real-robot testing, the LiDAR and odometry data were noisier than in simulation.

We reduced the map update rate, but the resulting map was still distorted.

The custom implementation includes exploration and occupancy grid mapping. It is not a complete SLAM system because it does not include localisation during map updates.


## Challenges We Faced

### 1. Path and Motion Planning

**Coordinate conversion**

At first, the robot sometimes moved in unexpected directions. We had problems converting between world coordinates and grid coordinates.

We corrected the transformations and adjusted the robot's starting position.

**Robot speed and potential field parameters**

The real robot moved faster than the simulation with the same velocity commands.

We adjusted the linear velocities and the attractive and repulsive force parameters.

Sideways movement also required a lower speed because the LiDAR could not detect all obstacles beside the robot.

**Invalid LiDAR data**

Some obstacle coordinates contained `NaN` values.

These values caused the calculated repulsive force to become `NaN`, so the robot could not move.

We filtered out the invalid values.

**Occupancy grid and replanning**

We initially created a new occupancy grid every time the robot needed a new path.

This removed information about obstacles found earlier.

We changed the code to keep one grid and update it when new obstacles were detected.

We also fixed problems with deleting waypoints and checking whether the final goal had really been reached.

**Real-robot testing**

We experienced problems with threading locks. Some locks blocked other parts of the code, and the robot could not react quickly enough.

We reduced the number of locks used in the callbacks and state methods.

We also added obstacle enlargement. Nearby cells around detected obstacles are marked as occupied.

This helped the real robot keep a safer distance from obstacles when planning its path.

### 2. Localisation

**Global localisation**

The first sensor model used a likelihood field. However, it could fail when the particles were concentrated around the wrong position.

We changed the sensor model to beam-based ray casting.

This helped the filter recover from incorrect estimates when the robot moved.

**Particle recovery**

Sometimes the particles became concentrated around an incorrect position.

We added 5% random particles after every resampling step to help the filter recover.

**ROS 2 integration**

We also had to solve problems with TF frames, simulation time and communication between the laptop and the real robot.

### 3. Environment Exploration

**Potential field parameters**

The robot needed enough attractive force to move towards the next goal and enough repulsive force to avoid obstacles.

Finding suitable values required several tests.

**Map size and resolution**

We had to select a map size and resolution that covered the entire environment.

If the map was too small, some areas were outside its boundaries and could not be explored.

**Mirrored map**

The custom occupancy grid appeared mirrored because of coordinate conversion problems.

Exchanging the x and y coordinates corrected the displayed map during testing, but the change was not added to the shared conversion methods.

**Unknown-cell exploration**

We tried to use A* to reach unknown cells after the robot had explored enough of the environment.

However, the planner could not always find a valid path to these cells.

**Noisy real-robot measurements**

The real robot's LiDAR and odometry measurements were noisier than those in simulation.

We reduced the map publishing rate, but the map was still distorted after the robot completed a full round.

Additional filtering and localisation would be needed to improve the real-robot mapping results.

## References

- [A* Search Algorithm in Python – GeeksforGeeks](https://www.geeksforgeeks.org/python/a-search-algorithm-in-python/)
- [Robile ROS Repository](https://github.com/HBRS-AMR/Robile)
