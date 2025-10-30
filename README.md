# Obstacle Avoidance Robot (ROS & Gazebo)

This project is an implementation of an autonomous obstacle-avoiding robot, developed as part of a Master's in AI and Robotics. The robot uses sensor data to navigate a simulated environment in Gazebo while avoiding collisions.

## Technologies Used

* **ROS (Robot Operating System)**: For communication between different parts of the robot (nodes).
* **Gazebo**: For simulating the robot and the environment.
* **Python**: For the obstacle avoidance logic and motion control.
* **C++**: For ROS node development.
* **catkin**: As the build system for ROS.

---

##  How to Run the Project

Follow these steps to set up the build environments and launch the simulation.

### 1. Prerequisites

* Ensure you have a working installation of ROS (e.g., Noetic, Melodic) and Gazebo.
* Clone this repository to your local machine:
    ```bash
    git clone [https://github.com/VaradPawaskar/ObstacleAvoidanceRobot.git](https://github.com/VaradPawaskar/ObstacleAvoidanceRobot.git)
    cd ObstacleAvoidanceRobot
    ```

### 2. Build Workspaces

You must build both the `catkin_ws` and `simulation_ws` workspaces.

```bash
# Build the first workspace
cd catkin_ws
catkin_make
source devel/setup.bash
cd ..

# Build the second workspace
cd simulation_ws
catkin_make
source devel/setup.bash
cd ..
```
## 3. Launch the Simulation
Open a new terminal for each of the following commands.

1. Launch Gazebo World: This command starts the Gazebo simulation environment with the chosen world.
```bash
roslaunch my_worlds <world_name>.launch
```
2. Spawn the Robot: In a new terminal, run this command to load the robot model into the environment at the default position (0,0,0).
   ```bash
   roslaunch robot_description spawn.launch`
   ```
3. Start the Obstacle Avoidance Algorithm: In a third terminal, run this command to start the robot's motion and activate the collision avoidance script.
   ```bash
   rosrun motion_plan obstacle_avoidance.py
   ```
   The robot will now start moving and autonomously avoid any obstacles it detects.

# Acknowledgements
This project was developed with reference to the "Exploring ROS using 2-Wheeled Robot" project from The Construct.

Reference: https://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1
