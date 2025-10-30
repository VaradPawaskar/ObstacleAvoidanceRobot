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
