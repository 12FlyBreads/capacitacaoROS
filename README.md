# 🤖 ROS 2 Training - Drone Simulation

A ROS 2 training project by the **Black Bee Drones** team.

The goal of this project is to simulate a drone's attributes and behavior (such as motor power, estimated position, and mission state), using the communication between ROS 2 nodes to orchestrate a simple task. The entire simulation runs in the terminal, focusing on ROS architecture and communication.

---

## 🎯 The Drone's Mission

Unlike a complex graphical simulation, this project focuses on a mission task with well-defined states, controlled entirely by ROS 2 nodes:

1.  **TAKEOFF**
    * The drone starts at the point `(0, 0, 0)` and ascends until it reaches a height of 5 meters.

2.  **MISSION**
    * The drone moves towards a destination point.
    * During its path, if the LiDAR sensor detects an obstacle, the drone must dodge it on the X-axis, go around it, and then return to its original route (X=0).

3.  **LAND**
    * Upon reaching the final destination, the drone begins the landing procedure until it is on the ground.

4.  **IDLE**
    * After landing, the drone completes its mission and remains in an idle state.

> **Note**: Flight physics (pitch, yaw, roll) and motor power have been generalized to simplify the simulation. The main focus of the project was the practical application of communication between multiple nodes in the ROS 2 ecosystem.

---

## 🛠️ Technologies Used

![Python](https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white)
![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white)

---

## 📁 Project Structure

All the source code for this project is in the `capacitacao` branch. The ROS 2 workspace (`ros2_ws`) is structured as follows:

```bash
.
├── src
│   └── sim_drone           # ROS 2 Package
│       ├── launch
│       │   └── sim.launch.py   # File to start all nodes
│       ├── sim_drone
│       │   ├── __init__.py
│       │   ├── drone_node.py     # Main drone node (controls the mission)
│       │   ├── lidar_node.py     # LiDAR sensor node (publishes detections)
│       │   └── obstaculos.py   # Node that simulates the existence of obstacles
│       ├── package.xml
│       ├── setup.cfg
│       └── setup.py
└── ...
```

---

## 🚀 How to Run

To run the simulation, make sure you have ROS 2 (Humble or newer) installed and follow the steps below in your terminal.

1.  **Clone the repository**
    ```bash
    cd ~/ros2_ws/src
    git clone git@github.com:12FlyBreads/capacitacaoROS.git
    cd capacitacaoROS
    ```

2.  **Check out the correct branch**
    ```bash
    git checkout capacitacao
    ```

3.  **Build the ROS 2 package**
    (Navigate to the root of your workspace, e.g., `cd ros2_ws`)
    ```bash
    colcon build --packages-select sim_drone
    ```

4.  **Source the environment**
    ```bash
    source install/setup.bash
    ```

5.  **Run the launch file**
    The following command will start all the necessary nodes for the simulation.
    ```bash
    ros2 launch sim_drone sim.launch.py
    ```

You will now see the drone's status printed in your terminal as it executes the mission!
