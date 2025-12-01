# 🤖 NED-2 Tic-Tac-Toe — ROS2 Humble Project

This project demonstrates a **Tic-Tac-Toe game played autonomously by a Niryo NED-2 robotic arm** using **ROS2 Humble**, **URDF/Xacro**, **JointState control**, and **RViz2 visualization**.

The robot:

- ✔ Moves to Tic-Tac-Toe board cells  
- ✔ Simulates pick & place from home  
- ✔ Opens/closes gripper  
- ✔ Returns home after every action  
- ✔ Detects win/draw  
- ✔ Plays against the user with a simple AI  

This project is ideal for academic submissions, robotics demos, and portfolio showcasing.

---

## 📁 Workspace Structure
ros2_ws/
├── src/
│ ├── niryo_robot_description/
│ └── ned2_control/
│ ├── package.xml
│ ├── setup.py
│ ├── setup.cfg
│ └── ned2_control/
│ └── pick_place_demo.py

---

# 🛠 Requirements

- ROS2 Humble  
- colcon build tools  
- RViz2  
- Niryo NED-2 description package  
- Python3  

---

# 🚀 Installation

Clone this repository into your workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/sanskarmudnur2005/TIC_TAC_TOE_NIRYO_NED_2.git
```

Build your workspace
```
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

# ▶️ Running the Simulation

Open three terminals.

🟦 Terminal 1 — Robot State Publisher + URDF
```
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Build URDF from xacro
ros2 run xacro xacro \
  ~/ros2_ws/src/niryo_robot_description/urdf/ned2/niryo_ned2_gripper1_n_camera.urdf.xacro \
  -o /tmp/ned2.urdf

# Publish TF for the robot
ros2 run robot_state_publisher robot_state_publisher /tmp/ned2.urdf
```
leave this running <br>
<br>

🟩 Terminal 2 — RViz2 Viewer
```
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

rviz2
```
Leave this running
<br>
In RViz:<br>
Add → RobotModel<br>
Set Fixed Frame = base_link<br>
Robot should appear fully in RViz<br>
<br>

🟥 Terminal 3 — Tic-Tac-Toe Game Controller
```
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run ned2_control pick_place_demo
```

This will start the game:<br>
Enter cell (1–9), or q:<br>

---

# 🤖 Robot Action Sequence<br>
Move to HOME<br>
Simulate PICK<br>
Move to selected cell<br>
Simulate PLACE<br>
Return HOME<br>
Computer makes its move<br>
Win/draw detection triggers
<br><br>

---

# 🧠 Game Logic Features
Full win detection for:<br>
3 rows<br>
3 columns<br>
2 diagonals<br>
Draw detection<br>
Computer makes valid random moves<br>
Gripper movement synchronized with arm motion<br>
Smooth joint interpolation animation<br>
Robot always returns to HOME after every action<br>
<br>

---

# 🎮 Game Overview
You play as X, computer plays as O.<br>
Board positions:<br>
 1 | 2 | 3 <br>
----------- <br>
 4 | 5 | 6 <br>
----------- <br>
 7 | 8 | 9 <br>
<br>

---

# 📝 Project Summary
This project demonstrates how a robotic arm can be controlled through ROS2 using
URDF models, TF broadcasting, and joint state publishing.  
It combines robotics with a classic game — **Tic Tac Toe** — to create a fun,
interactive demonstration of motion control, game logic, and simulation skills.
<br>
The goal is to show that even with basic ROS2 functionalities, it is possible to
create engaging robotic behaviors suitable for academic presentations,
assignments, and beginner-level robotics research.<br>
<br>

---

# ⚙️ How It Works
1. **URDF/Xacro loads the NED-2 robot model**  
   RViz visualizes the robot using robot_state_publisher.
2. **JointState messages control the robot**  
   The arm and gripper positions are updated in real time using Python code.
3. **You choose a Tic Tac Toe cell (1–9)**  
   The robot moves to that cell and simulates placing an X.
4. **Computer selects a empty cell**  
   The robot moves again and simulates placing an O.
5. **Win/draw conditions are checked**  
   If someone wins, the game stops.

This provides a very simple version of pick-and-place logic using only joint
control—no MoveIt, no Gazebo, and no complex planning.
<br>

---

# ⭐ Features

| Feature | Status |
|--------|--------|
| RViz2 robot visualization | ✅ |
| Manual Tic Tac Toe input | ✅ |
| Robot move simulation | ✅ |
| Gripper open/close | ✅ |
| Computer AI (random moves) | ✅ |
| Win/draw detection | ✅ |
| Pick-from-home motion imitation | ✅ |
| Full Gazebo support | ⚠️ Optional (may crash on some systems) |

---

# 🔧 Future Enhancements
Smarter AI (Minimax Algorithm)<br>
MoveIt2 integration for real trajectory planning<br>
RViz markers for X/O tokens<br>
Real Hardware Mode for physical NED-2<br>
Robot animation when it wins 😄<br>
<br>


