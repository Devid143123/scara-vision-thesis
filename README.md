# SCARA Vision-Guided Color-Sorting Robot with Digital Twin

A joint Bachelor thesis project at the **Institute of Technology of Cambodia (ITC)**.

This repository contains the full source code of a vision-guided pick-and-place
SCARA robot. The project is built on top of the mechanical platform of the
previous thesis by **SRONG Ougy** and **VANN Panha**, and was developed by:

- **SARIN Chandevid** — software side: simulation, ROS 2, kinematics, YOLO model,
  digital-twin integration. See [`software/`](software/).
- **SAR Seihak Reach** — hardware side: motor electronics, real-time joint
  control, vacuum gripper, camera calibration. See [`hardware/`](hardware/).

The two halves communicate over a local wireless network using standard ROS 2
topics.

---

## Repository layout

```
.
|-- software/                   <- SARIN's work (run on the simulation computer)
|   |-- scara_sim/              <- URDF + RViz simulation (mock_components hardware)
|   |   |-- urdf/scara.urdf.xacro
|   |   |-- meshes/             <- STL meshes from SolidWorks
|   |   |-- config/             <- controllers.yaml, scara.rviz
|   |   `-- launch/sim.launch.py
|   `-- scara_trajectory/       <- Trajectory + digital-twin nodes
|       `-- scara_trajectory/
|           |-- ik_to_rviz_controller.py   <- digital-twin bridge
|           |-- color_cube_visualizer.py   <- cube markers in RViz
|           |-- home_position.py
|           |-- pick_place_demo.py
|           |-- square_demo.py
|           |-- recorder.py                <- log /joint_states to CSV
|           |-- plot_results.py            <- chapter 4 trajectory graphs
|           |-- twin_recorder.py           <- log real + sim simultaneously
|           |-- plot_twin.py
|           `-- plot_twin_xy.py
|
|-- hardware/                   <- Reach's work (run on the robot computer)
|   |-- SCARA_pkg/              <- main hardware code
|   |   `-- SCARA_pkg/
|   |       |-- calibrateauto.py            <- automated calibration routine
|   |       |-- camdetect.py                <- camera-based cube detection
|   |       |-- ikpos.py / fkpos.py         <- IK and FK
|   |       |-- newposition.py              <- position commander
|   |       |-- position.py / positionradient.py
|   |       |-- motor_selection.py
|   |       |-- steppertest.py
|   |       `-- *.json                      <- calibration data
|   |-- can_driver/             <- CAN bus driver for ODrive motors
|   |-- custom_messages/        <- custom .msg / .srv types
|   |-- testing_can/            <- bench tests (solenoid, encoder, relay)
|   |-- yolo_pick_place/        <- YOLO + pick-place node
|   |   |-- yolo_pick_place/
|   |   |   |-- yolopickplace.py
|   |   |   |-- yolotest.py
|   |   |   |-- camcal.py
|   |   |   `-- resultplot.py
|   |   `-- launch/webcamtest.launch.py
|   |-- yolo_ros/               <- third-party YOLO ROS bringup (vendored)
|   `-- scara_physical_square.csv  <- recorded hardware trajectory data
|
|-- .gitignore
`-- README.md                   <- this file
```

> **Note on weight files.** The YOLO `.pt` weight files (best.pt, yolov8m.pt,
> yolov8n.pt) are not stored in this repository because of GitHub size limits.
> They can be downloaded from the official Ultralytics releases or trained
> from the dataset described in the thesis. Place the resulting weights in
> `hardware/yolo_pick_place/models/`.

---

## Quick start

### 0. Prerequisites

Both computers run **Ubuntu 22.04** with **ROS 2 Humble** installed.
On both computers, set the same domain ID so the topics can flow between them:

```bash
export ROS_DOMAIN_ID=42
```

### 1. Building

On the **simulation computer** (SARIN):

```bash
mkdir -p ~/ros2_ws/src
cp -r software/scara_sim ~/ros2_ws/src/
cp -r software/scara_trajectory ~/ros2_ws/src/
cd ~/ros2_ws
colcon build
source install/setup.bash
```

On the **robot computer** (Reach):

```bash
mkdir -p ~/ros2_ws/src
cp -r hardware/* ~/ros2_ws/src/
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 2. Running the digital twin

**Simulation computer (SARIN):**

```bash
# Terminal 1: launch the URDF simulation in RViz
ros2 launch scara_sim sim.launch.py

# Terminal 2: digital-twin controller (mirrors the real robot)
ros2 run scara_trajectory ik_to_rviz_controller

# Terminal 3: coloured-cube visualiser
ros2 run scara_trajectory color_cube_visualizer
```

**Robot computer (Reach):**

```bash
# Camera calibration / auto-calibration
ros2 run SCARA_pkg calibrateauto

# YOLO + pick-and-place (webcam test launch)
ros2 launch yolo_pick_place webcamtest.launch.py
```

When both sides are running, the physical robot's motion is mirrored in
real time inside RViz on the simulation computer, and the detected cubes
appear as coloured markers in the simulated workspace.

### 3. Stand-alone demos (software side only)

```bash
ros2 run scara_trajectory square_demo       # 100 mm square trajectory
ros2 run scara_trajectory pick_place_demo   # single pick-and-place cycle
ros2 run scara_trajectory home_position     # go home
```

### 4. Recording and plotting data for the report

```bash
# Record /joint_states while the arm moves
ros2 run scara_trajectory recorder

# Record real-robot + simulation angles simultaneously
ros2 run scara_trajectory twin_recorder

# Generate the thesis graphs from the CSV files
python3 src/scara_trajectory/scara_trajectory/plot_results.py
python3 src/scara_trajectory/scara_trajectory/plot_twin.py
python3 src/scara_trajectory/scara_trajectory/plot_twin_xy.py
```

---

## Environment summary

| Tool         | Version              |
|--------------|----------------------|
| OS           | Ubuntu 22.04         |
| ROS 2        | Humble               |
| Python       | 3.10+                |
| Simulation   | mock_components + RViz |
| Vision       | YOLOv8 (Ultralytics) |
| CAD          | SolidWorks + SW2URDF |
| Motors       | ODrive via CAN       |
| Gripper      | Pneumatic vacuum     |

---

## Acknowledgements

The mechanical platform was inherited from the senior thesis of
**SRONG Ougy** and **VANN Panha**, who generously shared their design files.

The weekly progress meetings with our advisor **Asst. Prof. Dr. SRANG Sarot**,
head of the ECAM-LaSalle Phnom Penh and Dynamics and Control Laboratory (DCLab),
were essential to align the two halves of this project despite the team being
geographically split between France and Cambodia.

---

## License

MIT - see LICENSE.
