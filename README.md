# 🤖 Gesture Controlled Robot using IMU with micro-ROS & ROS 2 (ESP32-based)

Welcome to the **Gesture Controlled Robot** project!
Here we bring together **ESP32 + MPU6050 IMU + micro-ROS + ROS 2** to control and visualize robot movements using hand gestures.

---

## 🎥 ESP32+MPU6050 Gesture Control Demo  
https://github.com/user-attachments/assets/ebf6ab36-b1be-4994-98d0-e850d915f325

---

This project shows how you can:

* 📡 Stream real-time IMU data from an **ESP32** running micro-ROS.
* 🔗 Bridge it to a **ROS 2 agent** over serial USB.
* 🎛️ Visualize orientation and motion data inside **RViz2**.
* 🦾 Control a robot in simulation (and extend it to real robots).

---

## 🛠️ Hardware Setup (MPU6050 ↔ ESP32 I²C Wiring)

**Connections (default ESP32 pins for I²C):**

| MPU6050 Pin | ESP32 Pin                          |
| ----------- | ---------------------------------- |
| **VCC**     | 3.3V                               |
| **GND**     | GND                                |
| **SDA**     | GPIO21                             |
| **SCL**     | GPIO22                             |

👉 You can remap pins with `Wire.begin(SDA_pin, SCL_pin)` if you need different GPIOs.

---

## 💻 Software Requirements

* ROS 2 (Humble or compatible distro)
* [micro-ROS setup tools](https://micro.ros.org/docs/tutorials/core/first_application_linux/) 🛠️
* Arduino IDE with [`micro_ros_arduino`](https://github.com/micro-ROS/micro_ros_arduino) library
* Java (for running Telemetry Viewer `.jar` if used) → [Download from FarrellF](http://www.farrellf.com/TelemetryViewer/) ☕

---

## ⚡ Install `micro_ros_arduino` in Arduino IDE

1. Open Arduino IDE → `Sketch` → `Include Library` → `Manage Libraries...`
2. Search for **micro\_ros\_arduino** and install.
   *(Or download the repo as ZIP from [GitHub](https://github.com/micro-ROS/micro_ros_arduino) and add manually.)*
3. Open example sketches under `File → Examples → micro_ros_arduino`.
4. Select your ESP32 board & correct port, then upload.

---

## 📊 Telemetry Viewer Setup

1. Install Java (JRE/JDK).
2. Download `TelemetryViewer.jar` (or the tool provided).
3. Run it with:

   ```bash
   java -jar TelemetryViewer.jar
   ```
4. Configure serial port (e.g., `/dev/ttyUSB0`) and baud rate (e.g., `115200` or `921600`).

---

## 🔌 Running the micro-ROS Agent

On your ROS 2 host:

```bash
# Create agent workspace
ros2 run micro_ros_setup create_agent_ws.sh

# Build the agent
ros2 run micro_ros_setup build_agent.sh

# Source environment
source install/local_setup.bash

# Run the agent via USB serial
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600 -v4
```

⚠️ Replace `/dev/ttyUSB0` with your board’s serial port (check with `ls /dev/serial/by-id/*`).

---

🧩 **Run URDF**

```bash
ros2 run robot_state_publisher robot_state_publisher ~/gesture_ws/src/imu_visualizer/urdf/imu_cube.urdf & rviz2
```
---

## 🚀 Run the Full Project

Open **3 terminals** (each sourced to your ROS 2 workspace).

1️⃣ **Start the micro-ROS Agent**

```bash
ros2 run robot_state_publisher robot_state_publisher ~/imu_ws/src/imu_visualizer/urdf/imu_cube.urdf & rviz2
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600 -v4
```

2️⃣ **Bring up the simulated robot**

```bash
ros2 launch bot_bringup simulated_robot.launch.py
```

3️⃣ **Run IMU controller (subscriber + logic)**

```bash
ros2 launch imu_visualizer imu_controller.launch.py
```

4️⃣ **Visualize IMU in RViz**

```bash
ros2 launch imu_visualizer view_imu.launch.py
```

👉 Now you can **see your IMU orientation in RViz** and control the robot with gestures 🎉

---

## 🧩 Troubleshooting

* **No data?** → Check wiring (SDA/SCL), baud rate, and serial port.
* **Permission denied on `/dev/ttyUSB0`?** →

  ```bash
  sudo usermod -a -G dialout $USER
  ```

  *(Log out and back in after adding user to `dialout`.)*
* **IMU outputs 0** → Ensure VCC = 3.3V and address (AD0 pin).

---

## 🙌 Credits

* 💡 Original repo: [MechaMind-Labs/Gesture\_Controlled\_Robot](https://github.com/MechaMind-Labs/Gesture_Controlled_Robot)
* 🧑‍💻 Maintained & documented by [Curious-Utkarsh](https://github.com/Curious-Utkarsh)
* 🌐 [micro-ROS Tutorials](https://micro.ros.org/docs/tutorials/core/first_application_linux/)

---

✨ That’s it — You’ve built your own **Gesture Controlled Robot with ESP32 + micro-ROS + ROS 2**!

---
