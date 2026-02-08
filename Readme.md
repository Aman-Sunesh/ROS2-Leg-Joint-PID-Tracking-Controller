# ROS2 Leg Joint PID Tracking Controller (ROB-UY 2004 — Lab 01)

A small ROS 2 controller node used in **ROB-UY 2004 (Robotic Locomotion & Manipulation)** Lab 01 to command **effort/torque** on one Pupper leg joint (front-left joint 3) and **track** a “leader” joint (front-right joint 3) using **P / PD / PID** control.

This repo is a fork/refactor of the course base code, with a working leader–follower PID implementation and some safety/quality-of-life improvements (deadband, anti-windup clip, SIGINT zero-torque on exit).

---

## What it does

- Subscribes to: ` /joint_states ` (`sensor_msgs/JointState`)
- Publishes torque commands to:
  - ` /forward_command_controller/commands ` (`std_msgs/Float64MultiArray`)
  - ` /forward_command_controller_lead/commands ` (`std_msgs/Float64MultiArray`)

**Control modes (toggle in `lab_1.py`):**
1. **Pendulum mode**: swings the follower joint back and forth with a small constant effort.
2. **Leg tracking mode (leader → follower)**: makes **front-left joint 3** track the (mirrored) position of **front-right joint 3** using PID:
   - Target position: `target_joint_pos = - joint_pos_lead`
   - Target velocity: `0` (by default)

---

## Repository structure (relevant files)

```
Labs/
  Lab1/
    lab_1.py          # main controller node (PID + publishers/subscribers)
    lab_1.launch.py   # launches ros2_control + state publishers + controllers
    lab_1.yaml        # controller_manager configuration (joints + interfaces)
README.md             
```

---

## How to run (typical Lab 01 flow)

### 0) Put Pupper on the stand
Do **not** run this with the robot on the ground.

### 1) SSH into the robot + go to your repo directory
```bash
ssh pi@<PUPPER_IP>
cd /path/to/your/repo/Labs/Lab1
```

### 2) Launch the ROS graph (controllers + state publishers)
Run the launch file:
```bash
ros2 launch lab_1.launch.py
```

This will start:
- `ros2_control_node` (controller manager)
- `robot_state_publisher`
- spawners for:
  - `joint_state_broadcaster`
  - `imu_sensor_broadcaster`
  - `forward_command_controller` (follower)
  - `forward_command_controller_lead` (leader)

### 3) Start the controller node (in a second terminal)
In another terminal (SSH or VSCode Remote-SSH terminal):
```bash
cd /path/to/your/repo/Labs/Lab1
python3 lab_1.py
```

### 4) Stop safely
Press `CTRL-C`. The code installs a SIGINT handler that publishes **zero torque** before shutdown.

---

## Configuration

### `lab_1.yaml` (controllers)
- Follower controller controls: `leg_front_l_3`
- Leader controller controls: `leg_front_r_3`
- Both controllers expose interfaces: `['effort', 'kp', 'kd']`
- Joint states are broadcast for 12 joints (all legs), position interface.

### `lab_1.launch.py` (launch graph)
Loads the Pupper URDF (`pupper_v3_description`) via xacro, starts `controller_manager`, then spawns controllers after the joint state broadcaster is up (using `OnProcessExit` event handlers).

---

## Controller logic (lab_1.py)

### Topics & message format
This lab uses `Float64MultiArray` for commands:
```python
command_msg.data = [effort, kp, kd]
```
In this implementation, `kp` and `kd` are set to `0.0` and all control is done in Python (effort output only).

### PID (leg tracking mode)
- Error: `e = target_pos - pos`
- Derivative term (as implemented): `target_vel - vel`
- Integral: `sum_error += e * dt`, clipped to `[-0.3, 0.3]` (anti-windup)
- Output: `torque = KP*e + KI*sum_error + KD*e_dot`

**Safety/robustness helpers:**
- `MAX_TORQUE` clamp (default `2.0`)
- Deadband: ensures the commanded torque exceeds a minimum magnitude to overcome stiction:
  - if torque > 0 → `max(torque, DEAD_BAND_SIZE)`
  - if torque < 0 → `min(torque, -DEAD_BAND_SIZE)`

### Key parameters (top of `lab_1.py`)
- `KP`, `KI`, `KD` – tuning gains
- `LOOP_RATE = 200` Hz, `DELTA_T = 1/LOOP_RATE`
- `MAX_TORQUE = 2.0`
- `DEAD_BAND_SIZE = 0.095`
- Mode toggle:
  - `PENDULUM_CONTROL = False`
  - `LEG_TRACKING_CONTROL = not PENDULUM_CONTROL`

---

## Tuning tips (quick + practical)

1. Start **very small** gains, especially `KP`, then increase until you see responsive tracking.
2. If you see overshoot/oscillation, reduce `KP` slightly and increase `KD` a bit.
3. Add `KI` only after you have a stable PD controller; increase slowly to reduce steady-state error.
4. If the leg “sticks” and doesn’t move for small errors, the **deadband** may help; but too large a deadband can cause chattering.

---

## Notes

- This code assumes `JOINT_NAME` and `JOINT_NAME_LEAD` exist in `/joint_states`. If not, `msg.name.index(...)` will throw.
- Target mirroring uses: `target_joint_pos = -joint_pos_lead`. If your robot coordinate conventions differ, adjust the sign.
- Ensure your ROS environment is sourced on the robot (course setup typically handles this).

---

## Credits

- Course base code and lab spec: **ROB-UY 2004** (Christopher Clark).
- This repo: personal fork/refactor with PID tracking implementation and cleanup.
