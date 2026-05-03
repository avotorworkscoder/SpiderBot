# SpiderBot V4.1 — Digital Twin Guide
## Physical-to-Virtual Synchronization Methods + CoppeliaSim Integration

---

## Part 1 — How Robotics Professionals Mirror Physical Robots in Simulation

There are **6 major categories** of physical-to-virtual synchronization, ranging from simple data streaming to full bidirectional digital twins.

---

### Method 1 ▸ Joint-Angle Telemetry Streaming *(What SpiderBot V4.1 uses)*

> **Concept:** Read actual or computed joint angles from onboard sensors/firmware → stream them over a communication channel → apply directly to the virtual model's joints.

**Variants:**
| Sub-type | Source of angles | Notes |
|---|---|---|
| **IK-computed (software)** | Computed angles inside the MCU from IK solver | No physical encoders needed — used in SpiderBot V4.1 |
| **Encoder-read (hardware)** | Real angle from motor encoders / IMUs | True physical state, most accurate |
| **Observer-estimated** | Kalman / complementary filter fusing IMU + cmd | Estimated state between sensor reads |

**SpiderBot V4.1 approach:** The ESP32's IK solver (`moveFootXYZ`) computes coxa/femur/tibia angles then stores them in `currentJointAngles[]`. These are serialised as 18 comma-separated floats over USB Serial @ 115200 baud and consumed by a Python ZMQ bridge that calls `sim.setJointTargetPosition()` on each frame.

---

### Method 2 ▸ Forward-Kinematics (FK) Pose Replay

> **Concept:** Record the full sequence of ***commands*** sent to the robot (not its response), then replay them on the virtual model using the same kinematic equations.

- Used in: offline gait analysis, classroom demos, replay-based debugging.
- Limitation: doesn't capture physical deformation, slip, or gravity effects.
- Often implemented as a **trajectory log file** (CSV / ROS bag) replayed in sim.

---

### Method 3 ▸ ROS / Middleware State Publisher

> **Concept:** The robot publishes its full state (`/joint_states` topic) into a ROS network. The simulator subscribes to the same topic via a ROS–sim bridge plugin.

- Used in: ROS 2 + Gazebo / RViz2 setups (most common in academia/industry).
- The robot runs `robot_state_publisher` → sim runs a subscriber node.
- **CoppeliaSim has a ROS 2 plugin** (`simROS2`) that can subscribe to `/joint_states` natively.
- Requires a full ROS 2 stack on the robot computer.

---

### Method 4 ▸ Motion Capture (MoCap) Re-targeting

> **Concept:** Optical or IMU-based motion capture tracks the physical robot's links in 3-D space. The 3-D pose is retargeted to the virtual skeleton in real time.

- Used for: humanoids, quadrupeds (Boston Dynamics style), biomechanics research.
- Hardware: Vicon / OptiTrack (optical), or Xsens / APDM (IMU suits).
- Latency is typically 5–30 ms — near real-time.
- Overkill for a hexapod but the gold standard for legged robots.

---

### Method 5 ▸ Vision-Based Pose Estimation

> **Concept:** Camera(s) observe the physical robot. Computer vision (OpenPose, ArUco markers, depth sensing) estimates joint angles from images.

- Used in: labs without encoders, legacy hardware integration.
- Tools: OpenCV, MediaPipe, Intel RealSense.
- Accuracy limited by occlusion and camera calibration.

---

### Method 6 ▸ Full Bidirectional Digital Twin (Sim-in-the-Loop)

> **Concept:** Simulation and physical robot run simultaneously. The sim predicts the next state; the physical robot tries to match; errors feed back to update the sim model.

- Used in: industry (Siemens, NVIDIA Omniverse), advanced research.
- Includes physics-based contact modelling, wear estimation, predictive maintenance.
- Requires a real-time capable sim engine (Isaac Sim, Webots, MuJoCo+hardware).

---

## Part 2 — All Ways to Do It *Specifically* in CoppeliaSim

CoppeliaSim (formerly V-REP) supports **5 distinct control APIs**, each with different latency, language support, and use-case fit.

---

### CoppeliaSim Method A ▸ ZeroMQ Remote API *(SpiderBot V4.1 uses this)*

```
ESP32 Serial → Python (pyserial) → ZMQ socket → CoppeliaSim plugin
```

- **API:** `coppeliasim_zmqremoteapi_client` (Python / C++ / Matlab / Lua)
- **Key call:** `sim.setJointTargetPosition(handle, angle_rad)`
- **Latency:** ~2–10 ms per call (local IPC)
- **Blocking:** Non-blocking by default; synchronous mode available
- **Best for:** Real-time streaming at moderate rate (< 200 Hz)

✅ **Already working in your stack.** Python bridge reads serial → calls ZMQ API.

---

### CoppeliaSim Method B ▸ Legacy Remote API (C/C++/Python/Lua/Java/Matlab)

- Uses `simxSetJointTargetPosition()` over TCP port 19997
- Deprecated in CoppeliaSim 4.x but still available via the `remoteApi` plugin
- Replaced by ZMQ API (Method A) — **do not start new projects with this**

---

### CoppeliaSim Method C ▸ Embedded Lua / Python Child Script

> Write the bridge logic directly inside CoppeliaSim as an **associated child script** on the robot model.

```lua
-- Inside CoppeliaSim Lua child script
function sysCall_actuation()
    local angle = sim.getFloatSignal('leg0_coxa')  -- set by external process
    sim.setJointTargetPosition(joint_handle, angle)
end
```

- External process sets `sim.setFloatSignal()` via ZMQ or shared memory.
- Lua polls the signal every simulation step.
- **Very low overhead** — runs inside the sim's own scheduler.
- Good for: signal-based or event-driven control.

---

### CoppeliaSim Method D ▸ ROS / ROS 2 Bridge Plugin

- Install `simROS2` plugin → CoppeliaSim subscribes to `/joint_states`
- ESP32 → micro-ROS → ROS 2 `/joint_states` → CoppeliaSim
- **Most scalable** approach for multi-robot or sensor-rich systems
- Overkill for SpiderBot unless you are already on a ROS 2 stack

---

### CoppeliaSim Method E ▸ Shared Memory / Named Pipes (Advanced)

- Fastest possible IPC on Windows/Linux: `mmap` / named pipes
- A C++ bridge process writes angles to shared memory; a CoppeliaSim plugin reads it
- Used for ultra-low latency (< 1 ms) control loops
- Requires writing a custom CoppeliaSim plugin in C++

---

## Part 3 — Current SpiderBot V4.1 Stack Analysis

### Architecture Diagram

```
[ PS2 Controller ]
        │  SPI
        ▼
[ ESP32 — IK Solver ]  ──UART2──►  [ LSC-32 Servo Controller ] ──► Physical Servos
        │
        │  USB Serial @ 115200 baud
        │  Format: "c0,f0,t0,c1,f1,t1,...,c5,f5,t5\n"  (18 floats, 1 line)
        │  Sent only when anglesDirty==true (gait tick fires it)
        │
        ▼
[ pythonBridgeScript.py ]
        │  readline() → parse 18 CSV floats
        │  Apply coordinate corrections (Z-axis flip, coxa bias)
        │  ZMQ calls
        ▼
[ CoppeliaSim ZMQ API ]
        │  setJointTargetPosition(handle, rad) × 18
        ▼
[ Virtual SpiderBot in CoppeliaSim ]
```

### What the current `sendAnglesToPC()` does

```cpp
// Triggered ONLY when anglesDirty == true (set inside moveFootXYZ)
void sendAnglesToPC() {
  for (int i = 0; i < 6; i++) {
    uint8_t id = legs[i].id;
    Serial.print(currentJointAngles[id],   2); Serial.print(",");
    Serial.print(currentJointAngles[id+1], 2); Serial.print(",");
    Serial.print(currentJointAngles[id+2], 2);
    if (i < 5) Serial.print(",");
  }
  Serial.println();   // terminates the line → Python readline() picks it up
}
```

**Output format (single line per gait tick):**
```
16.70,-72.34,142.10,  -5.71,-72.34,142.10,  -21.80,-72.34,142.10,  0.00,-72.34,142.10,  11.31,-72.34,142.10,  31.00,-72.34,142.10
```
18 values, comma-separated, newline-terminated. ✅

---

## Part 4 — Problem Diagnosed: `btLog` is Polluting the Telemetry Stream

### The Bug

`btLog()` writes to ***both*** `SerialBT` AND `Serial.print()`:

```cpp
void btLog(const char* msg) {
    SerialBT.print(msg);
    Serial.print(msg);      // ← THIS goes to the same COM port the Python bridge reads!
}
```

So every time the bot logs `>> Tripod Walk: Forward\n`, that text line appears on the serial port **mixed in with** angle packets. The Python bridge handles this with `try/except ValueError: pass` which silently drops corrupted packets — but this is fragile and wasteful.

### The Fix

`btLog` should write to `SerialBT` only. A separate `pcLog()` or dedicated `Serial2` could be used for debug if needed. The telemetry channel (`Serial`) must be kept ***angle-packets only***.

---

## Part 5 — Recommended Code Change for V4.1

### Change 1: Isolate Serial from btLog

```cpp
// BEFORE (pollutes telemetry port):
void btLog(const char* msg) { SerialBT.print(msg); Serial.print(msg); }

// AFTER (telemetry port stays clean):
void btLog(const char* msg) { SerialBT.print(msg); }
```

### Change 2: One-line per button press (on-demand snapshot)

Add a dedicated command (e.g., `'T'` for Telemetry) that sends a **single snapshot** of the current joint angles on demand — useful for testing without running the full gait:

```cpp
case 'T': case 't':
  sendAnglesToPC();
  break;
```

This lets you press a BT debug button and immediately see one line of angles on the Python side without needing to start a gait.

### Change 3: Prefix telemetry packets to differentiate from debug text

```cpp
void sendAnglesToPC() {
  Serial.print("A:");   // 'A' prefix — Python filters lines that start with 'A:'
  for (int i = 0; i < 6; i++) {
    uint8_t id = legs[i].id;
    Serial.print(currentJointAngles[id],   2); Serial.print(",");
    Serial.print(currentJointAngles[id+1], 2); Serial.print(",");
    Serial.print(currentJointAngles[id+2], 2);
    if (i < 5) Serial.print(",");
  }
  Serial.println();
}
```

Python bridge then filters with:
```python
if line.startswith('A:'):
    angles = line[2:].split(',')
```
This is more robust than relying on `ValueError` to skip debug text.

---

## Summary Table

| Method | Transport | Latency | Complexity | SpiderBot fit |
|---|---|---|---|---|
| IK Telemetry (current) | USB Serial + ZMQ | ~15 ms | Low | ✅ Perfect |
| FK Replay | File replay | offline | Low | 🟡 Useful for gait review |
| ROS 2 bridge | Ethernet/WiFi | ~10 ms | High | 🔴 Overkill |
| MoCap retarget | Optical/IMU | ~5 ms | Very High | 🔴 Research-only |
| Vision pose-est. | Camera | ~50 ms | High | 🔴 Inaccurate for joints |
| Full digital twin | Bidirectional | <1 ms | Very High | 🔴 Not needed at this stage |
