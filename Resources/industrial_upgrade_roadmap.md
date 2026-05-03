# SpiderBot — Industrial-Grade Terrain Navigation Roadmap

## Where We Are Now (V2.8)

```
┌─────────────────────────────────────────────────┐
│  SpiderBot V2.8 — Current Capabilities          │
├─────────────────────────────────────────────────┤
│  ✅ 4 gaits (tripod, crab, diagonal, wave)      │
│  ✅ Omnidirectional movement (2D crab/diagonal)  │
│  ✅ Joystick + button control via Bluetooth      │
│  ✅ True wave gait (1-leg-at-a-time)            │
│  ✅ IK solver for foot positioning               │
│  ✅ Runtime height adjustment                    │
│  ❌ No terrain sensing                           │
│  ❌ No body stabilization                        │
│  ❌ No obstacle detection                        │
│  ❌ Open-loop control (no feedback)              │
│  ❌ No autonomous navigation                     │
└─────────────────────────────────────────────────┘
```

---

## 1. Sensor Suite 🔬

The biggest gap between hobby and industrial is **sensors**. Without sensors, the bot is blind.

### 1.1 IMU (Inertial Measurement Unit) ⭐⭐⭐

**What:** MPU6050 or BNO055 — measures body tilt (pitch, roll, yaw)

**Why critical:**
- Detect slopes and auto-adjust leg positions
- Detect if the bot is tipping over
- Enable body stabilization (keep body level on uneven ground)
- Fall detection and recovery

**Impact:** Unlocks body stabilization, slope navigation, and tilt-aware gaits.

```
         Flat ground:          Slope detected (IMU):
         ┌─────────┐          ┌─────────┐ ← body stays level
         │  BODY   │          │  BODY   │
        ╱│╲       ╱│╲        ╱│╲     ╱╱│╲╲  ← uphill legs shorter,
       ╱ │ ╲     ╱ │ ╲      ╱ │ ╲   ╱╱ │ ╲╲    downhill legs longer
      ───┴───   ───┴───    ───┴──────┴───
      FLAT GROUND          SLOPE (20°)
```

**Hardware:** MPU6050 (~$2), I2C connection to ESP32

---

### 1.2 Foot Contact / Pressure Sensors ⭐⭐⭐

**What:** Micro switches or FSR (Force Sensitive Resistors) on each foot tip

**Why critical:**
- Know when a foot has actually touched ground
- Detect holes, edges, and voids
- Enable **adaptive stepping** — if foot doesn't touch, extend further
- Measure weight distribution for stability

```
Normal step:           Hole detected:
  Leg descends ──▶     Leg descends ──▶  No contact!
  ✓ Contact at Z=-9    ✗ Still no contact at Z=-12
  → Plant foot          → RETRACT, try a different position
```

**Hardware:** 6× micro switches (~$3 total) or FSR402 sensors

---

### 1.3 Current Sensing (Per Servo) ⭐⭐

**What:** INA219 or ACS712 to measure servo current draw

**Why:**
- High current = servo is straining = obstacle or stuck
- Detect collisions without physical contact sensors
- Prevent servo burnout
- Estimate terrain resistance (soft sand vs hard rock)

**Hardware:** INA219 modules on servo power lines

---

### 1.4 Distance / Obstacle Sensors ⭐⭐

**What:** Front-facing ultrasonic (HC-SR04) or ToF sensor (VL53L0X), or even a micro LIDAR (RPLidar A1)

**Why:**
- Detect obstacles before contact
- Map terrain ahead
- Enable autonomous obstacle avoidance

**Hardware options:**

| Sensor | Range | Accuracy | Cost |
|--------|-------|----------|------|
| HC-SR04 ultrasonic | 2–400cm | ±3mm | $1 |
| VL53L0X ToF | 3–200cm | ±3% | $5 |
| VL53L1X ToF | 4–400cm | ±3% | $10 |
| RPLidar A1 | 0.15–12m | ±1% | $100 |

---

## 2. Body Stabilization 🏔️

### 2.1 Auto-Leveling (IMU-Based)

**Concept:** Read pitch/roll from IMU, adjust all 6 leg Z heights to keep the body level.

```cpp
void autoLevel() {
    float pitch = imu.getPitch();  // degrees
    float roll  = imu.getRoll();

    for (int i = 0; i < 6; i++) {
        Leg& leg = legs[i];
        // Compute height compensation based on leg position and tilt
        float dZ_pitch = sin(pitch * DEG_TO_RAD) * leg.walkY;
        float dZ_roll  = sin(roll  * DEG_TO_RAD) * leg.restX * leg.coxaDir;
        leg.restZ = bodyZ + dZ_pitch + dZ_roll;
    }
}
```

- Call `autoLevel()` in [loop()](file:///f:/Arduino/Git/SpiderBot/spiderBotV2/spiderBotV2.7/spiderBotV2.7.ino#681-701) at ~50Hz
- Body stays level even on a 30° slope
- Combine with wave gait for maximum stability on terrain

### 2.2 Active Compliance

**Concept:** Instead of rigid leg positions, legs act like springs — if external force pushes a leg, it yields smoothly instead of fighting.

- Requires current sensing or encoder feedback
- Implement as a PD controller per leg
- Makes the bot resilient to being bumped or stepping on unstable ground

---

## 3. Terrain-Adaptive Gaits 🌍

### 3.1 Surface Classification

Use IMU vibration patterns + servo current to classify terrain:

| Surface | IMU Signature | Current Signature |
|---------|--------------|-------------------|
| Hard floor | Low vibration | Low current |
| Gravel | Medium vibration | Medium current |
| Sand | Low vibration | High current (sinking) |
| Grass | High vibration | Medium current |

Auto-select appropriate gait:
- Hard floor → Tripod (fast)
- Gravel → Tetrapod (stable)
- Sand → Wave + high step (prevent sinking)
- Steep slope → Body-shift walk

### 3.2 Reactive Stepping

When foot sensor detects no contact (hole/edge):

```
State machine per leg:
  NORMAL → foot contact lost?
    YES → SEARCH (extend leg further, try ±2cm in X/Y)
      Found contact? → PLANT
      Max extension reached? → RETRACT, flag obstacle
```

### 3.3 Gait Transition Smoothing

Currently switching gaits is instant. Add smooth transitions:
- When switching g0→g3, gradually change phase timing over 2-3 cycles
- Prevents sudden jerky movements that could destabilize on terrain

---

## 4. Mechanical Improvements 🔧

### 4.1 Compliant Foot Tips

**Current:** Hard plastic/metal foot tips — slip on smooth surfaces, no shock absorption

**Upgrade:** Silicone or rubber foot tips
- Better grip on all surfaces
- Shock absorption when stepping down
- Cheap and easy to add (~$5)

### 4.2 Sealed / Weather-Resistant Design

For outdoor industrial use:
- Conformal coating on PCBs
- Silicone-sealed servo connections
- IP54 or higher enclosure for electronics
- Cable management (prevent snagging on terrain)

### 4.3 Stronger Servos

**Current:** Standard hobby servos — limited torque, no feedback

**Upgrade options:**

| Servo Type | Torque | Feedback | Cost/ea |
|------------|--------|----------|---------|
| Current (SG90/MG996) | 2-13 kg·cm | None | $3-8 |
| Smart serial servos (LX-16A) | 17 kg·cm | Position + load + temp | $12 |
| Dynamixel AX-12A | 16 kg·cm | Full telemetry | $45 |
| Dynamixel XL430 | 8.4 N·m | Full telemetry | $50 |

> [!TIP]
> Smart serial servos with feedback are the single biggest upgrade for industrial use. They give you position verification, load sensing, and temperature monitoring — all essential for reliable terrain navigation.

---

## 5. Power & Endurance 🔋

### 5.1 Better Battery System

- Use 3S or 4S LiPo with proper BMS
- Add voltage monitoring with auto-shutoff
- Estimated runtime improvement: 2-3x with proper power management

### 5.2 Power-Efficient Gaits

- Wave gait uses less power (only 1 servo moving at a time)
- Add a "low power mode" that reduces stride length and speed
- Sleep servos when idle (your LSC-32 may support this)

### 5.3 Power Budget Awareness

```cpp
// Estimate power consumption per gait:
// Tripod: 6 servos moving simultaneously → high peak current
// Wave:   ~2 servos moving at a time → low peak current
// Auto-switch to wave when battery < 30%
if (batteryPercent < 30 && activeGait != 3) {
    activeGait = 3;  // switch to wave for efficiency
    btLog(">> Low battery: switched to wave gait\n");
}
```

---

## 6. Autonomous Navigation 🤖

### 6.1 Waypoint Following

- GPS module (NEO-6M, ~$5) for outdoor waypoints
- Navigate to coordinates autonomously
- Combine with obstacle avoidance for path planning

### 6.2 SLAM (Simultaneous Localization and Mapping)

- Requires: RPLidar or camera + ROS2
- Maps the environment while navigating
- Industrial standard for autonomous mobile robots
- Significant upgrade — likely ESP32→Raspberry Pi + ESP32 architecture

### 6.3 Simple Path Planning

Without full SLAM, you can implement wall-following or bug algorithms:
```
1. Move toward target
2. If obstacle detected → follow wall until clear path
3. Resume toward target
```

---

## 7. Communication Upgrades 📡

### 7.1 WiFi Control (ESP32 Native)

- ESP32 already has WiFi — use it for web-based control
- Create a web dashboard showing IMU data, battery, gait status
- Longer range than Bluetooth

### 7.2 Telemetry Logging

- Log sensor data to SD card or stream via WiFi
- Replay sessions for debugging gait behavior
- Essential for iterating on terrain algorithms

### 7.3 ROS2 Integration

- Bridge ESP32 to ROS2 via micro-ROS
- Enables using the full ROS ecosystem (nav2, SLAM, visualization)
- Industrial standard for multi-robot systems

---

## 8. Software Architecture 🏗️

### 8.1 RTOS (FreeRTOS)

ESP32 already runs FreeRTOS. Use it properly:

```
Task 1: Gait Engine     (core 1, 50Hz)   — servo commands
Task 2: Sensor Fusion   (core 0, 100Hz)  — IMU, foot sensors
Task 3: Communication   (core 0, 10Hz)   — BT/WiFi parsing
Task 4: Navigation      (core 0, 5Hz)    — path planning
```

- Prevents sensor reads from blocking gait timing
- True real-time control

### 8.2 Finite State Machine (Robot Level)

```
                    ┌──────────┐
           ┌──────▶│  IDLE    │◀──────┐
           │       └────┬─────┘       │
         fault     command          stop
           │       ┌────▼─────┐       │
           │       │ WALKING  │───────┘
           │       └────┬─────┘
           │       obstacle│
           │       ┌────▼─────┐
           │       │AVOIDING  │
           │       └────┬─────┘
           │       clear│
           │       ┌────▼─────┐
           ├───────│RECOVERING│
           │       └──────────┘
           │
      ┌────▼─────┐
      │  FAULT   │  (tipped over, servo error, low battery)
      └──────────┘
```

### 8.3 Configuration System

Move magic numbers to a config struct:
```cpp
struct Config {
    float strideLength;   // vs hard-coded 3.0
    float liftHeight;     // vs hard-coded LIFT_H
    uint16_t moveTime;
    uint8_t defaultGait;
    bool autoLevel;
    bool terrainAdapt;
};
```
- Save/load from EEPROM
- Tune over Bluetooth without re-flashing
- Different profiles for different terrain presets

---

## Implementation Roadmap

### Phase 1 — V2.9: Sensor Foundation (~2 weeks)
- [ ] Add MPU6050 IMU (I2C)
- [ ] Implement body tilt reading
- [ ] Add basic auto-leveling
- [ ] Add rubber foot tips
- [ ] Add battery percentage to status report

### Phase 2 — V3.0: Terrain Awareness (~3 weeks)
- [ ] Add foot contact switches (6x)
- [ ] Implement reactive stepping (search if no contact)
- [ ] Add tetrapod and ripple gaits
- [ ] Add high-step mode for wave gait
- [ ] Smooth gait transitions

### Phase 3 — V3.5: Smart Navigation (~4 weeks)
- [ ] Add front distance sensor (VL53L1X)
- [ ] Implement obstacle avoidance
- [ ] Add surface classification (IMU vibration)
- [ ] Auto gait selection based on terrain
- [ ] WiFi web dashboard

### Phase 4 — V4.0: Industrial Grade (~2 months)
- [ ] Upgrade to smart serial servos (position feedback)
- [ ] FreeRTOS task architecture
- [ ] Telemetry logging (SD card)
- [ ] GPS waypoint navigation
- [ ] Sealed enclosure design
- [ ] ROS2 bridge (optional)

---

## Priority Matrix

```
              HIGH IMPACT
                  ▲
                  │
    IMU + Auto    │   Smart Servos
     Level ★★★   │    ★★★
                  │
    Foot Contact  │   SLAM/ROS2
     Sensors ★★★  │    ★★
                  │
    Rubber Feet   │   GPS Navigation
       ★★        │    ★
                  │
    High-Step     │   Sealed
     Wave ★       │   Design ★
                  │
          ────────┼───────────────▶
         LOW      │            HIGH
         EFFORT   │           EFFORT
```

> [!IMPORTANT]
> The **single highest-impact upgrade** is IMU + auto-leveling. It transforms every existing gait into a terrain-capable gait by keeping the body stable on slopes and uneven ground. Everything else builds on top of this foundation.
