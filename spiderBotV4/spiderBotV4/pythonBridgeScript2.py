import serial
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math

print("1. Connecting to CoppeliaSim ZMQ API...")
try:
    client = RemoteAPIClient()
    sim = client.require('sim')
    print("   -> Success! Found CoppeliaSim.")
except Exception as e:
    print(f"   -> FAILED to connect to CoppeliaSim: {e}")
    print("   -> Make sure CoppeliaSim is open and the ZMQ Remote API plugin is loaded!")
    exit(1)

print("\n2. Finding Joint Handles in Simulator...")
joint_paths = [
    # ── MAPPING ESP32 TO COPPELIASIM ──
    # CoppeliaSim numbers legs anticlockwise: FL(0), ML(1), RL(2), RR(3), MR(4), FR(5)
    # ESP32 sends them linearly: FL(0), ML(1), RL(2), FR(3), MR(4), RR(5)

    # Leg 0: Front-Left (CoppeliaSim Leg 0)
    '/hexapod/joint1_[0]',
    '/hexapod/joint1_[0]/link1Respondable_/joint2_',
    '/hexapod/joint1_[0]/link1Respondable_/joint2_/link2Respondable_/joint3_',
    
    # Leg 1: Mid-Left (CoppeliaSim Leg 1)
    '/hexapod/joint1_[1]',
    '/hexapod/joint1_[1]/link1Respondable_/joint2_',
    '/hexapod/joint1_[1]/link1Respondable_/joint2_/link2Respondable_/joint3_',
    
    # Leg 2: Rear-Left (CoppeliaSim Leg 2)
    '/hexapod/joint1_[2]',
    '/hexapod/joint1_[2]/link1Respondable_/joint2_',
    '/hexapod/joint1_[2]/link1Respondable_/joint2_/link2Respondable_/joint3_',
    
    # Leg 3: Front-Right (CoppeliaSim Leg 5)
    '/hexapod/joint1_[5]',
    '/hexapod/joint1_[5]/link1Respondable_/joint2_',
    '/hexapod/joint1_[5]/link1Respondable_/joint2_/link2Respondable_/joint3_',
    
    # Leg 4: Mid-Right (CoppeliaSim Leg 4)
    '/hexapod/joint1_[4]',
    '/hexapod/joint1_[4]/link1Respondable_/joint2_',
    '/hexapod/joint1_[4]/link1Respondable_/joint2_/link2Respondable_/joint3_',
    
    # Leg 5: Rear-Right (CoppeliaSim Leg 3)
    '/hexapod/joint1_[3]',
    '/hexapod/joint1_[3]/link1Respondable_/joint2_',
    '/hexapod/joint1_[3]/link1Respondable_/joint2_/link2Respondable_/joint3_'
]

try:
    handles = [sim.getObject(path) for path in joint_paths]
    print("   -> Success! Found all 18 joints.")
except Exception as e:
    print(f"   -> FAILED to find joints: {e}")
    print("   -> Make sure your CoppeliaSim scene hierarchy exactly matches the names in the script!")
    exit(1)

com_port = 'COM3'
print(f"\n3. Connecting to ESP32 on {com_port}...")
try:
    # Adding a timeout is CRITICAL! Otherwise readline() hangs forever if no newline is received
    ser = serial.Serial(com_port, 115200, timeout=1) 
    print(f"   -> Success! Connected to {com_port}.")
except serial.SerialException as e:
    print(f"   -> FAILED to open {com_port}. Is it open in the Arduino IDE?")
    exit(1)

DEG_TO_RAD = math.pi / 180.0
print("\n--- ALL SYSTEMS GO! Listening for angles... (Press Ctrl+C to stop) ---")

# Tell CoppeliaSim that the Python bridge is active
sim.setInt32Signal('python_connected', 1)

# Tibia offset (degrees): applied on top of tDeg when mapping to simulator.
# Adjust this if the tibia is bent forward/back at standstill.
t_offset = 0.0

# rest_angles: captured from the very first frame (bot at standstill).
# Used as a reference so the home pose is always correct and gait direction is right.
rest_angles = None
sim_rest = []

packet_count = 0
last_print = time.time()

# ── COXA CORRECTIONS for rectangular → hexagonal body mapping ────────────
#
# The physical bot's walkY values (3, -1, -4, 0, 2, 6) were tuned for a
# RECTANGULAR body shape. They create coxa angle biases via atan2(walkY, restX).
# CoppeliaSim's model is a HEXAGON (legs at exactly 60° intervals), so these
# rectangular biases cause the virtual robot to not walk straight.
# We subtract them here to re-center each leg's stride.
#
# Formula: correction = -atan2(walkY, restX) in degrees
# From Arduino code (legs array):  restX=10 for all legs
#   Leg 0 FL: walkY= 3  → -atan2( 3, 10)*57.3 = -16.7°
#   Leg 1 ML: walkY=-1  → -atan2(-1, 10)*57.3 =  +5.7°
#   Leg 2 RL: walkY=-4  → -atan2(-4, 10)*57.3 = +21.8°
#   Leg 3 FR: walkY= 0  → -atan2( 0, 10)*57.3 =   0.0°
#   Leg 4 MR: walkY= 2  → -atan2( 2, 10)*57.3 = -11.3°
#   Leg 5 RR: walkY= 6  → -atan2( 6, 10)*57.3 = -31.0°
COXA_CORRECTION = [-16.7, +5.7, +21.8, 0.0, -11.3, -31.0]  # degrees


# Smoothing/Interpolation State
target_sim_angles = None
current_sim_angles = None
EMA_ALPHA = 0.25  # Lower = smoother (but slightly more lag). 0.25 is highly responsive but strips jitter.

try:
    while True:
        # ── 1. DRAIN THE USB BUFFER ──────────────────────────────────────────
        # Windows USB buffers serial data into "chunks". If we process every line
        # sequentially, they bottleneck and ZMQ sends a rapid-fire burst of 
        # targets to CoppeliaSim, which causes the PID controllers to violently snap.
        # Fix: Read ALL available lines, but ONLY parse the absolute newest one.
        if ser.in_waiting > 0:
            lines = ser.readlines() # Drains the entire buffer instantly
            if len(lines) > 0:
                line = lines[-1].decode('utf-8', errors='ignore').strip()
                angles = line.split(',')
                
                if len(angles) >= 18:
                    try:
                        # ── FIRST FRAME CALIBRATION ──
                        if rest_angles is None:
                            rest_angles = [float(a) for a in angles[:18]]
                            print(f"   -> Rest pose calibrated! f_rest[0]={rest_angles[1]:.1f}°")

                        parsed_targets = [0.0] * 18
                        for leg in range(6):
                            c_idx, f_idx, t_idx = leg*3, leg*3+1, leg*3+2
                            
                            c_deg = float(angles[c_idx])
                            f_deg = float(angles[f_idx])
                            t_deg = float(angles[t_idx])

                            f_r = rest_angles[f_idx]
                            t_r = rest_angles[t_idx]

                            parsed_targets[c_idx] = c_deg
                            parsed_targets[f_idx] = f_deg - 2.0 * f_r
                            parsed_targets[t_idx] = 2.0 * (t_r + t_offset) - (t_deg + t_offset)

                        target_sim_angles = parsed_targets
                        packet_count += 1
                        
                        if time.time() - last_print > 1.0:
                            print(f"Streaming... ({packet_count} packets decoded. f0={float(angles[1]):.1f}°)")
                            last_print = time.time()
                    except ValueError:
                        pass # Ignore text lines

        # ── 2. HIGH-FREQUENCY INTERPOLATION (EMA) ─────────────────────────────
        # Instead of slamming the Physics engine with discrete jumps, we ease 
        # the targets toward the goal at a decoupled, high-frequency pace.
        if target_sim_angles is not None:
            if current_sim_angles is None:
                current_sim_angles = list(target_sim_angles)
            
            for i in range(18):
                # Exponential Moving Average cleanly interpolates dropped/clumped frames
                current_sim_angles[i] += EMA_ALPHA * (target_sim_angles[i] - current_sim_angles[i])
                
                # MUST use setJointTargetPosition for dynamic physics models!
                sim.setJointTargetPosition(handles[i], current_sim_angles[i] * DEG_TO_RAD)
                
        # Run this interpolation loop continuously at ~100Hz 
        # (much faster & smoother than the 50Hz discrete serial baud)
        time.sleep(0.01)
            
except KeyboardInterrupt:
    print("\nStopped by User")
finally:
    try:
        sim.clearInt32Signal('python_connected')
    except:
        pass
    if 'ser' in locals() and ser.is_open:
        ser.close()