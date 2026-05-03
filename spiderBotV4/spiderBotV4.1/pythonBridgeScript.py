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


try:
    while True:
        if ser.in_waiting > 0:
            # errors='ignore' prevents crashes from corrupted serial bytes
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            angles = line.split(',')
            
            if len(angles) >= 18:
                try:
                    # ─── FIRST FRAME: capture standstill as reference ─────────────────
                    # The standstill pose is the "zero" for the simulator.
                    # We record the rest angles on the first frame and use them
                    # so that the HOME pose is always visually correct,
                    # AND the delta direction matches CoppeliaSim's joint axes.
                    if rest_angles is None:
                        rest_angles = [float(a) for a in angles[:18]]
                        # Precompute the sim rest angles using the same formula as standstill.
                        # At standstill: sim_f = -f_deg  →  f_rest_sim = -f_rest
                        #                sim_t = t_deg + t_offset
                        sim_rest = []
                        for leg in range(6):
                            c_r = rest_angles[leg*3]
                            f_r = rest_angles[leg*3+1]
                            t_r = rest_angles[leg*3+2]
                            sim_rest.extend([c_r, -f_r, t_r + t_offset])
                        print(f"   -> Rest pose calibrated! f_rest[0]={rest_angles[1]:.1f}°")

                    for leg in range(6):
                        c_idx, f_idx, t_idx = leg*3, leg*3+1, leg*3+2

                        c_deg = float(angles[c_idx])
                        f_deg = float(angles[f_idx])
                        t_deg = float(angles[t_idx])

                        # Rest values captured at standstill (first frame)
                        f_r = rest_angles[f_idx]
                        t_r = rest_angles[t_idx]

                        # Coxa: same direction works for rotation in the XY plane
                        sim_c = c_deg

                        # Femur: Z convention is inverted (ESP32 Z-down vs CoppeliaSim Z-up)
                        # At rest:  sim_f = -f_r             (gives correct standstill angle)
                        # In motion: apply delta OPPOSITE to the convention flip
                        #    sim_f = f_rest_sim + (-(f_deg - f_r))
                        #          = -f_r - (f_deg - f_r)
                        #          = -f_r - f_deg + f_r
                        #          = f_deg - 2 * f_r    ← this flips the MOTION direction
                        #                                  while keeping the rest pose correct
                        sim_f = f_deg - 2.0 * f_r

                        # Tibia: same delta flip approach
                        sim_t = 2.0 * (t_r + t_offset) - (t_deg + t_offset)

                        sim.setJointTargetPosition(handles[c_idx], sim_c * DEG_TO_RAD)
                        sim.setJointTargetPosition(handles[f_idx], sim_f * DEG_TO_RAD)
                        sim.setJointTargetPosition(handles[t_idx], sim_t * DEG_TO_RAD)

                    packet_count += 1
                    if time.time() - last_print > 1.0:
                        print(f"Streaming... ({packet_count} packets. f0={float(angles[1]):.1f}°)")
                        last_print = time.time()
                except ValueError:
                    pass  # Ignore debug text lines like '>> Forward: Tripod Walk'
        else:
            # Small sleep to prevent 100% CPU usage
            time.sleep(0.001)
            
except KeyboardInterrupt:
    print("\nStopped by User")
finally:
    try:
        sim.clearInt32Signal('python_connected')
    except:
        pass
    if 'ser' in locals() and ser.is_open:
        ser.close()