"""
SpiderBot V3.4 — Smooth Gait Controller (Python)
=================================================
Direct port of originalHexapodScript.lua to Python.

This is a STANDALONE autonomous controller for the CoppeliaSim virtual robot.
It does NOT read from the ESP32 — run this when you want the virtual robot
to demonstrate the smooth gait algorithm on its own.

HOW TO USE:
  1. Open the CoppeliaSim hexapod scene and press Play (▶)
  2. Run:  python smoothGaitController.py
  3. The virtual robot will execute the full demo sequence

HOW IT DIFFERS FROM pythonBridgeScript.py:
  - pythonBridgeScript.py: reads raw joint angles from ESP32, mirrors physical robot
  - smoothGaitController.py: computes foot positions autonomously using smooth
    phase-based algorithm; CoppeliaSim's IK engine handles joint angles internally
"""

import time
import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# ═══════════════════════════════════════════════════════════════════════════════
#  1. CONNECT
# ═══════════════════════════════════════════════════════════════════════════════

print("1. Connecting to CoppeliaSim ZMQ API...")
try:
    client = RemoteAPIClient()
    sim    = client.require('sim')
    print("   -> Connected!")
except Exception as e:
    print(f"   -> FAILED: {e}")
    print("   -> Make sure CoppeliaSim is open and running (press Play first)!")
    exit(1)

# ═══════════════════════════════════════════════════════════════════════════════
#  2. GET SCENE HANDLES
#     Adjust these paths to match YOUR scene's hierarchy.
#     Look in the Scene Hierarchy panel in CoppeliaSim.
# ═══════════════════════════════════════════════════════════════════════════════

print("2. Finding scene objects...")
try:
    # The reference frame of the leg assembly (same as legBase in Lua)
    leg_base = sim.getObject('/hexapod/body')

    # footTip = the actual physical tip of each leg (READ ONLY — to get rest pos)
    # footTarget = invisible IK dummy targets (we WRITE these to drive the legs)
    # Both are numbered 0-5 (matching Lua's i-1 offset)
    foot_tips    = [sim.getObject(f'/hexapod/footTip{i}')    for i in range(6)]
    foot_targets = [sim.getObject(f'/hexapod/footTarget{i}') for i in range(6)]

    # Record the natural rest position of each foot in the legBase frame
    # This mirrors:  initialPos[i] = sim.getObjectPosition(simLegTips[i], legBase)
    initial_pos = [sim.getObjectPosition(tip, leg_base) for tip in foot_tips]
    print(f"   -> Found all objects. FL rest pos = {[f'{v:.3f}' for v in initial_pos[0]]}")

except Exception as e:
    print(f"   -> FAILED to find objects: {e}")
    print("   -> Check that footTip0..5 and footTarget0..5 exist in your scene!")
    exit(1)

# ═══════════════════════════════════════════════════════════════════════════════
#  3. GAIT STATE
#     Mirrors the global variables initialized in sysCall_init()
# ═══════════════════════════════════════════════════════════════════════════════

# legMovementIndex = {1,4,2,6,3,5} from the Lua script.
# Each entry is the 1-based "slot" this leg occupies in the ripple sequence.
# The phase offset for leg i = (legMovementIndex[i] - 1) / 6.
LEG_MOVEMENT_INDEX = [1, 4, 2, 6, 3, 5]

step_progression       = 0.0   # Advances continuously; drives all leg phases
real_movement_strength = 0.0   # Smoothly converges toward mov['strength']

# Active movement parameters — change via set_step_mode()
mov = {
    'vel':       0.9,    # Phase advancement speed (1.0 = full speed gait)
    'amplitude': 0.11,   # Stride half-length in metres (11 cm)
    'height':    0.02,   # Foot lift in metres (2 cm)
    'dir':       0.0,    # Walk direction in radians (0 = forward)
    'rot':       0.0,    # Rotation: 0 = straight, ±1 = spin in place
    'strength':  0.0,    # 0 = stopped, 1 = full movement
}

# ═══════════════════════════════════════════════════════════════════════════════
#  4. set_step_mode()
#     Mirrors setStepMode() in Lua — call this any time to change the motion.
# ═══════════════════════════════════════════════════════════════════════════════

def set_step_mode(vel:float, amplitude:float, height:float,
                  direction_deg:float, rotation:float, strength:float):
    """
    Args:
        vel           : gait speed (0.5–1.5 is typical)
        amplitude     : stride half-length in metres (e.g. 0.11)
        height        : foot lift height in metres (e.g. 0.02)
        direction_deg : walk bearing in degrees (0=forward, 90=left, 180=back)
        rotation      : 0=straight  +1=spin anticlockwise  -1=spin clockwise
        strength      : 0=stop  1=full speed (eases in/out automatically)
    """
    mov['vel']       = vel
    mov['amplitude'] = amplitude
    mov['height']    = height
    mov['dir']       = math.radians(direction_deg)
    mov['rot']       = rotation
    mov['strength']  = strength

# ═══════════════════════════════════════════════════════════════════════════════
#  5. gait_step(dt)
#     The heart of the algorithm. Mirrors sysCall_actuation() in Lua.
#     Call this every ~20ms (50 Hz).
# ═══════════════════════════════════════════════════════════════════════════════

def gait_step(dt: float):
    """
    Advance the gait by dt seconds and update all 6 foot target positions.
    CoppeliaSim's IK engine will automatically compute joint angles to reach them.
    """
    global step_progression, real_movement_strength

    # ── Smooth the movement strength (ease-in / ease-out) ────────────────────
    # When strength changes (e.g. 0→1 on start or 1→0 on stop), the robot
    # accelerates/decelerates gracefully instead of snapping.
    dx = mov['strength'] - real_movement_strength
    if abs(dx) > dt * 0.1:              # If error is large, cap the correction
        dx = abs(dx) * dt * 0.5 / dx   # proportional step capped at dt*0.5
    real_movement_strength += dx

    # ── Compute foot position for each leg ───────────────────────────────────
    for leg in range(6):

        # Phase for this leg: staggered evenly so neighbouring legs are 1/6 cycle apart
        sp = (step_progression + (LEG_MOVEMENT_INDEX[leg] - 1) / 6.0) % 1.0

        # ── The 4-segment foot path ───────────────────────────────────────────
        #
        #  Phase    Segment          What happens
        #  0→1/3    DRAG 1           Foot on ground, slides backward
        #  1/3→1/2  SWING RISE       Foot lifts off, arcs upward and forward
        #  1/2→2/3  SWING FALL       Foot descends from apex to landing zone
        #  2/3→1    DRAG 2           Foot on ground again, continues backward
        #
        #  off_stride : displacement along the walk direction (horizontal)
        #  off_lift   : vertical displacement (positive = up)
        #
        off_stride = 0.0
        off_lift   = 0.0

        if sp < (1.0/3.0):
            # DRAG 1: foot on ground → slides from 0 to +amplitude/2
            off_stride = sp * 3 * mov['amplitude'] / 2

        elif sp < (1.0/3.0 + 1.0/6.0):
            # SWING RISE: foot leaves ground → moves from +amp/2 to 0, lifts to apex
            s          = sp - 1.0/3.0
            off_stride = mov['amplitude'] / 2 - mov['amplitude'] * s * 6 / 2
            off_lift   = s * 6 * mov['height']

        elif sp < (2.0/3.0):
            # SWING FALL: foot descends from apex → moves from 0 to -amp/2, touches down
            s          = sp - 1.0/3.0 - 1.0/6.0
            off_stride = -mov['amplitude'] * s * 6 / 2
            off_lift   = (1 - s * 6) * mov['height']

        else:
            # DRAG 2: foot back on ground → slides from -amp/2 back to 0
            s          = sp - 2.0/3.0
            off_stride = -mov['amplitude'] * (1 - s * 3) / 2

        # ── Apply direction and rotation ──────────────────────────────────────
        # md = the actual bearing of THIS leg's stride (modified for rotation mode).
        # For straight walk: md == mov['dir'] for all legs.
        # For rotation: each leg gets a tangential direction around the body centre,
        #               computed from its initial foot position.
        ix = initial_pos[leg][0]
        iy = initial_pos[leg][1]
        md = mov['dir'] + abs(mov['rot']) * math.atan2(
            ix * mov['rot'], -iy * mov['rot'])

        # Scale by actual movement strength and decompose into world XY + Z
        off2_x = off_stride * math.cos(md) * real_movement_strength
        off2_y = off_stride * math.sin(md) * real_movement_strength
        off2_z = off_lift   * real_movement_strength

        # Final foot position = initial rest position + computed offset
        p = [
            initial_pos[leg][0] + off2_x,
            initial_pos[leg][1] + off2_y,
            initial_pos[leg][2] + off2_z,
        ]
        # Place the IK target dummy at position p (in legBase frame).
        # CoppeliaSim's IK solver will bend the leg to reach this point.
        sim.setObjectPosition(foot_targets[leg], p, leg_base)

    # ── Advance the global phase ──────────────────────────────────────────────
    step_progression = (step_progression + dt * mov['vel']) % 10.0

# ═══════════════════════════════════════════════════════════════════════════════
#  6. HELPER — run_for(seconds)
#     Keeps the gait ticking for a fixed duration.
# ═══════════════════════════════════════════════════════════════════════════════

def run_for(seconds: float):
    """Tick the gait loop for a fixed duration."""
    t_start = time.time()
    t_last  = t_start
    while time.time() - t_start < seconds:
        now = time.time()
        dt  = min(now - t_last, 0.1)   # Clamp dt against pauses
        t_last = now
        gait_step(dt)
        time.sleep(0.02)   # 50 Hz update rate

# ═══════════════════════════════════════════════════════════════════════════════
#  7. DEMO SEQUENCE
#     Mirrors sysCall_thread() in the Lua script.
# ═══════════════════════════════════════════════════════════════════════════════

STEP_H   = 0.02    # 2 cm lift height
MAX_STEP = 0.11    # 11 cm stride amplitude
WALK_VEL = 0.9     # Gait speed

print("\n--- Starting demo sequence (mirrors Lua sysCall_thread) ---")
print("    Press Ctrl+C at any time to stop.\n")

try:
    # ── Phase 1: Walk forward for 12 seconds ─────────────────────────────────
    print(">>> Walking forward (12 s)...")
    set_step_mode(WALK_VEL, MAX_STEP, STEP_H, 0, 0, 1)
    run_for(12)

    # ── Phase 2: Walk while slowly turning through 270° ──────────────────────
    print(">>> Turning while walking (27 × 0.5 s steps)...")
    for i in range(1, 28):
        set_step_mode(WALK_VEL, MAX_STEP, STEP_H, 10 * i, 0, 1)
        run_for(0.5)

    # ── Phase 3: Stop ─────────────────────────────────────────────────────────
    print(">>> Stopping (2 s)...")
    set_step_mode(WALK_VEL, MAX_STEP, STEP_H, 270, 0, 0)
    run_for(2)

    # ── Phase 4: Walk forward at half stride ──────────────────────────────────
    print(">>> Walking forward at half stride (12 s)...")
    set_step_mode(WALK_VEL, MAX_STEP * 0.5, STEP_H, 0, 0, 1)
    run_for(12)

    # ── Phase 5: Stop again ───────────────────────────────────────────────────
    print(">>> Stopping (2 s)...")
    set_step_mode(WALK_VEL, MAX_STEP * 0.5, STEP_H, 0, 0, 0)
    run_for(2)

    # ── Phase 6: Rotate on the spot ───────────────────────────────────────────
    print(">>> Rotating on spot (24 s)...")
    set_step_mode(WALK_VEL, MAX_STEP * 0.5, STEP_H, 0, 1, 1)
    run_for(24)

    # ── Phase 7: Final stop ───────────────────────────────────────────────────
    print(">>> Sequence complete. Stopping.")
    set_step_mode(WALK_VEL, MAX_STEP * 0.5, STEP_H, 0, 0, 0)
    run_for(2)

except KeyboardInterrupt:
    print("\n\nStopped by user.")
