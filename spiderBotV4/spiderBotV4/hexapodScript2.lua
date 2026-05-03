sim=require'sim'
simIK=require'simIK'

-- =========================================================================
-- SPIDERBOT V3.5 - SMART DUAL-MODE IK ENGINE
-- =========================================================================
-- This script smartly transitions between internal IK and external control.
--
-- 1. If 'pythonBridgeScript.py' connects (ESP32 Live Telemetry):
--    -> The bridge broadcasts 'python_connected' = 1.
--    -> This script mutes the internal IK engine so it doesn't fight the 
--       direct joint positions being streamed from the ESP32.
--
-- 2. If the bridge is NOT connected:
--    -> The script runs the internal CoppeliaSim IK solver dynamically.
--    -> This allows the standalone 'smoothGaitController.py' to move the  
--       invisible footTarget dummies and have the virtual legs track them!
-- =========================================================================

function sysCall_init()
    local legBase = sim.getObject('../legBase')
    
    local simLegTips = {}
    local simLegTargets = {}
    for i=1,6,1 do
        simLegTips[i] = sim.getObject('../footTip'..i-1)
        simLegTargets[i] = sim.getObject('../footTarget'..i-1)
    end
    
    -- Initialize Inverse Kinematics Environment
    local simBase = sim.getObject('..')
    ikEnv = simIK.createEnvironment()
    ikGroup = simIK.createGroup(ikEnv)
    
    for i=1,#simLegTips,1 do
        simIK.addElementFromScene(ikEnv, ikGroup, simBase, simLegTips[i], simLegTargets[i], simIK.constraint_position)
    end
    
    lastConnectedState = false
    sim.addLog(sim.verbosity_scriptinfos, "Internal IK Enabled. Awaiting Python controller...")
end

function sysCall_actuation()
    -- Check if pythonBridgeScript.py has pinged us
    local isConnected = sim.getInt32Signal('python_connected') == 1
    
    if isConnected and not lastConnectedState then
        sim.addLog(sim.verbosity_scriptinfos, "=====================================================")
        sim.addLog(sim.verbosity_scriptinfos, ">>> ESP32 PYTHON BRIDGE DETECTED!                 <<<")
        sim.addLog(sim.verbosity_scriptinfos, ">>> Internal IK Solver Disabled.                  <<<")
        sim.addLog(sim.verbosity_scriptinfos, "=====================================================")
        lastConnectedState = true
    elseif not isConnected and lastConnectedState then
        sim.addLog(sim.verbosity_scriptinfos, "=====================================================")
        sim.addLog(sim.verbosity_scriptinfos, ">>> PYTHON BRIDGE DISCONNECTED.                   <<<")
        sim.addLog(sim.verbosity_scriptinfos, ">>> Internal IK Solver Re-Enabled.                <<<")
        sim.addLog(sim.verbosity_scriptinfos, "=====================================================")
        lastConnectedState = false
    end
    
    -- If the ESP32 is NOT driving the joints, run the CoppeliaSim IK engine.
    -- This handles the kinematics for 'smoothGaitController.py'.
    if not isConnected then
        simIK.handleGroup(ikEnv, ikGroup, {syncWorlds=true, allowError=true})
    end
end

function sysCall_sensing()
    -- Empty
end

function sysCall_cleanup()
    simIK.eraseEnvironment(ikEnv)
end
