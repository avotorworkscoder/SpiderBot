-- =========================================================================
-- SPIDERBOT V3.4 - DEFAULT SCRIPT DISABLED
-- =========================================================================
-- This script has been disabled because the robot's 18 joint angles 
-- are now being controlled in real-time by the ESP32 via the external 
-- pythonBridgeScript.py (ZeroMQ client).
--
-- If this script's IK engine was left running, it would aggressively 
-- fight the Python script over the joint positions, causing the robot 
-- to glitch violently in the simulation.
-- =========================================================================

function sysCall_init()
    -- We do NOT initialize the default CoppeliaSim IK engine.
    -- The external Python ZMQ client will handle joint positioning!
    sim.addLog(sim.verbosity_scriptinfos, "Default Hexapod Lua Script DISABLED. Waiting for Python Bridge to connect...")
    lastConnectedState = false
end

function sysCall_actuation()
    -- Check if pythonBridgeScript.py has pinged us
    local isConnected = sim.getInt32Signal('python_connected') == 1
    
    if isConnected and not lastConnectedState then
        sim.addLog(sim.verbosity_scriptinfos, "=====================================================")
        sim.addLog(sim.verbosity_scriptinfos, ">>> PYTHON BRIDGE CONNECTED! ESP32 CONTROL ACTIVE <<<")
        sim.addLog(sim.verbosity_scriptinfos, "=====================================================")
        lastConnectedState = true
    elseif not isConnected and lastConnectedState then
        sim.addLog(sim.verbosity_scriptinfos, "=====================================================")
        sim.addLog(sim.verbosity_scriptinfos, ">>> PYTHON BRIDGE DISCONNECTED. WAITING...        <<<")
        sim.addLog(sim.verbosity_scriptinfos, "=====================================================")
        lastConnectedState = false
    end
end

function sysCall_sensing()
    -- Empty
end

function sysCall_cleanup()
    -- Empty
end

