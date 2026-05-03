1. Session Management
        
        SESSION_SUMMARY.md:
                "We are done. Generate a file called SESSION_SUMMARY.md. List exactly what we changed today, what logic we fixed, and the exact 'Next Steps' for the next session. Include any new variables or constants we decided on."

        CONTEXT_SYNC:
                "Read the CLAUDE.md and PRIME_INSTRUCTIONS.md. Summarize the current 'Active Goal' and confirm you are ready to proceed with the specific coding standards defined in the STYLE_GUIDE.md."

2. Firmware Engineering
        
        NON_BLOCKING_REFACTOR:
                "Identify all delay() calls in the following code. Refactor them into a non-blocking millis() state machine using the TICK_INTERVAL_MS constant. Ensure the logic remains compatible with the LobotServoController buffer."
        
        HARDWARE_LIMIT_CHECK:
                "Analyze this movement function. Does it validate the output against the MIN_SERVO_ANGLE and MAX_SERVO_ANGLE defined in the KNOWLEDGE_BASE.md? If not, rewrite it with a constrain() layer to prevent mechanical stalling."

3. Mathematical & Kinematic Analysis
        
        IK_EXPLAINER:
                "Before writing code, explain the Inverse Kinematics (IK) math for the SpiderBot's 3-segment leg. Use the Law of Cosines to derive the Femur and Tibia angles. Show the geometric transformation from 3D space (x, y, z) to 2D plane(x,y)" 

        GAIT_PHASE_CALC:
                "Calculate the timing offsets for a Tripod Gait vs. a Ripple Gait. Explain how the TICK_INTERVAL_MS heartbeat handles the phase shift between Leg Group A and Leg Group B without losing synchronization."

4. Debugging & Optimization
        
        UART_TRAFFIC_AUDIT:
                "Review my updateServoBuffer() implementation. Is it effectively using the MOVEMENT_THRESHOLD to reduce UART congestion? Suggest optimizations to minimize the packet size sent to the Lobot controller."
        
        MEMORY_PROFILER:
                "Review the variable types in this IK solver. Suggest where we can use float vs. double to optimize for the ESP32’s FPU (Floating Point Unit) while maintaining precision for 18-degree-of-freedom coordination."

5. Documentation & Meta-Work
        
        CHECKLIST_UPDATE:
                "We just finished the [Feature Name]. Update the CHECKLISTS.md file by marking the relevant items as complete and adding any new 'Pre-Flight' checks required for this specific hardware logic."
        
        STYLE_POLICE:
                "Audit the attached code snippet against our STYLE_GUIDE.md. Identify any 'Magic Numbers' or naming convention violations and provide a 'Diff' to fix them."