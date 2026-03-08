import time
import numpy as np
import pybullet as p
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics

# 1. Initialize the minimalist environment
env = CtrlAviary(
    drone_model=DroneModel.CF2X,
    num_drones=1,
    physics=Physics.PYB,
    gui=True # GUI enabled to see the 3D physics
)

# 2. Initialize PID controller
ctrl = DSLPIDControl(drone_model=DroneModel.CF2X)

# --- HYBRID SYSTEM DEFINITION ---
# State vector x = [z, v_z, q]^T
# q is the discrete mode: 1 (Flying up to 1.5m), 2 (Flying down to 0.5m)
q = 1 
target_z = {1: 1.5, 2: 0.5}

obs, info = env.reset(seed=42, options={})
print("Starting Hybrid System Simulation...")

# Main Time-stepping loop
for i in range(10000):
    state = obs[0]
    z = state[2] # Extract current Z altitude
    
    # Evaluate Jump Set (D)
    in_jump_set = (q == 1 and z >= 1.45) or (q == 2 and z <= 0.55)
    
    if in_jump_set:
        # Apply Jump Map (g): Discrete mode switch
        q = 3 - q 
        print(f"Jump Triggered! Switching to mode {q}. Current Altitude: {z:.2f}m")
    
    # Execute Flow Map (f): Continuous control
    action, _, _ = ctrl.computeControlFromState(
        control_timestep=env.CTRL_TIMESTEP,
        state=state,
        target_pos=np.array([0, 0, target_z[q]]),
        target_rpy=np.array([0, 0, 0])
    )
    
    obs, reward, terminated, truncated, info = env.step({0: action})
    time.sleep(env.CTRL_TIMESTEP)

env.close()