"""
 * @file            gym-pybullet-drones/case-1-waypoint/case-1-mpc/uav_waypoint_tracking_mpc_pylib.py
 * @description     
 * @author          nicewang <wangxiaonannice@gmail.com>
 * @createTime      2026-03-15
 * @lastModified    2026-03-25
 * Copyright © Xiaonan (Nice) Wang. All rights reserved
"""

import numpy as np
import matplotlib.pyplot as plt
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics

# Import Model Predictive Control (MPC) Module
from mpc_python import MPCController

import os, sys

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)

sys.path.insert(0, parent_dir)

# Import TrajectoryUtils which includes the tasks
from utils.trajectory_utils import TrajectoryUtils

# ================================================================================
# 1. Initialization
# ================================================================================

# ------------------------------------------------------------
# 1.1 Define Waypoints
# Task: Tracking a square climbing trajectory in 3D space
# ------------------------------------------------------------
WAYPOINTS = TrajectoryUtils.get_square_climbing_waypoints()

# ------------------------------------------------------------
# 1.2 Environment Initialization
# ------------------------------------------------------------
env = CtrlAviary(
    drone_model=DroneModel.CF2X,
    num_drones=1,
    physics=Physics.PYB,
    gui=True
)
ctrl = DSLPIDControl(drone_model=DroneModel.CF2X)

# --- Hybrid System State Initialization ---
q = 0             # Discrete state: Current target waypoint index (0 to 4)
epsilon = 0.15    # Jump tolerance radius (triggers Jump when within 0.15m of the target)

obs, info = env.reset(seed=42, options={})

# todo: check `[OPTIMIZATION]: Initialize the virtual target position at the drone's spawn location`
# [OPTIMIZATION]: Initialize the virtual target position at the drone's spawn location
virtual_target_pos = obs[0][0:3].copy() 

# ------------------------------------------------------------
# 1.3 Model Predictive Control (MPC) Initialization
# ------------------------------------------------------------

# Define 3D kinematic model: x(k+1) = x(k) + v(k) * dt
A_mat = np.eye(3)
B_mat = np.eye(3) * env.CTRL_TIMESTEP
Q_mat = np.eye(3) * 100.0  # Strong position tracking penalty
R_mat = np.eye(3) * 0.1    # Moderate control effort (velocity) penalty
u_min_vec = np.array([-2.0, -2.0, -2.0]) # Velocity limits bounds
u_max_vec = np.array([2.0, 2.0, 2.0])
N_horizon = 4

mpc_controller = MPCController(A_mat, B_mat, Q_mat, R_mat, u_min_vec, u_max_vec, N_horizon)

# [MODIFIED]: Cache variable to store the last optimal control since it's not exposed in C++
last_optimal_v = np.zeros(3)

# Data containers for recording the trajectory for plotting
history_t = []       # Time t
history_pos = []     # Continuous state: Position (x, y, z)
history_rpy = []     # Continuous state: Orientation (Roll, Pitch, Yaw)
history_q = []       # Discrete state: Mode q
history_jump_points = [] 

print("Physics engine is computing the hybrid system evolution... Please wait.")

# ================================================================================
# 2. Main Simulation Loop
# ================================================================================
for i in range(10000):
    # Extract the current continuous physical state
    state = obs[0]
    pos = state[0:3]
    rpy = state[7:10] # Extract Roll, Pitch, Yaw from state vector
    current_time = i * env.CTRL_TIMESTEP
    
    # Log current states BEFORE checking jumps
    history_t.append(current_time)
    history_pos.append(pos)
    history_rpy.append(rpy)
    history_q.append(q)
    
    target_pos = WAYPOINTS[q]
    
    # ------------------------------------------------------------
    # Hybrid System: Evaluate Jump Set (D)
    # Condition: The Euclidean distance between current position and target is less than epsilon
    # ------------------------------------------------------------
    dist = np.linalg.norm(pos - target_pos)
    in_jump_set = (dist < epsilon)
    
    if in_jump_set and q < len(WAYPOINTS) - 1:
        # ------------------------------------------------------------
        # Hybrid System: Execute Jump Map (g) - Transition
        # ------------------------------------------------------------
        q += 1
        history_jump_points.append(pos)
        print(f"Jump Triggered! Reached waypoint {q-1}, switching to waypoint {q}. Current coordinates: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")
    

    if in_jump_set and q == len(WAYPOINTS) - 1 and dist < 0.05:
        print("All waypoints tracked successfully.")
        break
        
    # ------------------------------------------------------------
    # Hybrid System: Execute Flow Map (f) - PID calculation and physical stepping
    # ------------------------------------------------------------
    
    # todo: check `[MODIFIED]: Upgraded to Model Predictive Control (MPC)`
    # [MODIFIED]: Upgraded to Model Predictive Control (MPC)
    # Execute MPC optimization at a specific sampling rate (e.g., every 20 steps) to balance real-time performance and CPU load
    if i % 20 == 0:
        # [MODIFIED]: Plan from the virtual target (the 'carrot') instead of the lagging drone position
        optimal_v = mpc_controller.solve(virtual_target_pos, WAYPOINTS[q])
        
        # [MODIFIED]: Cache the solved optimal velocity
        last_optimal_v = optimal_v.copy()
    else:
        # [MODIFIED]: Use cached velocity instead of accessing controller internal state
        optimal_v = last_optimal_v

    # todo: check `[MODIFIED]: Continuously integrate the optimal velocity to advance the virtual target` 
    # [MODIFIED]: Continuously integrate the optimal velocity to advance the virtual target
    virtual_target_pos = virtual_target_pos + optimal_v * env.CTRL_TIMESTEP

    action, _, _ = ctrl.computeControlFromState(
        control_timestep=env.CTRL_TIMESTEP,
        state=state,
        target_pos=virtual_target_pos, # [MODIFIED]: Feed the virtual target instead of the absolute waypoint
        target_rpy=np.array([0, 0, 0])
    )
    
    obs, reward, terminated, truncated, info = env.step(np.array([action]))

env.close()

# Convert logs to numpy arrays for plotting
history_t = np.array(history_t)
history_pos = np.array(history_pos)
history_rpy = np.array(history_rpy)
history_q = np.array(history_q)

# ================================================================================
# 3. Visualization: Generating Academic-Level Figures
# ================================================================================

# --- Figure 1: 3D Spatial Trajectory ---
fig1 = plt.figure(figsize=(10, 8))
ax1 = fig1.add_subplot(111, projection='3d')
ax1.plot(history_pos[:, 0], history_pos[:, 1], history_pos[:, 2], label='UAV Trajectory (Continuous Flow)', color='b', linewidth=2)
ax1.scatter(WAYPOINTS[:, 0], WAYPOINTS[:, 1], WAYPOINTS[:, 2], color='r', s=100, label='Waypoints  (Targets)', marker='X')
if history_jump_points:
    jp = np.array(history_jump_points)
    ax1.scatter(jp[:, 0], jp[:, 1], jp[:, 2], color='g', s=80, label='Jump Events (Discrete Transitions)', marker='o')
ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)')
ax1.set_zlabel('Altitude Z (m)')
ax1.set_title('UAV Hybrid System Simulation (Waypoint Tracking) - Trajectory')
ax1.legend()

# --- Figure 2: Time-Domain State Evolution ---
fig2, axs = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

# Subplot 1: Position
axs[0].plot(history_t, history_pos[:, 0], label='X')
axs[0].plot(history_t, history_pos[:, 1], label='Y')
axs[0].plot(history_t, history_pos[:, 2], label='Z')
axs[0].set_ylabel('Position (m)')
axs[0].set_title('Continuous and Discrete States over Time')
axs[0].legend(loc='upper right')
axs[0].grid(True, linestyle='--', alpha=0.7)

# Subplot 2: Orientation Angle
axs[1].plot(history_t, np.degrees(history_rpy[:, 0]), label='Roll')
axs[1].plot(history_t, np.degrees(history_rpy[:, 1]), label='Pitch')
axs[1].plot(history_t, np.degrees(history_rpy[:, 2]), label='Yaw')
axs[1].set_ylabel('Orientation (deg)')
axs[1].legend(loc='upper right')
axs[1].grid(True, linestyle='--', alpha=0.7)

# Subplot 3: Discrete Mode
axs[2].step(history_t, history_q, where='post', color='k', linewidth=2, label='Mode q')
axs[2].set_xlabel('Time t (s)')
axs[2].set_ylabel('Discrete Mode')
axs[2].set_yticks(range(len(WAYPOINTS)))
axs[2].legend(loc='upper right')
axs[2].grid(True, linestyle='--', alpha=0.7)

plt.tight_layout()
plt.show()