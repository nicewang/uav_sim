import numpy as np
import matplotlib.pyplot as plt
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics

# 1. Define Waypoints in the Hybrid System (similar to the predefined path in vehicle-on-path)
# This represents a square climbing trajectory in 3D space
WAYPOINTS = np.array([
    [0.0, 0.0, 1.0],  # Takeoff point
    [2.0, 0.0, 1.0],  # Waypoint 1
    [2.0, 2.0, 1.5],  # Waypoint 2 (Climbing)
    [0.0, 2.0, 1.5],  # Waypoint 3
    [0.0, 0.0, 2.0]   # Waypoint 4 (Return above origin and continue climbing)
])

# 2. Initialize Environment: Crucial point! gui=False bypasses Mac graphics card crash issues
env = CtrlAviary(
    drone_model=DroneModel.CF2X,
    num_drones=1,
    physics=Physics.PYB,
    gui=False  # Disable OpenGL GUI rendering for silent background computation
)
ctrl = DSLPIDControl(drone_model=DroneModel.CF2X)

# --- Hybrid System State Initialization ---
q = 0             # Discrete state: Current target waypoint index (0 to 4)
epsilon = 0.15    # Jump tolerance radius (triggers Jump when within 0.15m of the target)

obs, info = env.reset(seed=42, options={})
state = obs[0]

# Data containers for recording the trajectory for plotting
history_pos = []
history_jump_points = [] # Records the 3D coordinates where a Jump event occurs

print("Physics engine is computing the hybrid system evolution in the background. Please wait...")

# 3. Main Simulation Loop (Time-stepping)
for i in range(8000): # Run enough steps to complete the trajectory
    # Extract the current continuous physical state
    state = obs[0]
    pos = state[0:3]
    history_pos.append(pos)
    
    # Get the current target waypoint
    target_pos = WAYPOINTS[q]
    
    # ----------------------------------------------------
    # Hybrid System: Evaluate Jump Set (D)
    # Condition: The Euclidean distance between current position and target is less than epsilon
    # ----------------------------------------------------
    dist = np.linalg.norm(pos - target_pos)
    in_jump_set = (dist < epsilon)
    
    if in_jump_set and q < len(WAYPOINTS) - 1:
        # Execute Jump Map (g): Advance the discrete state q
        q += 1
        history_jump_points.append(pos)
        print(f"Jump Triggered! Reached waypoint {q-1}, switching to waypoint {q}. Current coordinates: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")
    
    if in_jump_set and q == len(WAYPOINTS) - 1 and dist < 0.05:
        print("All waypoints tracked successfully. Terminating continuous evolution.")
        break
        
    # ----------------------------------------------------
    # Hybrid System: Execute Flow Map (f) - PID calculation and physical stepping
    # ----------------------------------------------------
    action, _, _ = ctrl.computeControlFromState(
        control_timestep=env.CTRL_TIMESTEP,
        state=state,
        target_pos=WAYPOINTS[q],
        target_rpy=np.array([0, 0, 0])
    )
    
    obs, reward, terminated, truncated, info = env.step(np.array([action]))

env.close()

# 4. Visualization: Generate an academic-level trajectory plot matching the web example
history_pos = np.array(history_pos)

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot the actual continuous flight trajectory of the UAV (Flow)
ax.plot(history_pos[:, 0], history_pos[:, 1], history_pos[:, 2], label='UAV Trajectory (Continuous Flow)', color='b', linewidth=2)

# Plot the predefined waypoints (Targets)
ax.scatter(WAYPOINTS[:, 0], WAYPOINTS[:, 1], WAYPOINTS[:, 2], color='r', s=100, label='Waypoints (Targets)', marker='X')

# Plot the actual physical positions where Jump events (mode switches) occurred
if history_jump_points:
    jp = np.array(history_jump_points)
    ax.scatter(jp[:, 0], jp[:, 1], jp[:, 2], color='g', s=80, label='Jump Events (Discrete Transitions)', marker='o')

ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Altitude Z (m)')
ax.set_title('UAV Hybrid System Simulation (Waypoint Tracking)')
ax.legend()
plt.show()