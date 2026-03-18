#### To be complete
- faster due to `Eigen`

# UAV Sim
Simple Hybrid UAV Simulation

## Using `gym-pybullet-drones`

### Case 1: Waypoint Tracking - MPC

MPC (Model Predictive Control) based UAV simple waypoint tracking task: _square climbing_.

#### Mathematical Mode

#### Simulation Results

[uav_waypoint_tracking_mpc.py](uav_waypoint_tracking_mpc.py)

1. State Plot
![state plot](fig/state.png)

2. Trajectory
![traj](fig/traj.png)

3. Running Logs
```log
pybullet build time: Mar  9 2026 01:33:09
[INFO] BaseAviary.__init__() loaded parameters from the drone's .urdf:
[INFO] m 0.027000, L 0.039700,
[INFO] ixx 0.000014, iyy 0.000014, izz 0.000022,
[INFO] kf 3.160000e-10, km 7.940000e-12,
[INFO] t2w 2.250000, max_speed_kmh 30.000000,
[INFO] gnd_eff_coeff 11.368590, prop_radius 0.023135,
[INFO] drag_xy_coeff 0.000001, drag_z_coeff 0.000001,
[INFO] dw_coeff_1 2267.180000, dw_coeff_2 0.160000, dw_coeff_3 -0.110000
Version = 4.1 INTEL-18.8.16
Vendor = Intel Inc.
Renderer = Intel Iris Pro OpenGL Engine
b3Printf: Selected demo: Physics Server
startThreads creating 1 threads.
starting thread 0
started thread 0 
MotionThreadFunc thread started
viewMatrix (0.6427875757217407, -0.4393851161003113, 0.6275069117546082, 0.0, 0.766044557094574, 0.36868780851364136, -0.5265407562255859, 0.0, -0.0, 0.8191521167755127, 0.5735764503479004, 0.0, 2.384185791015625e-07, -0.0, -5.13300085067749, 1.0)
projectionMatrix (0.525390625, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0000200271606445, -1.0, 0.0, 0.0, -0.02000020071864128, 0.0)
/Users/wangxiaonan/miniconda3/envs/drones_py310/lib/python3.10/site-packages/gymnasium/spaces/box.py:236: UserWarning: WARN: Box low's precision lowered by casting to float32, current low.dtype=float64
  gym.logger.warn(
/Users/wangxiaonan/miniconda3/envs/drones_py310/lib/python3.10/site-packages/gymnasium/spaces/box.py:306: UserWarning: WARN: Box high's precision lowered by casting to float32, current high.dtype=float64
  gym.logger.warn(
Physics engine is computing the hybrid system evolution... Please wait.
Jump Triggered! Reached waypoint 0, switching to waypoint 1. Current coordinates: [-0.00, -0.00, 0.85]
Jump Triggered! Reached waypoint 1, switching to waypoint 2. Current coordinates: [1.85, -0.00, 1.01]
Jump Triggered! Reached waypoint 2, switching to waypoint 3. Current coordinates: [2.09, 1.88, 1.51]
Jump Triggered! Reached waypoint 3, switching to waypoint 4. Current coordinates: [0.12, 2.08, 1.51]
All waypoints tracked successfully.
numActiveThreads = 0
stopping threads
Thread with taskId 0 exiting
Thread TERMINATED
destroy semaphore
semaphore destroyed
destroy main semaphore
main semaphore destroyed
```

[uav_waypoint_tracking_mpc_pylib.py](uav_waypoint_tracking_mpc_pylib.py)

1. State Plot
![state plot](fig/state_pylib.png)

2. Trajectory
![traj](fig/traj_pylib.png)

3. Running Logs
```log
pybullet build time: Mar  9 2026 01:33:09
[INFO] BaseAviary.__init__() loaded parameters from the drone's .urdf:
[INFO] m 0.027000, L 0.039700,
[INFO] ixx 0.000014, iyy 0.000014, izz 0.000022,
[INFO] kf 3.160000e-10, km 7.940000e-12,
[INFO] t2w 2.250000, max_speed_kmh 30.000000,
[INFO] gnd_eff_coeff 11.368590, prop_radius 0.023135,
[INFO] drag_xy_coeff 0.000001, drag_z_coeff 0.000001,
[INFO] dw_coeff_1 2267.180000, dw_coeff_2 0.160000, dw_coeff_3 -0.110000
Version = 4.1 INTEL-18.8.16
Vendor = Intel Inc.
Renderer = Intel Iris Pro OpenGL Engine
b3Printf: Selected demo: Physics Server
startThreads creating 1 threads.
starting thread 0
started thread 0 
MotionThreadFunc thread started
viewMatrix (0.6427875757217407, -0.4393851161003113, 0.6275069117546082, 0.0, 0.766044557094574, 0.36868780851364136, -0.5265407562255859, 0.0, -0.0, 0.8191521167755127, 0.5735764503479004, 0.0, 2.384185791015625e-07, -0.0, -5.0, 1.0)
projectionMatrix (0.525390625, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0000200271606445, -1.0, 0.0, 0.0, -0.02000020071864128, 0.0)
/Users/wangxiaonan/miniconda3/envs/drones_py310/lib/python3.10/site-packages/gymnasium/spaces/box.py:236: UserWarning: WARN: Box low's precision lowered by casting to float32, current low.dtype=float64
  gym.logger.warn(
/Users/wangxiaonan/miniconda3/envs/drones_py310/lib/python3.10/site-packages/gymnasium/spaces/box.py:306: UserWarning: WARN: Box high's precision lowered by casting to float32, current high.dtype=float64
  gym.logger.warn(
[MPC] Initialized with:
  - States (nx): 3
  - Inputs (nu): 3
  - Prediction horizon (N): 4
Physics engine is computing the hybrid system evolution... Please wait.
Jump Triggered! Reached waypoint 0, switching to waypoint 1. Current coordinates: [0.00, 0.00, 0.85]
Jump Triggered! Reached waypoint 1, switching to waypoint 2. Current coordinates: [1.85, 0.00, 1.01]
Jump Triggered! Reached waypoint 2, switching to waypoint 3. Current coordinates: [2.10, 1.89, 1.50]
Jump Triggered! Reached waypoint 3, switching to waypoint 4. Current coordinates: [0.11, 2.10, 1.50]
All waypoints tracked successfully.
numActiveThreads = 0
stopping threads
Thread with taskId 0 exiting
Thread TERMINATED
destroy semaphore
semaphore destroyed
destroy main semaphore
main semaphore destroyed
```