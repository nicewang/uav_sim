[back](../)
# UAV Sim
Simple Hybrid UAV Simulation

## Using `gym-pybullet-drones`

### Case 1: Waypoint Tracking

#### Mathematical Model    
1. State Space (in 3D space)
- For _Continuous State_:
    - Position: $p\in\mathbb{R}^3$
    - Velocity: $v\in\mathbb{R}^3$
- For _Discrete State_:
    - Index of Waypoint (Tracking Target): $q\in\mathcal{Q}=\left\lbrace 0,1,2,\dots,N \right\rbrace$
        - Coordinates $W=\left\lbrace W_0,W_1,\dots,W_N \right\rbrace$, where $W_q\in\mathbb{R}^3$ is the coordinate of idx $q$
- Thus, the complete **hybrid state vector $x$** of the system is defined as:
  
$$
x = \begin{bmatrix} p \\\ v \\\ q \end{bmatrix} \in \mathbb{R}^6 \times \mathcal{Q}
$$

2. Flow Set $C$

The flow set determines the conditions under which the system undergoes continuous-time physical evolution:

- As long as the Euclidean distance between the UAV and the current target waypoint $W_q$ is greater than or equal to the tolerance radius $\epsilon$
- or if the system has reached the final waypoint ($q=N$), it maintains continuous flight:

$$
C = \left\lbrace x \in \mathbb{R}^6 \times \mathcal{Q} \mid \lVert p - W_q \rVert \ge \epsilon \lor q = N \right\rbrace
$$

3. Flow Map $f$

$(x_t,a_t) \rightarrow x_{t+1}$

Within the flow set, the evolution of the system follows the **differential equation $\dot{x}=f(x)$**. During this phase:
    
- For continuous state:

$$\dot{p} = v$$

$$\dot{v} = \mathcal{F}_{dyn}(p,v,u_{pid}(p,v,W_q))$$
    
- For discrete state $q$: $\dot{q}=0$ since $q$ is invariant in flow set
- Physical state is governed jointly by the underlying _rigid body dynamics_ and the _closed-loop PID controller_:

$$
\dot{x} = f(x) = \left[ \matrix{ v \\\ \mathcal{F}_{dyn}(p, v, u_{pid}(p, v, W_q)) \\\ 0 } \right], \quad x \in C
$$

Note:
- $u_{pid}$ is the _control command_ with $W_q$ acting as the absolute target
- $\mathcal{F}_{dyn}$ represents the _rigid body dynamics integration_ computed by the underlying _6-DOF physics engine_

4. Jump Set ($D$)

The jump set defines the strict boundary where discrete events (_transients_)  are triggered: 

- When the UAV enters the $\epsilon$-neighborhood of the target waypoint
- and the current waypoint is not the final destination

The state machine triggers a jump:

$$
D = \left\lbrace x \in \mathbb{R}^6 \times \mathcal{Q} \mid \lVert p - W_q \rVert < \epsilon \land q < N \right\rbrace
$$

5. Jump Map ($g$) (_i.e. Transition_)

When the state $x\in D$, the system undergoes a discrete jump $x^+=g(x)$. 

- At this instant, the physical position and velocity remain continuous (**no spatial mutation occurs**)
- but the target waypoint index $q$ advances to the next one:

$$x^+=g(x)=\begin{bmatrix}p\\\v\\\q+1\end{bmatrix},\quad x\in D$$

6. Overall **Hybrid Automaton Equations**

$$\mathcal{H} := \begin{cases}
\dot{x}=f(x),&x\in C \\\ x^+=g(x),&x\in D
\end{cases}$$

#### Simulation Results - Initial Exp
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
viewMatrix (0.6427875757217407, -0.4393851161003113, 0.6275069117546082, 0.0, 0.766044557094574, 0.36868780851364136, -0.5265407562255859, 0.0, -0.0, 0.8191521167755127, 0.5735764503479004, 0.0, 2.384185791015625e-07, -0.0, -5.0, 1.0)
projectionMatrix (0.5263671875, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0000200271606445, -1.0, 0.0, 0.0, -0.02000020071864128, 0.0)
/Users/wangxiaonan/miniconda3/envs/drones_py310/lib/python3.10/site-packages/gymnasium/spaces/box.py:236: UserWarning: WARN: Box low's precision lowered by casting to float32, current low.dtype=float64
  gym.logger.warn(
/Users/wangxiaonan/miniconda3/envs/drones_py310/lib/python3.10/site-packages/gymnasium/spaces/box.py:306: UserWarning: WARN: Box high's precision lowered by casting to float32, current high.dtype=float64
  gym.logger.warn(
Physics engine is computing the hybrid system evolution... Please wait.
Jump Triggered! Reached waypoint 0, switching to waypoint 1. Current coordinates: [0.00, 0.00, 0.85]
Jump Triggered! Reached waypoint 1, switching to waypoint 2. Current coordinates: [1.87, -0.00, 0.93]
numActiveThreads = 0
stopping threads
Thread with taskId 0 exiting
Thread TERMINATED
destroy semaphore
semaphore destroyed
destroy main semaphore
main semaphore destroyed
```
![waypoint_tracking](fig/waypoint_tracking.png)

