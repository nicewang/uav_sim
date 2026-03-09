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

#### Simulation Results
![waypoint_tracking](fig/waypoint_tracking.png)
