"""
 * @file            gym-pybullet-drones/case-1-waypoint/method/mpc/BasicMPC.py
 * @description     
 * @author          nicewang <wangxiaonannice@gmail.com>
 * @createTime      2026-03-15
 * @lastModified    2026-03-15
 * Copyright © Xiaonan (Nice) Wang All rights reserved
"""

import numpy as np
from scipy.optimize import minimize

class BasicMPC:
    def __init__(self, ctrl_dt, horizon=4):
        self.ctrl_dt = ctrl_dt
        self.mpc_dt = 0.05   # Prediction timestep for a longer look-ahead
        self.N = horizon
        self.Q = np.diag([10.0, 10.0, 10.0]) # Weight for position error
        self.R = np.diag([0.1, 0.1, 0.1])    # Weight for control effort (velocity)
        self.max_v = 0.8                     # Max velocity constraint
        self.last_u = np.zeros(self.N * 3)   # Cache previous solution for warm start
        
    def solve(self, current_pos, target_pos):
        # Element 4: Optimization Engine (Solver)
        # Using scipy.optimize to solve the Receding Horizon control problem
        
        # Element 3: Constraints
        # Bounds on velocity inputs to prevent aggressive kinematic maneuvers
        bounds = [(-self.max_v, self.max_v)] * (self.N * 3)
        
        def objective(u):
            # Element 2: Cost Function (Objective Function)
            # Penalizes tracking error and minimizes control effort over the horizon
            cost = 0
            p = current_pos.copy()
            for i in range(self.N):
                v = u[i*3 : (i+1)*3]
                # Element 1: Prediction Model
                # Kinematic state propagation: p(k+1) = p(k) + v(k) * dt
                p = p + v * self.mpc_dt
                
                error = p - target_pos
                cost += error.T @ self.Q @ error + v.T @ self.R @ v
            return cost
            
        # Run Sequential Least SQuares Programming (SLSQP) optimizer
        res = minimize(objective, self.last_u, method='SLSQP', bounds=bounds)
        self.last_u = res.x # Warm start next step
        
        optimal_v = res.x[0:3]
        return optimal_v