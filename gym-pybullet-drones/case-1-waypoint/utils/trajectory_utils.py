"""
 * @file            gym-pybullet-drones/case-1-waypoint/utils/trajectory_utils.py
 * @description     
 * @author          nicewang <wangxiaonannice@gmail.com>
 * @createTime      2026-03-17
 * @lastModified    2026-03-17
 * Copyright © Xiaonan (Nice) Wang. All rights reserved
"""

import numpy as np

class TrajectoryUtils:
    """
    Utility class for handling (generating & processing) 
    3D flight trajectories and waypoint operations.
    """

    @staticmethod
    def get_square_climbing_waypoints() -> np.ndarray:
        """
        Provides a predefined square climbing trajectory in 3D space.
        
        The path sequence:
        1. Takeoff to 1m height.
        2. Move to (2, 0).
        3. Climb to 1.5m while moving to (2, 2).
        4. Move to (0, 2).
        5. Return to origin (0, 0) and continue climbing to 2m.

        Returns:
            A simple square climbing trajectory in 3D space
        """
        return np.array([
            [0.0, 0.0, 1.0], # Takeoff point
            [2.0, 0.0, 1.0], # Waypoint 1
            [2.0, 2.0, 1.5], # Waypoint 2 (Climbing)
            [0.0, 2.0, 1.5], # Waypoint 3
            [0.0, 0.0, 2.0]  # Waypoint 4 (Return above origin and continue climbing)
        ])
    