"""Position calculation utilities"""
import numpy as np
from config import WORKSPACE_LIMITS

def calculate_safe_picking_position(car_pos_cm):
    """Calculate a safe picking position"""
    car_pos = np.array(car_pos_cm)
    xy_distance = np.sqrt(car_pos[0]**2 + car_pos[1]**2)
    
    # Adjust if too close to base
    if xy_distance < WORKSPACE_LIMITS['xy_min']:
        scale = WORKSPACE_LIMITS['xy_min'] / xy_distance
        car_pos[0] *= scale
        car_pos[1] *= scale
    
    # Set safe height - reduced height for better reachability
    picking_height = max(WORKSPACE_LIMITS['z_min'], min(car_pos[2] + 10.0, 18.0))  # Maximum 18cm
    return np.array([car_pos[0], car_pos[1], picking_height])

def get_relative_position(home_transform, target_transform):
    """Calculate relative position between markers"""
    # Both transforms are 4x4 numpy arrays
    relative_transform = np.linalg.inv(home_transform) @ target_transform
    pos_m = relative_transform[:3, 3]  # Get translation part
    pos_cm = pos_m * 100.0
    # Transform to arm coordinates (Z=0 as markers are on table)
    return np.array([pos_cm[0], pos_cm[1], 0.0]) 