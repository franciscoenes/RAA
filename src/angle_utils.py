"""Utility functions for angle operations"""

def normalize_angle(angle):
    """Normalize angle to be between -180 and 180 degrees"""
    return ((angle + 180) % 360) - 180 