#!/usr/bin/python3
"""Session data for robot skill execution context"""

class Session:
    """Stores context information for skill execution"""
    def __init__(self):
        self.object = ""
        self.place = {}  # dict with bin, pose_name, or joints
        self.plan_only = False
        self.vel_scale = 1.0
        self.acc_scale = 1.0
