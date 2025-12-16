"""
Mock quanser.common module for Limo robot compatibility.
This module provides dummy implementations of Quanser API classes.
YOLO functionality is disabled for Limo platform.
"""


class Timeout:
    """Mock Timeout class for Quanser API compatibility"""
    def __init__(self, seconds=0, nanoseconds=0):
        self.seconds = seconds
        self.nanoseconds = nanoseconds
    
    def __repr__(self):
        return f"Timeout(seconds={self.seconds}, nanoseconds={self.nanoseconds})"
