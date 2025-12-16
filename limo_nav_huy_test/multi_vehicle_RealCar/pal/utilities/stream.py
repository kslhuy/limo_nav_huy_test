"""
Mock pal.utilities.stream module for Limo robot compatibility.
This module provides dummy implementations for YOLO streaming functionality.
YOLO is disabled for Limo platform - BasicStream will not attempt any connections.
"""

import numpy as np


class BasicStream:
    """
    Mock BasicStream class for Limo robot compatibility.
    This simulates the Quanser streaming interface but does not establish any connections.
    """
    
    def __init__(self, uri='', agent='C', receiveBuffer=None, sendBuffer=None,
                 recvBufferSize=0, sendBufferSize=0, nonBlocking=True, reshapeOrder='C'):
        self.uri = uri
        self.agent = agent
        self.receiveBuffer = receiveBuffer if receiveBuffer is not None else np.zeros((5, 7), dtype=np.float64)
        self.sendBuffer = sendBuffer
        self.recvBufferSize = recvBufferSize
        self.sendBufferSize = sendBufferSize
        self.nonBlocking = nonBlocking
        self.reshapeOrder = reshapeOrder
        
        # Always report as disconnected for Limo platform
        self.connected = False
    
    def checkConnection(self, timeout=None):
        """
        Check connection status. Always returns False for Limo platform.
        """
        return False
    
    def receive(self, timeout=None, iterations=1):
        """
        Mock receive method. Returns no new data for Limo platform.
        
        Returns:
            tuple: (new_data, bytes_received) - Always (False, 0)
        """
        return False, 0
    
    def send(self, data, timeout=None):
        """
        Mock send method. Does nothing for Limo platform.
        
        Returns:
            tuple: (success, bytes_sent) - Always (False, 0)
        """
        return False, 0
    
    def terminate(self):
        """
        Terminate the stream. Does nothing for Limo platform.
        """
        pass
    
    def __enter__(self):
        """Used for with statement."""
        return self
    
    def __exit__(self, type, value, traceback):
        """Used for with statement. Terminates the stream."""
        self.terminate()
