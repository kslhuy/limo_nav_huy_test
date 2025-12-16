"""
Mock QCar module for ROS integration
This replaces the actual Quanser QCar hardware with ROS adapters
"""

# Flag to indicate we're NOT using physical QCar (using ROS instead)
IS_PHYSICAL_QCAR = True
IS_LIMO_CAR = True

class QCar:
    """Mock QCar class - actual functionality provided by ROSQCarAdapter"""
    def __init__(self, readMode=1, frequency=100):
        self.readMode = readMode
        self.frequency = frequency
        
    def read(self):
        """This will be overridden by ROSQCarAdapter"""
        return [0.0] * 10  # Return dummy array
    
    def write(self, throttle=0.0, steering=0.0):
        """This will be overridden by ROSQCarAdapter"""
        pass

class QCarGPS:
    """Mock QCarGPS class - actual functionality provided by ROSGPSAdapter"""
    def __init__(self):
        pass
    
    def read(self):
        """This will be overridden by ROSGPSAdapter"""
        return [0.0, 0.0, 0.0]  # x, y, z

# Export the classes
__all__ = ['QCar', 'QCarGPS', 'IS_PHYSICAL_QCAR']
