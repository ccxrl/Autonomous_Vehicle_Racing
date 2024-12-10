# ir_sensor_handler.py
import Mock.GPIO as GPIO
from time import sleep

class IRSensorHandler:
    def __init__(self, left_pin, center_pin, right_pin):
        """
        Initialize IR sensors with their respective GPIO pins
        White surface typically returns 1 (HIGH)
        Black surface typically returns 0 (LOW)
        """
        self.left_pin = left_pin
        self.center_pin = center_pin
        self.right_pin = right_pin
        
        # Setup GPIO - Using same settings as motor module
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Setup pins as inputs
        GPIO.setup(self.left_pin, GPIO.IN)
        GPIO.setup(self.center_pin, GPIO.IN)
        GPIO.setup(self.right_pin, GPIO.IN)
    
    def read_sensors(self):
        """
        Read all three IR sensors
        Returns: tuple of (left, center, right) values
        1 indicates white surface, 0 indicates black surface
        """
        try:
            left = GPIO.input(self.left_pin)
            center = GPIO.input(self.center_pin)
            right = GPIO.input(self.right_pin)
            return (left, center, right)
        except Exception as e:
            print(f"Error reading IR sensors: {e}")
            return (0, 0, 0)
    
    def all_white(self):
        """
        Check if all sensors are detecting white surface
        Returns: True if all sensors detect white, False otherwise
        """
        sensors = self.read_sensors()
        return all(sensor == 1 for sensor in sensors)
        
    def get_sensor_status(self):
        """
        Get a human-readable status of all sensors
        Useful for debugging
        """
        left, center, right = self.read_sensors()
        return {
            'left': 'WHITE' if left == 1 else 'BLACK',
            'center': 'WHITE' if center == 1 else 'BLACK',
            'right': 'WHITE' if right == 1 else 'BLACK'
        }