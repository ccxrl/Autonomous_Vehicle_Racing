import Mock.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

class Motor():
    def __init__(self, PWM_A, DIR_A, PWM_B, DIR_B):
        """
        Initialize motor driver with Cytron 10A pins
        PWM_A, PWM_B: PWM pins for speed control
        DIR_A, DIR_B: Direction pins for motor direction
        """
        self.PWM_A = PWM_A
        self.DIR_A = DIR_A
        self.PWM_B = PWM_B
        self.DIR_B = DIR_B

        # Setup GPIO pins
        GPIO.setup(self.PWM_A, GPIO.OUT)
        GPIO.setup(self.DIR_A, GPIO.OUT)
        GPIO.setup(self.PWM_B, GPIO.OUT)
        GPIO.setup(self.DIR_B, GPIO.OUT)

        # Setup PWM
        self.pwmA = GPIO.PWM(self.PWM_A, 1000)  # 100Hz frequency
        self.pwmB = GPIO.PWM(self.PWM_B, 1000)

        # Start PWM with 0% duty cycle
        self.pwmA.start(0)
        self.pwmB.start(0)
        self.mySpeed = 0

        # Default turn settings
        self.turn_speed = 50  # Default turning speed (0-100)
        self.turn_time = 0.7  # Default time for 90-degree turn

    def move(self, speed=0.5, turn=0, t=0):
        """
        Move the robot with given speed and turn values
        speed: -1 to 1 (converted to -100 to 100)
        turn: -1 to 1 (converted to -70 to 70)
        t: time to move in seconds
        """
        speed *= 100
        turn *= 70
        leftSpeed = speed - turn
        rightSpeed = speed + turn

        # Constrain speeds to -100 to 100
        leftSpeed = min(max(leftSpeed, -100), 100)
        rightSpeed = min(max(rightSpeed, -100), 100)

        # Set motor directions using DIR pins - INVERTED LOGIC
        GPIO.output(self.DIR_A, leftSpeed < 0)    # Inverted condition
        GPIO.output(self.DIR_B, rightSpeed < 0)   # Inverted condition

        # Set motor speeds (always positive for PWM)
        self.pwmA.ChangeDutyCycle(abs(leftSpeed))
        self.pwmB.ChangeDutyCycle(abs(rightSpeed))

        sleep(t)

    def stop(self, t=0):
        """Stop both motors"""
        self.pwmA.ChangeDutyCycle(0)
        self.pwmB.ChangeDutyCycle(0)
        self.mySpeed = 0
        sleep(t)

    def set_turn_settings(self, speed=20, time=0.7):
        """
        Configure the turning settings
        speed: turning speed (0-100)
        time: duration of 90-degree turn in seconds
        """
        self.turn_speed = min(max(speed, 0), 100)
        self.turn_time = time

    def turn_right_90(self):
        """
        Turn the robot 90 degrees to the right
        Uses class turn_speed and turn_time settings
        """
        self.stop()

        # Left wheel forward, right wheel backward - INVERTED LOGIC
        GPIO.output(self.DIR_A, False)
        GPIO.output(self.DIR_B, True)

        # Set both wheels to turn_speed
        self.pwmA.ChangeDutyCycle(self.turn_speed)
        self.pwmB.ChangeDutyCycle(self.turn_speed)

        # Hold for turn duration
        sleep(self.turn_time)

        # Stop and pause
        self.stop()
        sleep(0.5)  # Short pause after turn

    def turn_left_90(self):
        """
        Turn the robot 90 degrees to the left
        Uses class turn_speed and turn_time settings
        """
        self.stop()

        # Left wheel backward, right wheel forward - INVERTED LOGIC
        GPIO.output(self.DIR_A, True)
        GPIO.output(self.DIR_B, False)

        # Set both wheels to turn_speed
        self.pwmA.ChangeDutyCycle(self.turn_speed)
        self.pwmB.ChangeDutyCycle(self.turn_speed)

        # Hold for turn duration
        sleep(self.turn_time)

        # Stop and pause
        self.stop()
        sleep(0.5)

def main():
    """Test function for the motor driver"""
    motor = Motor(4, 17, 27, 22)  # PWM_A, DIR_A, PWM_B, DIR_B

    # Example usage of different functions
    print("Testing basic movements...")
    motor.move(0.5, 0, 2) 
    motor.stop(1)
    motor.move(-0.5, 0, 2)
    motor.stop(1)

    print("Testing 90-degree turns with default settings...")
    motor.turn_right_90()
    motor.turn_left_90()    

    print("Testing 90-degree turns with custom settings...")
    motor.set_turn_settings(speed=70, time=0.6)
    motor.turn_right_90()
    motor.turn_left_90()

if __name__ == '__main__':
    main()