# motor_control.py

def set_motor_speeds(left_speed, right_speed):
    """
    Function to control motor speeds.
    Replace the print statements with actual motor control commands.
    """
    print(f"Setting motor speeds: Left = {left_speed}, Right = {right_speed}")
    # Add code to send speed commands to the motors (e.g., GPIO, I2C, etc.)

def stop_motors():
    """Stop both motors."""
    set_motor_speeds(0, 0)

def move_forward():
    """Move forward."""
    set_motor_speeds(100, 100)  # Adjust speeds as needed

def turn_left():
    """Turn left."""
    set_motor_speeds(50, 100)  # Left motor slower, right motor faster

def turn_right():
    """Turn right."""
    set_motor_speeds(100, 50)  # Left motor faster, right motor slower

def reverse():
    """Move backward."""
    set_motor_speeds(-100, -100)  # Reverse speed
