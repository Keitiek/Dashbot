import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
import serial
import struct

def open_serial_port():
    ports = ['/dev/ttyACM1', '/dev/ttyACM0']

    for port in ports:
        try:
            ser = serial.Serial(port)
            print(f"Serial port opened successfully on {port}.")
            return port
        except serial.SerialException:
            print(f"Failed to open {port}.")
          
    print("Could not open any specified serial port.")
    return None

serial_port = open_serial_port()
serial_baudrate = 115200  
arduino_publisher = None

def twist_callback(msg):
    global arduino_publisher, ser
    ser.flushInput()
    print("Linear Velocity (x): ", msg.linear.x)
    print("Angular Velocity (z): ", msg.angular.z)
    
    message_to_send = str((msg.linear.x > 0.5) * (msg.linear.x * 10 + 10)) # if bigger than 0.5, sends x * 10 + 10, else sends 0
    print("string to send : ", message_to_send)

    ser.write(message_to_send.encode() + b'\n')

def main():
    global arduino_publisher, ser
    rclpy.init()
    node = rclpy.create_node('twist_arduino_node')

    subscription = node.create_subscription(
        Twist,
        '/cmd_vel',
        twist_callback,
        10
    )
    ser = serial.Serial(serial_port, serial_baudrate)

    while rclpy.ok():
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
