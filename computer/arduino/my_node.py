import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
import serial

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
#    linear_value = int(msg.linear.x * 10)
#    linear_value = max(-128, min(linear_value, 127))
#    data_to_send = linear_value.to_bytes(1, byteorder='little', signed=True)
    print("Linear Velocity (x): ", msg.linear.x)
    print("Angular Velocity (z): ", msg.angular.z)
#    print("Data to send: ", data_to_send)
#    ser.write(data_to_send+b'\n')
#    ser.write(struct.pack('<i',value))
    
    key_char = None

    if (msg.linear.x > 0.5):
        key_char = 'i'
    else:
        key_char = 'k'
    """ if (msg.linear.x == -0.5):
        key_char = ','
    if (msg.angular.z == 1.0):
        key_char = 'j'
    if (msg.angular.z == -1.0):
        key_char = 'l'
    if (msg.linear.x == 0.0):
        key_char = 'k' # stop """
    """if (msg.linear.x != 0.5 & msg.linear.x != 0.0):
        # send speeds #key_char = 'q'
    """

    if (key_char != None):
        data_to_send = ord(key_char).to_bytes(1, byteorder='little')
        print("data_to_send: ", data_to_send)
        ser.write(data_to_send)

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
