import pynmea2
import pymap3d as pm
import socket
import math
import rclpy
import serial
import time
from rclpy.node import Node
from std_msgs.msg import Float32
import threading
# from sensor_msgs.msg import NavSatFix  # Assuming the GPS data comes as NavSatFix

class Point:
    def __init__(self, latitude, longitude, altitude):
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude

lat0 = 58.3428685594
lon0 = 25.5692475361  
alt0 = 91.357

lat = 58.34221
lon = 25.5680983

# for testing outdoors
base_point = Point(58.3428685594, 25.5692475361, 91.357)
home_point = Point(58.342035, 25.56852433333333, 64.3)
gps_port = '/dev/ttyACM0'

def open_serial_port_gps(port):
    try:
        ser_gps = serial.Serial(port)
        print(f"Serial port opened successfully on {port}.")
        return ser_gps
    except serial.SerialException:
        print(f"Failed to open {port}.")

    print("Could not open specified serial port for GPS.")
    return None

def write_data_to_file(gps_data):
    with open('coordinates.txt', 'a') as f:  # changed to append mode
        f.write(f"{gps_data['latitude']},{gps_data['longitude']},{gps_data['altitude']}\n")
    
def print_gps_data(gps_data):
    if gps_data['latitude'] == 0.0 and gps_data['longitude'] == 0.0:
        print(" ----- Go outside for GPS data -----")
        print(" The following data is inaccurate! ")
    else:
        print("Received GPS data:", gps_data)

def angle_theta(E, N):
    return math.atan2(N, E)

def quadrant(E, N):
    if E > 0 and N > 0:
        return 0
    elif E > 0 and N < 0:
        return 2 * math.pi
    elif  E < 0:
        return math.pi
    else:
        return 0

class GPSPublisher(Node):
    def __init__(self):
        super().__init__('DASHBOT_gps_publisher')

        self.serial_port_gps = None
        self.gps_state = Float32()
        self.my_location = Point(0.0, 0.0, 0.0)
        #self.desired_location = Point(666.0, 666.0, 666.0)
        self.home = home_point
        self.E = 100.0
        self.N = 100.0
        self.U = 100.0
        self.quality = 0.0

        # create publisher for 'dashbot_gps_message' topic
        self.publisher = self.create_publisher(Float32, 'dashbot_gps_message', 10)

        # Create a timer to publish the message periodically
        #timer_period = 1.0 # seconds
        #self.timer = self.create_timer(timer_period, self.gps_callback)

        while not self.serial_port_gps:
            self.serial_port_gps = open_serial_port_gps(gps_port)
            if not self.serial_port_gps:
                time.sleep(1)

        # Start a thread to read GPS data
        self.gps_thread = threading.Thread(target=self.read_gps_data(), args=(self.serial_port_gps,))
        self.gps_thread.daemon = True
        self.gps_thread.start()    

        #self.read_gps_data(self.serial_port_gps)

    def gps_callback(self):
        x = self.am_i_home()
        if self.am_i_home == True:
            self.gps_state.data = 5.5 # self.my_location.latitude # todo: change to a custom r>
        else:
            self.gps_state.data = 0.0
        self.publisher.publish(self.gps_state)
        self.get_logger().info(f'GPS (home?): {x}, E: {self.E},  N: {self.N}, quality: {self.quality}')
        if self.E > 0:
            print('Move west')
        else:
            print('Move east')
        if self.N > 0:
            print('Move south')
        else:
            print('Move north')

    def am_i_home(self):
        return abs(self.E) < 4 and abs(self.N) < 4

    def read_gps_data(self):
        try:
            HOST = '213.168.5.170'
            PORT = 8002
            BUFFER_SIZE = 1024

            # Create the socket connection outside the main loop
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as data_transition:
                data_transition.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                data_transition.connect((HOST, PORT))

                while True:
                    try:
                        # Read a chunk of data from the serial port
                        data = self.serial_port_gps.read(BUFFER_SIZE)

                        # Split the data into individual lines
                        lines = data.decode('utf-8').strip().split('\n')

                        for line in lines:
                            if line.startswith('$GNGGA'):
                                msg = pynmea2.parse(line)

                                gps_data = {
                                    "latitude": msg.latitude,
                                    "longitude": msg.longitude,
                                    "altitude": msg.altitude,
                                    "fix quality": msg.gps_qual,
                                    "Number of Satellites": msg.num_sats
                                }

                                self.my_location.latitude = gps_data['latitude']
                                self.my_location.longitude = gps_data['longitude']
                                self.my_location.altitude = gps_data['altitude']
                                self.quality = gps_data['fix quality'] * 1.0

                                self.E, self.N, self.U = pm.geodetic2enu(
                                    self.my_location.latitude, self.my_location.longitude, self.my_location.altitude,
                                    self.home.latitude, self.home.longitude, self.home.altitude)

                                self.gps_callback()

                                # Send the data to the socket
                                data_transition.sendall(line.encode('utf-8') + b'\n')
                                #time.sleep(1)

                    except KeyboardInterrupt:
                        break
                    except Exception as e:
                        self.get_logger().error(f"An error occurred: {e}")

        finally:
            self.serial_port_gps.close()

def main(args=None):
    print('started gps.py')
    rclpy.init(args=args)  
    node = GPSPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__': 
    main()
