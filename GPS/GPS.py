import serial
import socket
import pynmea2
import time
import pymap3d as pm
import math

class Point:
    def __init__(self, lat, lon):
        self.lat = lat
        self.lon = lon
        
lat0 = 58.3428685594
lon0 = 25.5692475361  # corrected variable name
alt0 = 91.357

base_point = Point(lat0, lon0)

#print(base_point.lat)

lat1 = 58.342035
lon1 = 25.568843166666667

home_point = Point(lat1, lon1)

#print(home_point.lon)

lat = 58.34221
lon = 25.5680983
alt = 80.6
def open_serial_port(ports):
    for port in ports:
        try:
            ser = serial.Serial(port, 115200)
            return ser
        except serial.SerialException:
            print(f'Failed to open {port}')
    print("Could not open any specified serial port.")
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

def gps_callback(gps_data):
    write_data_to_file(gps_data)
    print_gps_data(gps_data)
    
    E, N, H = pm.geodetic2enu(lat, lon, alt, lat0, lon0, alt0)
    #E, N, H = pm.geodetic2enu({gps_data['latitude']},{gps_data['longitude']},{gps_data['altitude']}, lat0, lon0, alt0)
    print(E, " | ", N, " | ", H)
    angle = angle_theta(E, N)
    print('angle', (angle * 180) / math.pi)
    #result = quadrant(E, N)
    #print(result)


def angle_theta(E, N):
    return math.atan2(N, E)
    #distance_to_base = math.sqrt((N * N) + (E * E)) # distance to base
    #absolute_angle = quadrant(N, E) + math.atan(N / E) # absolute angle between bot and base in radians
    #print(absolute_angle)
       
def quadrant(E, N):
    if E > 0 and N > 0:
        return 0
    elif E > 0 and N < 0:
        return 2 * math.pi
    elif  E < 0:
        return math.pi
    else:
        return 0

ports_to_try = ['/dev/tty_Ardusimple']
                
ser = open_serial_port(ports_to_try)

if ser is None:
    exit()

try:
    HOST = '213.168.5.170'
    PORT = 8002
    BUFFER_SIZE = 1024
    while True:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line.startswith('$GNGGA'):
                msg = pynmea2.parse(line)

                gps_data = {
                    "latitude": msg.latitude,
                    "longitude": msg.longitude,
                    "altitude": msg.altitude,
                    "fix quality": msg.gps_qual,
                    "Number of Satellites": msg.num_sats
                }
                gps_callback(gps_data)
            
            time.sleep(2)

            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as data_transition:
                data_transition.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                data_transition.connect((HOST, PORT))
                data = data_transition.recv(BUFFER_SIZE)
                ser.write(data)

        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"An error occurred: {e}")

finally:
    ser.close()
