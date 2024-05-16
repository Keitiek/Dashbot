import serial
import socket
import pynmea2
import time

def open_serial_port(ports):
    for port in ports:
        try:
            ser = serial.Serial(port, 115200)
            return ser
        except serial.SerialException:
            print(f'Failed to open {port}')

    print("Could not open any specified serial port.")
    return None

def gps_callback(gps_data):
    print("Received GPS data:", gps_data)

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
            data_transition = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            data_transition.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            data_transition.connect((HOST, PORT))
            data = data_transition.recv(BUFFER_SIZE)
            fixed = ser.write(data)

        except KeyboardInterrupt:
            break
        
finally:
    ser.close()
