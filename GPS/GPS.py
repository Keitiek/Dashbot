import time
import pymap3d as pm
import socket
import pynmea2
import serial
import datetime

# Reference location
lat0 = 58.3428685594
lon0 = 25.5692475361
alt0 = 91.357

def open_serial_port(ports):
    for port in ports:
        try:
            ser = serial.Serial(port, 115200)
            return ser
        except serial.SerialException:
            print(f'Failed to open {port}')
    print("Could not open any specified serial port.")
    return None

def write_data_to_file(E, N, U, timestamp):
    with open('coordinates4.txt', 'a') as f:
        #f.write(f"{timestamp},{E},{N},{U}\n")
        f.write(f"{E},{N},{U}\n")

def process_gps_data(gps_data):
    try:
        # Timestamp when data processing starts
        processing_timestamp = datetime.datetime.now().isoformat()
        
        latitude = gps_data['latitude']
        longitude = gps_data['longitude']
        altitude = gps_data['altitude']
        quality = gps_data['fix quality']

        # Compute ENU coordinates
        E, N, U = pm.geodetic2enu(latitude, longitude, altitude, lat0, lon0, alt0)
        
        # Print and save the processed data
        print(f"[{processing_timestamp}] Coordinates: Latitude={latitude}, Longitude={longitude}, Altitude={altitude}, fix quality={quality}")
        print(f"[{processing_timestamp}] Computed values -> E: {E}, N: {N}, U: {U}")
        write_data_to_file(E, N, U, processing_timestamp)
        #write_data_to_file(E, N, U)
        
        time.sleep(0.5)
        
    except ValueError as e:
        print(f"Error processing GPS data: {gps_data}, Error: {e}")

def gps_callback(gps_data):
    if gps_data['latitude'] == 0.0 and gps_data['longitude'] == 0.0:
        print("----- Go outside for GPS data -----")
        print("The following data is inaccurate!")
        return
    
    process_gps_data(gps_data)

ports_to_try = [
    '/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2', '/dev/ttyACM3', '/dev/ttyACM4',
    '/dev/ttyACM5', '/dev/ttyACM6', '/dev/ttyACM7', '/dev/ttyACM8', '/dev/ttyACM9',
    '/dev/ttyACM10', '/dev/ttyACM11'
]

def main():
    ser = open_serial_port(ports_to_try)
    if ser is None:
        exit()

    HOST = '213.168.5.170'
    PORT = 8002
    BUFFER_SIZE = 1024

    last_process_time = 0  # Initialize the last processing time

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as data_transition:
            data_transition.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            data_transition.connect((HOST, PORT))

            while True:
                try:
                    line = ser.readline().decode('utf-8').strip()
                    if line.startswith('$GNGGA'):
                        current_time = time.time()  # Get the current time

                        # Check if 2 seconds have passed since the last processing
                        if current_time - last_process_time >= 2:
                            msg = pynmea2.parse(line)

                            gps_data = {
                                "latitude": msg.latitude,
                                "longitude": msg.longitude,
                                "altitude": msg.altitude,
                                "fix quality": msg.gps_qual,
                                "Number of Satellites": msg.num_sats
                            }
                            gps_callback(gps_data)

                            # Update the last processing time
                            last_process_time = current_time

                            # Only send data if GPS data is valid
                            if gps_data['latitude'] != 0.0 and gps_data['longitude'] != 0.0:
                                data = data_transition.recv(BUFFER_SIZE)
                                ser.write(data)

                except KeyboardInterrupt:
                    break
                except Exception as e:
                    print(f"An error occurred: {e}")

    finally:
        ser.close()

if __name__ == "__main__":
    main()
