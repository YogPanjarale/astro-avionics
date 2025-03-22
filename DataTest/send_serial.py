# this code reads flight_data_cleaned.csv and sends the data to the serial port of arduino

# before start sending data , it should wait for user to press a key
# then send data every 50 ms

import csv
import serial
import time
import sys
import os

PORT = ''

def get_port():
    global PORT
    ports = os.popen('python3 -m serial.tools.list_ports').read().split('\n')
    for port in ports:
        if 'dev/ttyACM' in port:
            PORT = port.split()[0]
            print(f'Arduino found on {PORT}')
            return
    print('Arduino not found')
    sys.exit()

def send_data():
    with open('flight_data_cleaned.csv', 'r') as file:
        reader = csv.reader(file)
        ser = serial.Serial(PORT, 115200)
        print('Waiting for user to press a key...')
        sys.stdin.read(1)
        print('Sending data...')
        for row in reader:
            data = '$' + ','.join(row)
            ser.write(data.encode())
            time.sleep(0.05)
            print("Data sent: ", data)
            # if serial available , read the data
            while ser.in_waiting:
                print(ser.readline().decode())
        print('Data sent')
        ser.close()

if __name__ == '__main__':
    get_port()
    send_data()