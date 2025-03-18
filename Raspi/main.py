import asyncio
import aioserial
import subprocess
import time
import sys
import os
import board
import busio
import adafruit_bmp3xx
import RPi.GPIO as GPIO
# pip install aioserial Rpi.GPIO adafruit-circuitpython-bmp3xx
# enable i2c and serial in raspi config
# https://www.waveshare.com/wiki/BMP390_Barometric_Pressure_Sensor

# define log directory
log_dir = os.path.expanduser("~/flightlogs")
os.makedirs(log_dir, exist_ok=True)  # Ensure the directory exists

# bmp init
i2c = busio.I2C(board.SCL, board.SDA)
bmp = adafruit_bmp3xx.BMP3XX_I2C(i2c, address=0x77) # install i2c tools and verify address with $ i2cdetect -y 1
bmp.pressure_oversampling = 8
bmp.temperature_oversampling = 4
bmp.sea_level_pressure = 1012 # Set this before flight!

# constants
FLIGHT_LOG = os.path.join(log_dir, time.strftime("flight_%H%M%S.log"))
BMP_LOG = os.path.join(log_dir, time.strftime("bmp_%H%M%S.log"))
TIMEOUT = 20
SERIAL_PORT = "/dev/serial0"
HIGH_RES_VEL = 30  # Ask garg
BAUD_RATE = 115200
DROGUE_PIN = 27
MAIN_PIN = 17
STATUS = 25
TEL = 24
MAIN_VEL = -40 # This should be negative!
MAIN_HEIGHT = 500
FLIGHTMODE_ALT = 200
GROUND_ALT = bmp.altitude
BUFFER_SIZE = 50

# GPIO init
GPIO.setmode(GPIO.BCM) 
GPIO.setup(DROGUE_PIN, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(MAIN_PIN, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(STATUS, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(TEL, GPIO.OUT, initial=GPIO.LOW)

# global variables
arduino_status = True
vel_logging = True
flight_buffer = []
vel_buffer = []
current_alt = 0.0
height = 0.0
velocity = 0.0
last_packet_time = time.monotonic()
previous_alt = bmp.altitude
previous_time = time.monotonic()

async def flight_log(data):
    global flight_buffer
    flight_buffer.append(f"{time.strftime('%H:%M:%S')} - {data}")
    if len(flight_buffer) >= BUFFER_SIZE:
        await asyncio.to_thread(write_to_flight)

def write_to_flight():
    global flight_buffer
    if flight_buffer:
        with open(FLIGHT_LOG, "a") as file:
            file.write("\n".join(flight_buffer) + "\n")
            file.flush()
        flight_buffer = []

async def vel_log(data):
    global vel_buffer
    vel_buffer.append(f"{time.strftime('%H:%M:%S')} - {data}")
    if len(vel_buffer) >= BUFFER_SIZE:
        await asyncio.to_thread(write_to_vel)

def write_to_vel():
    global vel_buffer
    if vel_buffer:
        with open(BMP_LOG, "a") as file:
            file.write("\n".join(vel_buffer) + "\n")
            file.flush()
        vel_buffer = []

async def flush_logs():
    if flight_buffer:
        await asyncio.to_thread(write_to_flight)
    if vel_buffer:
        await asyncio.to_thread(write_to_vel)

async def start_recording():
    script_path = "/home/pi/start_recording.sh"
    if os.path.exists(script_path):
        subprocess.run(["bash", script_path])
    else:
         await flight_log("ERROR: start_recording.sh not found!")

async def stop_recording():
    script_path = "/home/pi/stop_recording.sh"
    if os.path.exists(script_path):
        subprocess.run(["bash", script_path])
    else:
        await flight_log("ERROR: stop_recording.sh not found!")

async def high_res_start():
    script_path = "/home/pi/high_res_start.sh"
    if os.path.exists(script_path):
        subprocess.run(["bash", script_path])
    else:
        await flight_log("ERROR: high_res_start.sh not found!")

async def high_res_stop():
    script_path = "/home/pi/high_res_stop.sh"
    if os.path.exists(script_path):
        subprocess.run(["bash", script_path])
    else:
        await flight_log("ERROR: high_res_stop.sh not found!")

async def high_res_video():
    while height < 200:
        await asyncio.sleep(1)
    while velocity > HIGH_RES_VEL:
        await asyncio.sleep(0.04)
    if arduino_status:
        await stop_recording()
    await high_res_start()
    while velocity > -HIGH_RES_VEL:
        await asyncio.sleep(0.04)
    await high_res_stop()
    if arduino_status:
        await start_recording()

async def deploy_drogue():
    GPIO.output(DROGUE_PIN, GPIO.HIGH)
    await asyncio.sleep(2)
    GPIO.output(DROGUE_PIN, GPIO.LOW)
    await flight_log("Drogue chute deployed!")

async def deploy_main():
    GPIO.output(MAIN_PIN, GPIO.HIGH)
    await asyncio.sleep(2)
    GPIO.output(MAIN_PIN, GPIO.LOW)
    await flight_log("Main chute deployed!")

async def read_serial(aios):
    global last_packet_time
    while arduino_status:
        try:
            data = await aios.readline_async()
            if not data:
                continue

            data = data.decode().strip()
            last_packet_time = time.monotonic()
            await flight_log(data)

            match data:
                case "STOPREC":
                    await stop_recording()
                case "PING":
                    await aios.write_async(b"PONG\n")
                case "EXIT":
                    await cleanup(aios)
                    sys.exit()
           
        except Exception as e:
            await flight_log(f"Serial read error: {e}")
            await asyncio.sleep(0.5)

async def check_failure():
    global last_packet_time, arduino_status
    while True:
        if time.monotonic() - last_packet_time > TIMEOUT:
            await flight_log("ERROR: Arduino timed out! Switching to failure mode.")
            await stop_recording()
            arduino_status = False
            return
        await asyncio.sleep(1)  # Check every second

async def update_vel():
    global previous_alt, previous_time
    await flight_log(f"Started velocity loggging. Ground altitude: {GROUND_ALT}")
    previous_alt = bmp.altitude
    previous_time = time.monotonic()
    while vel_logging:
        await asyncio.to_thread(velocity_calc)
        await vel_log(f"Altitude= {current_alt:.2f}  Height= {height:.2f}  Velocity= {velocity:.2f}")
        await asyncio.sleep(0.04)
    await flight_log("Stopping velocity logging.")

def velocity_calc():
    global previous_alt, previous_time, height, current_alt, velocity
    current_alt = bmp.altitude
    height = current_alt - GROUND_ALT
    delta_alt = current_alt - previous_alt
    current_time = time.monotonic()
    delta_time = current_time - previous_time
    if delta_time > 0:
        velocity = delta_alt / delta_time
    previous_alt = current_alt
    previous_time = current_time

async def failure_mode():
    while height < FLIGHTMODE_ALT:
        await asyncio.sleep(0.25)

async def flight_mode():
    counter = 0
    while True:
        if velocity < 0:
            counter += 1
        else:
            counter = 0
        if counter > 6:
            break
        await asyncio.sleep(0.04)
    await asyncio.sleep(0.5)
    await flight_log("Apogee detected! Deploying drogue.")
    await deploy_drogue()
        
async def descent_mode():
    while height > MAIN_HEIGHT and velocity < MAIN_VEL:
        await asyncio.sleep(0.1)
    await flight_log(f"{MAIN_HEIGHT}m reached, deploying main chute!")
    await deploy_main()

async def landing_mode():
    counter = 0
    while counter < 10:
        if abs(velocity) < 0.5:
            counter += 1
        else:
            counter = 0
        await asyncio.sleep(0.5)
    await flight_log("Landing detected!")

async def cleanup(aios):
    global vel_logging
    vel_logging = False
    GPIO.output(STATUS, GPIO.LOW)
    GPIO.output(TEL, GPIO.LOW)
    GPIO.cleanup()
    await aios.close()
    await flush_logs()
    await flight_log("Cleaned up completed. Exiting")

async def main():
    GPIO.output(STATUS, GPIO.HIGH)
    GPIO.output(TEL, GPIO.HIGH)
    await start_recording()
    aios = aioserial.AioSerial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=1)
    asyncio.gather(update_vel(), high_res_video())
    await asyncio.gather(read_serial(aios), check_failure()) # make sure this function returns if arduino_status is false
    
    # Failure mode
    await flight_log("Entered failure mode.")
    GPIO.output(TEL, GPIO.LOW)
    await failure_mode()
    await flight_mode()
    await descent_mode()
    await landing_mode()
    await cleanup(aios)  

if __name__ == "__main__":
    asyncio.run(main())
