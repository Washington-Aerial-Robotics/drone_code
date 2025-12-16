import serial
import numpy as np
import matplotlib.pyplot as plt
import sys
import time
import subprocess
import os
import serial.tools.list_ports
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from src.visualizeAttitude import visualizeAttitude
from pathlib import Path


# ---------------- Serial ----------------
PORT = "/dev/cu.usbserial-02C554D8"
os.environ["PORT"] = PORT # Change as required, /dev/ttyusb, COM3, e.g.
BAUD = 115200


PROJECT_DIR = Path(__file__).resolve().parent

def run(cmd):
    """Runs a PlatformIO command using the current Python interpreter."""
    print(">", " ".join(cmd))

    subprocess.run([sys.executable, "-m", "platformio"] + cmd, cwd=PROJECT_DIR, check=True)


response = input("(C)onnect or (S)imulate? ")

if (response == "c" or response == "C"):
    #Build firmware
    run(["run"])
    # Upload firmware
    run(["run", "-t", "upload"])
    print("✅ Firmware flashed successfully")
    ser = serial.Serial(PORT, BAUD, timeout=1)


    line = ser.readline().decode("utf-8", errors="ignore").strip()
    prevLine = line
    print(line)
    while(True):
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if (line!=prevLine):
            print(line)
            prevLine = line
        if (line == "setup complete"):
            break
        time.sleep(0.01)


viz_attitude = visualizeAttitude(MaxHz=60)

print("Listening for attitude ... Ctrl+C to stop")

def read_data(line, viz_attitude):
    try:        
        (
        qw, qx, qy, qz, 
        accX, accY, accZ,
        magX, magY, magZ 
        ) = map(float, line.split(","))
    except ValueError:
        return
    
    acc = [accX, accY, -accZ]
    mag = [magX, magY, magZ]
    viz_attitude.update_attitude(qw, qx, qy, qz, acc=acc, mag=mag) # should work for either .apply or .inv().apply, but doesn't

i = 1
# ---------------- Main loop ----------------
try:
    while True:
        if (response == "c" or response == "C"):
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            #with open("data.txt", "a") as file:
                #file.write(line)
                #file.write("\n")
            read_data(line, viz_attitude=viz_attitude)

        elif (response == "s" or response == "S"):
            with open("data.txt", "r") as file:
                lines = file.readlines()
            for line in lines:
                read_data(line, viz_attitude=viz_attitude)
        if not line:
            continue
except KeyboardInterrupt:
    print("\nStopping...")
    plt.close("all")

    

