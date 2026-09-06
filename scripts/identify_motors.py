#!/usr/bin/env python3
import os
import glob
import time
import subprocess

try:
    import minimalmodbus
except ImportError:
    print("Error: minimalmodbus module not found.")
    print("Please install it: pip3 install minimalmodbus")
    print("If you are outside the docker container, you may need to install it on your host system.")
    exit(1)

def check_motor(port, slave_id):
    try:
        instrument = minimalmodbus.Instrument(port, slave_id, minimalmodbus.MODE_ASCII)
        instrument.serial.baudrate = 9600
        instrument.serial.timeout = 0.5
        instrument.clear_buffers_before_each_transaction = True
        
        # Address 10 is Lines per rotation in Rhino RMCS2303, typically 334. Just a safe read.
        # MinimalModbus parameters: read_register(registeraddress, number_of_decimals=0, functioncode=3, signed=False)
        val = instrument.read_register(10, 0, 3, False) 
        return True
    except Exception as e:
        return False

def get_devpath(port):
    try:
        output = subprocess.check_output(["udevadm", "info", "-q", "path", "-n", port]).decode("utf-8").strip()
        # Find the parent USB device path ending in :1.0 (typical for CP210x UART)
        parts = output.split("/")
        for part in reversed(parts):
            if ":1.0" in part:
                return part
        return None
    except Exception:
        return None

def main():
    ports = glob.glob('/dev/ttyUSB*')
    if not ports:
        print("No ttyUSB devices found in /dev/")
        return

    left_port = None
    right_port = None
    lidar_port = None

    print("Checking ports for Companio motors via Modbus...")
    for p in ports:
        print(f"Testing {p}...")
        
        # Test Left Motor (Slave ID 2)
        if check_motor(p, 2):
            print(f"  -> Response received from Slave 2! -> LEFT MOTOR")
            left_port = p
            continue
            
        # Test Right Motor (Slave ID 5)
        if check_motor(p, 5):
            print(f"  -> Response received from Slave 5! -> RIGHT MOTOR")
            right_port = p
            continue
        
        # If neither responded, assume it's LiDAR
        print(f"  -> No motor response. Assuming this is the LiDAR.")
        lidar_port = p

    print("\n==============================")
    print("=== ASSIGNMENT SUMMARY ===")
    print("==============================")
    print(f"Left Motor  : {left_port}")
    print(f"Right Motor : {right_port}")
    print(f"LiDAR       : {lidar_port}")

    # Generate persistent udev rules based on physical paths
    print("\nIf you want to update your persistent udev rules based on the current physical USB ports, copy these rules:\n")
    print("---------------------------------------------------------")
    for name, p in [("motor_left", left_port), ("motor_right", right_port), ("rplidar", lidar_port)]:
        if p:
            # We want to use the physical path so we don't rely on ttyUSBX enumeration orders
            devpath = get_devpath(p)
            if devpath:
                print(f'SUBSYSTEM=="tty", ATTRS{{idVendor}}=="10c4", ATTRS{{idProduct}}=="ea60", DEVPATH=="*{devpath}*", SYMLINK+="{name}"')
    print("---------------------------------------------------------")
    print("\nTo apply these rules, you would paste them into: /etc/udev/rules.d/99-companio.rules")
    print("And then run: sudo udevadm control --reload-rules && sudo udevadm trigger")

if __name__ == '__main__':
    main()
