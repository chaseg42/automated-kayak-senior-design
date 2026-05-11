#!/usr/bin/env python3
"""
radar_controlled.py

Runs on Raspberry Pi.

Purpose:
- Waits for commands from the STM32 over USB serial.
- STM32 sends:
      0x671\n  -> turn radar on
      0x670\n  -> turn radar off
- When radar is turned on, this script starts radar_usb.py.
- When radar is turned off, this script stops radar_usb.py.

This file is meant to be started automatically by radar.service.
"""

import os
import sys
import time
import glob
import signal
import serial
import subprocess


# -------------------------------------------------
# User settings
# -------------------------------------------------

USB_BAUD = 9600

# This should be the folder where both radar_controlled.py
# and radar_usb.py are located.
RADAR_DIR = "/home/analog/radar"

# Full path to radar_usb.py
RADAR_USB_FILE = os.path.join(RADAR_DIR, "radar_usb.py")

# Use the same Python interpreter that is running this script.
# This is important if radar.service uses the virtual environment.
PYTHON_EXE = sys.executable

# STM32 command values
RADAR_ON_CMD = "0x671"
RADAR_OFF_CMD = "0x670"

# Time between checking the serial port
LOOP_DELAY_S = 0.05


# -------------------------------------------------
# Serial port helper
# -------------------------------------------------

def find_stm_port():
    """
    Finds the STM32 USB serial port.

    STM32 virtual COM ports usually show up as /dev/ttyACM0 or /dev/ttyACM1.
    This checks the most likely ports first.
    """

    preferred_ports = [
        "/dev/ttyACM1",
        "/dev/ttyACM0",
        "/dev/ttyUSB0",
        "/dev/ttyUSB1",
    ]

    for port in preferred_ports:
        if os.path.exists(port):
            print(f"Found STM32 serial port: {port}", flush=True)
            return port

    acm_ports = sorted(glob.glob("/dev/ttyACM*"))
    usb_ports = sorted(glob.glob("/dev/ttyUSB*"))

    all_ports = acm_ports + usb_ports

    if len(all_ports) > 0:
        print(f"Found STM32 serial port: {all_ports[0]}", flush=True)
        return all_ports[0]

    raise RuntimeError("No STM32 serial port found. Checked /dev/ttyACM* and /dev/ttyUSB*.")


def open_stm_serial():
    """
    Opens USB serial connection to STM32.
    """

    port = find_stm_port()

    print(f"Opening STM32 serial port {port} at {USB_BAUD} baud", flush=True)

    ser = serial.Serial(
        port=port,
        baudrate=USB_BAUD,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.1,
        write_timeout=1,
    )

    # Some STM32 boards reset when the serial port opens.
    time.sleep(2.0)

    print("STM32 serial ready", flush=True)

    return ser


# -------------------------------------------------
# Radar subprocess control
# -------------------------------------------------

def radar_is_running(proc):
    """
    Returns True if radar_usb.py subprocess is still running.
    """

    if proc is None:
        return False

    return proc.poll() is None


def start_radar(proc):
    """
    Starts radar_usb.py if it is not already running.
    """

    if radar_is_running(proc):
        print("Radar is already running", flush=True)
        return proc

    if not os.path.exists(RADAR_USB_FILE):
        print(f"ERROR: radar_usb.py not found at {RADAR_USB_FILE}", flush=True)
        return None

    print("Starting radar_usb.py", flush=True)
    print(f"Command: {PYTHON_EXE} {RADAR_USB_FILE}", flush=True)

    proc = subprocess.Popen(
        [PYTHON_EXE, RADAR_USB_FILE],
        cwd=RADAR_DIR,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    print(f"radar_usb.py started with PID {proc.pid}", flush=True)

    return proc


def stop_radar(proc):
    """
    Stops radar_usb.py if it is running.
    """

    if not radar_is_running(proc):
        print("Radar is already stopped", flush=True)
        return None

    print("Stopping radar_usb.py", flush=True)

    try:
        proc.terminate()

        try:
            proc.wait(timeout=5)
            print("radar_usb.py stopped cleanly", flush=True)

        except subprocess.TimeoutExpired:
            print("radar_usb.py did not stop cleanly. Killing it.", flush=True)
            proc.kill()
            proc.wait(timeout=5)
            print("radar_usb.py killed", flush=True)

    except Exception as e:
        print("Error while stopping radar_usb.py:", repr(e), flush=True)

    return None


def read_radar_output(proc):
    """
    Reads and prints any available output from radar_usb.py.

    This keeps the systemd journal useful because output from radar_usb.py
    will show up under radar.service.
    """

    if not radar_is_running(proc):
        return

    if proc.stdout is None:
        return

    # Non-blocking stdout reading is tricky with pipes.
    # This function is intentionally simple.
    # The radar program output may still appear through systemd depending
    # on how the service is configured.
    return


# -------------------------------------------------
# Command parser
# -------------------------------------------------

def parse_command(line):
    """
    Cleans up the command string from STM32.

    Accepts:
        0x671
        0X671
        671
        0x670
        0X670
        670
    """

    if line is None:
        return ""

    cmd = line.strip()

    if cmd == "":
        return ""

    cmd = cmd.lower()

    if cmd == "671":
        cmd = "0x671"

    if cmd == "670":
        cmd = "0x670"

    return cmd


# -------------------------------------------------
# Main
# -------------------------------------------------

def main():
    print("Starting radar_controlled.py", flush=True)
    print("Waiting for STM32 commands over USB serial", flush=True)
    print("STM32 sends:", flush=True)
    print("  0x671 -> turn radar on", flush=True)
    print("  0x670 -> turn radar off", flush=True)
    print(f"Radar script path: {RADAR_USB_FILE}", flush=True)
    print(f"Python interpreter: {PYTHON_EXE}", flush=True)

    ser = None
    radar_proc = None

    try:
        ser = open_stm_serial()

        print("Ready. Waiting for STM32 0x671 command.", flush=True)

        while True:
            # If radar_usb.py crashed, clear the process handle.
            if radar_proc is not None and radar_proc.poll() is not None:
                print(
                    f"radar_usb.py exited with code {radar_proc.returncode}",
                    flush=True
                )
                radar_proc = None

            try:
                raw = ser.readline()

                if raw:
                    try:
                        line = raw.decode("ascii", errors="ignore")
                    except Exception:
                        line = ""

                    cmd = parse_command(line)

                    if cmd != "":
                        print(f"Received STM32 command: {cmd}", flush=True)

                    if cmd == RADAR_ON_CMD:
                        radar_proc = start_radar(radar_proc)

                    elif cmd == RADAR_OFF_CMD:
                        radar_proc = stop_radar(radar_proc)

                    elif cmd != "":
                        print(f"Unknown command from STM32: {cmd}", flush=True)

            except serial.SerialException as e:
                print("Serial error:", repr(e), flush=True)
                print("Trying to reopen STM32 serial port...", flush=True)

                try:
                    if ser is not None:
                        ser.close()
                except Exception:
                    pass

                time.sleep(1.0)
                ser = open_stm_serial()

            time.sleep(LOOP_DELAY_S)

    except KeyboardInterrupt:
        print("\nStopped by user", flush=True)

    except Exception as e:
        print("Fatal error in radar_controlled.py:", repr(e), flush=True)

    finally:
        print("Cleaning up radar_controlled.py", flush=True)

        radar_proc = stop_radar(radar_proc)

        if ser is not None:
            try:
                ser.close()
                print("Closed STM32 serial port", flush=True)
            except Exception:
                pass

        print("Done", flush=True)


if __name__ == "__main__":
    main()
