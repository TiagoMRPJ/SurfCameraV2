import cv2
import os
import serial
import sys
import time
import threading
import Camera
import numpy as np

import db

CHC_MOVE    = 8
CHB_MOVE    = 7
CHA_MOVE    = 6
CHC_PI      = 5
CHB_PI      = 4
CHA_PI      = 3
CHC_POS     = 2
CHB_POS     = 1
CHA_POS     = 0
MAX_FOCUS_STEP = 50


def normalize(value, range_min, range_max):
    return (value - range_min) / (range_max - range_min)


class SCF4():
    def send_command(self,ser, cmd, echo=False):
        ser.write(bytes(cmd+"\n", 'utf8'))
        data_in = ser.readline().decode('utf-8').strip()
        if echo:
            print("> "+cmd)
            print("< "+data_in)
            print("")
        return data_in

    # Status returns 9 arguments. Internal position counter, PI status and movement status
    def parse_status(self,status_string):
        temp = status_string.split(",")
        ret = []
        for t in temp:
            ret.append(int(t.strip()))
        return ret

    def wait_homing(self,ser, initial_status, axis):
        for i in range(10000):
            status_str = self.send_command(ser, "!1")
            status = self.parse_status(status_str)
            #print(status[axis])
            #print(status)
            time.sleep(0.01)
            if initial_status != status[axis]:
                break
        time.sleep(0.1)
    
scf4_tools = SCF4()

def focus_to_pos(focus):
    lowest_pos = 38000
    highest_pos = 29001
    pos = focus * (highest_pos - lowest_pos) + lowest_pos
    return pos

def pos_to_focus(pos):
    lowest_pos = 38000
    highest_pos = 29001
    diff = highest_pos - lowest_pos
    focus = (pos - lowest_pos) / diff
    return focus

def main():
    curr_zoom = 0
    curr_focus = 0
    conn = db.get_connection()
    camera_params = db.Commands(conn)

    c = None
    ser = serial.Serial()
    c = Camera.Cam(preview=False)
    lowest_pos = 0
    highest_pos = 0

    # def setup():
    ser.port = '/dev/ttyACM0'             # Controller com port
    ser.baudrate = 115200           # BAUD rate when connected over CDC USB is not important
    ser.timeout = 5                 # max timeout to wait for command response

    print("Open COM port:", ser.port)
    ser.open()
    ser.flushInput()
    ser.flushOutput()
    print("Starting cam")
    c.start()
    print("Waiting for camera")
    while c.fps == 0:
        time.sleep(0.1) # should be implemented with queue/signals but good enough for testing
    print("Cam is operational")
    print("Read controller version strings")
    scf4_tools.send_command(ser, "$S", echo=True)
    print("Initialize controller")
    scf4_tools.send_command(ser, "$B2", echo=True)
    print("# Set motion to forced mode")
    scf4_tools.send_command(ser, "M231 A", echo=True)
    print("Set stepping mode")
    scf4_tools.send_command(ser, "M243 C6", echo=True)
    print("Set normal move")
    scf4_tools.send_command(ser, 'M230', echo=True)
    print("Set to rel movement mode")
    scf4_tools.send_command(ser, 'G91', echo=True)
    print("Energize PI leds")
    scf4_tools.send_command(ser, "M238", echo=True)
    print("Set motor power")
    scf4_tools.send_command(ser, "M234 A190 B190 C190 D90", echo=True)
    print("Set motor sleep power")
    scf4_tools.send_command(ser, "M235 A120 B120 C120", echo=True)
    print("Set motor drive speed")
    scf4_tools.send_command(ser, "M240 A600 B600 C600", echo=True)
    print("Set PI low/high detection voltage")
    scf4_tools.send_command(ser, "M232 A400 B400 C400 E700 F700 G700", echo=True)
    print("Filter = VIS")
    scf4_tools.send_command(ser, "M7", echo=True)
    print("Get bus voltage")
    adc = scf4_tools.send_command(ser, "M247", echo=True)
    adc = (float)(adc.split("=")[1])
    volts = adc/4096.0*3.3/0.5
    print("  V(bus)=", round(volts, 2), "V")
    print()
    print("Home axis A")
    print("Get status")
    status_str = scf4_tools.send_command(ser, "!1")
    status = scf4_tools.parse_status(status_str)
    print(status_str)
    if status[3] == 0:
        print("Dir 1")
        scf4_tools.send_command(ser, "G91")
        scf4_tools.send_command(ser, "M231 A")          # Set motion to forced mode
        scf4_tools.send_command(ser, "G0 A+100")
        scf4_tools.wait_homing(ser, status[CHA_PI], CHA_PI)
    else:
        print("Dir 2")
        scf4_tools.send_command(ser, "G91")
        scf4_tools.send_command(ser, "M231 A")          # Set motion to forced mode
        scf4_tools.send_command(ser, "G0 A-100")
        scf4_tools.wait_homing(ser, status[CHA_PI], CHA_PI)     # Wait until homing is over
    print("Motor normal mode")
    scf4_tools.send_command(ser, "M230 A")          # Set motion back to normal mode
    scf4_tools.send_command(ser, "G0 A-200")
    scf4_tools.wait_homing(ser, 1, CHA_MOVE) # Wait until homing is over
    print("Motor forced mode")
    scf4_tools.send_command(ser, "G91")
    scf4_tools.send_command(ser, "M231 A")          # Set motion to forced mode
    scf4_tools.send_command(ser, "G0 A+100")
    scf4_tools.wait_homing(ser, status[CHA_PI], CHA_PI)     # Wait until homing is over
    print("Set current coordinate as middle")
    scf4_tools.send_command(ser, "G92 A32000")          # set current coordinate to 32000
    scf4_tools.send_command(ser, "M230 A")          # Set motion back to normal mode
    scf4_tools.send_command(ser, "G90")
    print()
    print("Home axis B")
    print("Get status")
    status_str = scf4_tools.send_command(ser, "!1")
    status = scf4_tools.parse_status(status_str)
    print(status_str)
    if status[4] == 0:
        print("Dir 1")
        scf4_tools.send_command(ser, "G91")
        scf4_tools.send_command(ser, "M231 B")          # Set motion to forced mode
        scf4_tools.send_command(ser, "G0 B+100")
        scf4_tools.wait_homing(ser, status[CHB_PI], CHB_PI)
    else:
        print("Dir 2")
        scf4_tools.send_command(ser, "G91")
        scf4_tools.send_command(ser, "M231 B")          # Set motion to forced mode
        scf4_tools.send_command(ser, "G0 B-100")
        scf4_tools.wait_homing(ser, status[CHB_PI], CHB_PI)     # Wait until homing is over
    print("Motor normal mode")
    scf4_tools.send_command(ser, "M230 B")          # Set motion back to normal mode
    scf4_tools.send_command(ser, "G0 B-200")
    scf4_tools.wait_homing(ser, 1, CHB_MOVE)        # Wait until homing is over
    print("Motor forced mode")
    scf4_tools.send_command(ser, "G91")
    scf4_tools.send_command(ser, "M231 B")          # Set motion to forced mode
    scf4_tools.send_command(ser, "G0 B+100")
    scf4_tools.wait_homing(ser, status[CHB_PI], CHB_PI)     # Wait until homing is over
    print("Set current coordinate as middle")
    scf4_tools.send_command(ser, "G92 B32000")          # set current coordinate to 32000
    scf4_tools.send_command(ser, "M230 B")          # Set motion back to normal mode
    scf4_tools.send_command(ser, "G90")
    print("Get status")
    status_str = scf4_tools.send_command(ser, "!1")
    status = scf4_tools.parse_status(status_str)
    print(status_str)
    # A:17850...38650
    # B:28500...38000
    print()
    print("Move to zoom preset position")
    #scf4_tools.send_command(ser, "G0 A17850")
    scf4_tools.send_command(ser, "G0 A38650")
    #scf4_tools.send_command(ser, "G0 A100")
    
    scf4_tools.wait_homing(ser, 1, CHA_MOVE)        # Wait until homing is over
    scf4_tools.send_command(ser, "G0 B35020")
    scf4_tools.wait_homing(ser, 1, CHB_MOVE)        # Wait until homing is over
    print("Done")
    time.sleep(1)
    lowest_pos = 38000
    highest_pos = 29001 
    focus_table = []
    scf4_tools.send_command(ser, "G0 B29000")
    scf4_tools.wait_homing(ser, 1, CHB_MOVE)
    scf4_tools.send_command(ser, "M240 B5000")    # make motor move slower
    scf4_tools.send_command(ser, "G0 B38000")
    time.sleep(0.1)
    print("highest pos: " + str(highest_pos))
    print("lowest pos: " + str(lowest_pos))
    focus_peak_val = -1
    focus_peak_pos = -1
    for f in focus_table:
        if f[1] > focus_peak_val:
            focus_peak_pos= f[0]
            focus_peak_val = f[1]
    print()
    print("Best focus pos:", focus_peak_pos)
    print("Best focus val:", focus_peak_val)
    # experimental offset value. due to camera frame and motor mismatch
    focus_peak_pos -= 100
    scf4_tools.send_command(ser, "M240 B600")    # make motor move normal
    scf4_tools.send_command(ser, "G0 B"+str(focus_peak_pos))
    scf4_tools.wait_homing(ser, 1, CHB_MOVE)

    try:
        while True:
            time.sleep(0.1)
            
            if curr_zoom != camera_params.camera_zoom_value:
                camValue = camera_params.camera_zoom_value * (58000 - 17850) + 17850
                curr_zoom = camera_params.camera_zoom_value
                com = "G0 A" + str(camValue)
                scf4_tools.send_command(ser, com)
                
            if curr_focus != camera_params.focus:
                focusValue = camera_params.focus * (highest_pos - lowest_pos) + lowest_pos
                curr_focus = camera_params.focus
                com = "G0 B" + str(focusValue)
                scf4_tools.send_command(ser, com)

            if camera_params.auto_focus: # Foco automático da camera: Vai iterar uma série de difernetes valores de foco e utilizar o que obtiver um melhor resultado (Laplacian)
                camera_params.auto_focus = False
                focus_table = []
                print("A TENTAR AUTO FOCAR")
            
                #c.set_cam_text("Moving to MIN focus point")
                scf4_tools.send_command(ser, "G0 B29000")
                scf4_tools.wait_homing(ser, 1, CHB_MOVE)

                scf4_tools.send_command(ser, "M240 B5000")    # make motor move slower
                #c.set_cam_text("Searching for best focus position (full range)")
                scf4_tools.send_command(ser, "G0 B38000")

                c.focus_traker_enabled = True
                for i in range(10000):
                    status_str = scf4_tools.send_command(ser, "!1")
                    status = scf4_tools.parse_status(status_str)
                    #print(status[1], c.focus_val)
                    focus_table.append([status[1], c.focus_val])
                    time.sleep(0.01)
                    if 1 != status[CHB_MOVE]:
                        break
                c.focus_traker_enabled = False
                time.sleep(0.1)

                focus_peak_val = -1
                focus_peak_pos = -1

                for f in focus_table:
                    if f[1] > focus_peak_val:
                        focus_peak_pos= f[0]
                        focus_peak_val = f[1]
                
                print()
                print("Best focus pos:", pos_to_focus(focus_peak_pos))
                print("Best focus val:", focus_peak_val)
                
                # experimental offset value. due to camera frame and motor mismatch
                focus_peak_pos -= 100
                scf4_tools.send_command(ser, "M240 B600")  # make motor move normal
                #c.set_cam_text("Moving to best position")
                scf4_tools.send_command(ser, "G0 B"+str(focus_peak_pos))
                scf4_tools.wait_homing(ser, 1, CHB_MOVE)
                curr_focus = pos_to_focus(focus_peak_pos)
                camera_params.focus = curr_focus

    except KeyboardInterrupt:
        pass
    print("Stopping camera")
    c.stop()


if __name__ == '__main__':
    main({ "stop": False })