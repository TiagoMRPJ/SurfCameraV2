import cv2
import os
import serial
import sys
import time
import threading
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

class ZoomFocusController():
    def __init__(self, Camera):
        self.conn = db.get_connection()
        self.camera_params = db.Commands(self.conn)
        
        self.ser = serial.Serial()
        self.ser.port = '/dev/ttyACM0'             # Controller com port
        self.ser.baudrate = 115200           # BAUD rate when connected over CDC USB is not important
        self.ser.timeout = 5                 # max timeout to wait for command response
        
        self.c = Camera.Cam(preview=False)
        lowest_pos = 0
        highest_pos = 0
        
    def setZoom(self):
        camValue = self.camera_params.camera_zoom_value * (58000 - 17850) + 17850
        com = "G0 A" + str(camValue)
        scf4_tools.send_command(self.ser, com)
        
    def autofocus(self):
        self.camera_params.camera_autofocus = False
        focus_table = []
        print("AutoFocusing")
    
        #c.set_cam_text("Moving to MIN focus point")
        scf4_tools.send_command(self.ser, "G0 B29000")
        scf4_tools.wait_homing(self.ser, 1, CHB_MOVE)

        scf4_tools.send_command(self.ser, "M240 B5000")    # make motor move slower
        #c.set_cam_text("Searching for best focus position (full range)")
        scf4_tools.send_command(self.ser, "G0 B38000")

        self.c.focus_traker_enabled = True
        for i in range(10000):
            status_str = scf4_tools.send_command(self.ser, "!1")
            status = scf4_tools.parse_status(status_str)
            focus_table.append([status[1], self.c.focus_val])
            time.sleep(0.01)
            if 1 != status[CHB_MOVE]:
                break
        self.c.focus_traker_enabled = False
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
        scf4_tools.send_command(self.ser, "M240 B600")  # make motor move normal
        #c.set_cam_text("Moving to best position")
        scf4_tools.send_command(self.ser, "G0 B"+str(focus_peak_pos))
        scf4_tools.wait_homing(self.ser, 1, CHB_MOVE)
        curr_focus = pos_to_focus(focus_peak_pos)
        self.camera_params.focus = curr_focus
        
        
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