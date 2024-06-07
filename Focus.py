'''
Contains the Class that will handle Zoom and Focus commands.
The class worker is ran as a separate thread on the Camera module, which is called as a process
'''


import serial
import time
import numpy as np
from utils import normalize
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

def send_command(ser, cmd, echo=False):
    ser.write(bytes(cmd+"\n", 'utf8'))
    data_in = ser.readline().decode('utf-8').strip()
    if echo:
        print("> "+cmd)
        print("< "+data_in)
        print("")
    return data_in

# Status returns 9 arguments. Internal position counter, PI status and movement status
def parse_status(status_string):
    temp = status_string.split(",")
    ret = []
    for t in temp:
        ret.append(int(t.strip()))
    return ret

def wait_homing(ser, initial_status, axis):
    for i in range(10000):
        status_str = send_command(ser, "!1")
        status = parse_status(status_str)
        time.sleep(0.01)
        if initial_status != status[axis]:
            break
    time.sleep(0.1)



class ZoomFocus:
    def __init__(self):
        self.conn = db.get_connection()
        self.commands = db.Commands(self.conn)
        self.camera_state = db.CameraState(self.conn)
                
        self.curr_zoom = self.commands.camera_zoom_value
        self.ser = serial.Serial()
        
        SERIAL_PORT = ""
        while SERIAL_PORT == "":
            ports = list(serial.tools.list_ports.comports())
            for p in ports:
                if p.description and "Kurokesu" in p.description:
                    SERIAL_PORT = p.device
                    break
        
        self.ser.port = SERIAL_PORT
        self.ser.baudrate = 115200
        self.ser.timeout = 5
        self.ser.open()
        self.ser.flushInput()
        self.ser.flushOutput()
        self.initialSetup()
        
    def initialSetup(self):
        send_command(self.ser, "$S", echo=False)
        send_command(self.ser, "$B2", echo=False)
        send_command(self.ser, "M231 A", echo=False)
        send_command(self.ser, "M243 C6", echo=False)
        send_command(self.ser, 'M230', echo=False)
        send_command(self.ser, 'G91', echo=False)
        send_command(self.ser, "M238", echo=False)
        send_command(self.ser, "M234 A190 B190 C190 D90", echo=False)
        send_command(self.ser, "M235 A120 B120 C120", echo=False)
        send_command(self.ser, "M240 A600 B600 C600", echo=False)
        send_command(self.ser, "M232 A400 B400 C400 E700 F700 G700", echo=False)
        send_command(self.ser, "M7", echo=False)
        adc = send_command(self.ser, "M247", echo=False)
        adc = (float)(adc.split("=")[1])
        volts = adc/4096.0*3.3/0.5
        status_str = send_command(self.ser, "!1")
        status = parse_status(status_str)
        if status[3] == 0:
            send_command(self.ser, "G91")
            send_command(self.ser, "M231 A")          # Set motion to forced mode
            send_command(self.ser, "G0 A+100")
            wait_homing(self.ser, status[CHA_PI], CHA_PI)
        else:
            send_command(self.ser, "G91")
            send_command(self.ser, "M231 A")          # Set motion to forced mode
            send_command(self.ser, "G0 A-100")
            wait_homing(self.ser, status[CHA_PI], CHA_PI)     # Wait until homing is over
        send_command(self.ser, "M230 A")          # Set motion back to normal mode
        send_command(self.ser, "G0 A-200")
        wait_homing(self.ser, 1, CHA_MOVE) # Wait until homing is over
        send_command(self.ser, "G91")
        send_command(self.ser, "M231 A")          # Set motion to forced mode
        send_command(self.ser, "G0 A+100")
        wait_homing(self.ser, status[CHA_PI], CHA_PI)     # Wait until homing is over
        send_command(self.ser, "G92 A32000")          # set current coordinate to 32000
        send_command(self.ser, "M230 A")          # Set motion back to normal mode
        send_command(self.ser, "G90")
        status_str = send_command(self.ser, "!1")
        status = parse_status(status_str)
        if status[4] == 0:
            send_command(self.ser, "G91")
            send_command(self.ser, "M231 B")          # Set motion to forced mode
            send_command(self.ser, "G0 B+100")
            wait_homing(self.ser, status[CHB_PI], CHB_PI)
        else:
            send_command(self.ser, "G91")
            send_command(self.ser, "M231 B")          # Set motion to forced mode
            send_command(self.ser, "G0 B-100")
            wait_homing(self.ser, status[CHB_PI], CHB_PI)     # Wait until homing is over
        send_command(self.ser, "M230 B")          # Set motion back to normal mode
        send_command(self.ser, "G0 B-200")
        wait_homing(self.ser, 1, CHB_MOVE)        # Wait until homing is over
        send_command(self.ser, "G91")
        send_command(self.ser, "M231 B")          # Set motion to forced mode
        send_command(self.ser, "G0 B+100")
        wait_homing(self.ser, status[CHB_PI], CHB_PI)     # Wait until homing is over
        send_command(self.ser, "G92 B32000")          # set current coordinate to 32000
        send_command(self.ser, "M230 B")          # Set motion back to normal mode
        send_command(self.ser, "G90")
        status_str = send_command(self.ser, "!1")
        status = parse_status(status_str)
        # A:17850...38650
        # B:28500...38000
        #send_command(self.ser, "G0 A17850")
        send_command(self.ser, "G0 A38650")
        #send_command(self.ser, "G0 A100")
        
        wait_homing(self.ser, 1, CHA_MOVE)        # Wait until homing is over
        send_command(self.ser, "G0 B35020")
        wait_homing(self.ser, 1, CHB_MOVE)        # Wait until homing is over
        time.sleep(1)
        lowest_pos = 38000
        highest_pos = 29001 
        focus_table = []
        send_command(self.ser, "G0 B29000")
        wait_homing(self.ser, 1, CHB_MOVE)
        send_command(self.ser, "M240 B5000")    # make motor move slower
        send_command(self.ser, "G0 B38000")
        
    def setZoom(self, zoom):
        try:
            zoomVal = 100 - zoom
            camValue = zoomVal/100 * (58000 - 17850) + 17850
            com = "G0 A" + str(camValue)
            send_command(self.ser, com)
        except Exception as e:
            print(f"Error zooming {e}")
        
    def setAutoFocus(self):
        focus_table = []
        # Moving to minimum focus axis val
        send_command(self.ser, "G0 B29000")
        wait_homing(self.ser, 1, CHB_MOVE)
        send_command(self.ser, "M240 B5000")    # make motor move slower
        # Searching for best focus position (full range)
        send_command(self.ser, "G0 B38000")
        
        self.camera_state.focus_tracker = True
        
        for i in range(10000):
            status_str = send_command(self.ser, "!1")
            status = parse_status(status_str)
            focus_table.append([status[1], self.camera_state.image_focus])
            time.sleep(0.01)
            if 1 != status[CHB_MOVE]:
                break
        
        self.camera_state.focus_tracker = False
        focus_peak_val = -1
            
        for f in focus_table:
            if f[1] > focus_peak_val:
                focus_peak_pos= f[0]
                focus_peak_val = f[1]
        
        focus_peak_pos -= 100
        send_command(self.ser, "M240 B600")  # make motor move normal
        # Moving to optimal focus position
        send_command(self.ser, "G0 B"+str(focus_peak_pos))
        wait_homing(self.ser, 1, CHB_MOVE)

    def worker(self):
        while True:
            time.sleep(1)
            
            if self.commands.camera_autofocus:
                self.setAutoFocus()
                self.commands.camera_autofocus = False
                
            if self.commands.camera_apply_zoom_value:
                self.setZoom(self.commands.camera_zoom_value)
                self.commands.camera_apply_zoom_value = False
                
            