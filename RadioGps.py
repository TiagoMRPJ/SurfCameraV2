import serial
import serial.tools.list_ports
import time
import math
import db
import threading
import queue
import utils

SERIAL_PORT = ""
while SERIAL_PORT == "":
	ports = list(serial.tools.list_ports.comports())
	for p in ports:
		if p.description and "CP2102N" in p.description:
			SERIAL_PORT = p.device
			break

  
ser = serial.Serial(SERIAL_PORT, baudrate=115200)#, timeout= None, xonxoff=False, rtscts=False, dsrdtr=False)
ser.flushInput()
ser.flushOutput()
receiveTime = 0
lastReceiveTime = 0
lastLat = 0
lastLon = 0
distance = 0

newRead = False

class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s

    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)
                
rl = ReadLine(ser)

def receive():
	if ser.in_waiting > 0:
		#print("Serial Working")
		try:
			line = rl.readline().decode().rstrip()
			if len(line.split(',')) != 2:
				pass
			else:
				decode(line)
		except:
			print('.')
   
def decode(line):
	global receiveTime, lastReceiveTime, interval, newRead
	global lastLat, lastLon, lastAlt, distance
	print(line)
	if len(line.split(',')) == 2:
		data = line.split(',')
		lat = float(data[0]) / 10000000
		lon = float(data[1]) / 10000000
		if int(lat) == 38 and int(lon) == -9: 
			lastLat = lat
			lastLon = lon
		print("new read",lastLat, lastLon)
		newRead = True
	else:
		print('ERROR: ',line)
  
def Rx_thread():
    while True:
        receive()
  
def main(d):
	import GPIO
	io = GPIO.MyGPIO()
	io.SetSecondLED(False)
	io.SetFirstLED(False)
	global lastLat, lastLon, newRead, distance
	conn = db.get_connection()
	gps_points = db.GPSData(conn)
	camera_state = db.CameraState(conn)

	success = 0
	start_time = time.time()
		
	rec_Thread = threading.Thread(target=Rx_thread)
	rec_Thread.start()
	
	while True:
		time.sleep(0.01)
		if newRead:
			position = {"latitude": float(lastLat), "longitude": float(lastLon)}
			gps_points.latest_gps_data = position
	
			# Time between readings calculations
			success += 1
			curT = time.time()
			elapT = curT - start_time
			
			if elapT >= 5:
				readsPerSec = success/elapT
				print('Reads per second: ',readsPerSec)
				success = 0
				start_time = time.time()
				if readsPerSec >= 4:
					io.SetSecondLED(True)
				else:
					io.SetSecondLED(False)
     
			gps_points.new_reading = True
			newRead = False
			if camera_state.is_recording:
				with open('gps_data.txt', 'a+') as f:
					f.write('{:.6f},{:.6f},{}\n'.format(lastLat, -lastLon, 'saved'))

    