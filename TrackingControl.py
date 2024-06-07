import db
import math
import utils
import time
import numpy as np
import PanTilt
import GPIO
from utils import Location
from collections import deque


conn = db.get_connection()
gps_points = db.GPSData(conn)
commands = db.Commands(conn)
cam_state = db.CameraState(conn)

PTController = PanTilt.PanTiltController()
pins = GPIO.MyGPIO()

def normalize_angle(angle):
    return (angle + 180) % 360 - 180

def latlon_to_meters(lat_diff, lon_diff, latitude):
    lat_meters = lat_diff * 111000
    lon_meters = lon_diff * 111000 * math.cos(math.radians(latitude))
    return lat_meters, lon_meters

def gpsDistance(lat1, lon1, lat2, lon2):
	lat1, lon1, lat2, lon2, = map(math.radians, [lat1, lon1, lat2, lon2])
	dlat = lat2 - lat1
	dlon = lon2 - lon1
	a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat1) * math.sin(dlon/2) **2
	c= 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
	distance = 6371 * c
	return distance

#Used for pan speed calculation
last_read = 0
read_time = 0
pan_speed = 0
old_rotation = 0
rotation = 0
servo_rotation = 0
tilt = 0
first_prediction = True
speed_time = 0


def calibrationCoordsCal():
    '''
    Saves 25 gps samples and returns the average lat and lon
    '''
    calibrationBufferLAT = np.array([])    # [lats]
    calibrationBufferLON = np.array([])    # [lats]
    while len(calibrationBufferLAT) < 15:  # 5 seconds at 3 Hz to fill the buffer with samples
        time.sleep(0.01)
        if gps_points.new_reading:         # For every new_reading that comes in
            gps_points.new_reading = False
            calibrationBufferLAT = np.append(calibrationBufferLAT, gps_points.latest_gps_data['latitude'])
            calibrationBufferLON = np.append(calibrationBufferLON, gps_points.latest_gps_data['longitude'])
        
    avg_lat = np.average(calibrationBufferLAT)
    avg_lon = np.average(calibrationBufferLON)
    print("Camera Calibration Complete")

    return avg_lat, avg_lon
        
        
def panCalculations():
    locationToTrack = Location(gps_points.latest_gps_data['latitude'], gps_points.latest_gps_data['longitude'])
    locationOrigin = Location(gps_points.camera_origin['latitude'], gps_points.camera_origin['longitude'])
    
    rotation = -np.degrees(utils.get_angle_between_locations(locationOrigin, locationToTrack) - gps_points.camera_heading_angle)
    rotation = normalize_angle(rotation)
    rotation = round(rotation, 1) # Round to 1 decimal place
    return rotation

def tiltCalculations():
    trackDistX = 1000 * gpsDistance(gps_points.camera_origin['latitude'], gps_points.camera_origin['longitude'],
                                    gps_points.latest_gps_data['latitude'], gps_points.latest_gps_data['longitude'])
    trackDistY = gps_points.camera_vertical_distance
    tiltAngle = np.degrees(math.atan2(trackDistX, trackDistY)) - 90
    tiltAngle = round(tiltAngle, 1) # Round to 1 decimal place
    return tiltAngle

def calculate_pan_speed(current_rotation, last_rotation, time_interval):
    # Calculate the difference in angles and divide by the time interval
    angle_difference = normalize_angle(current_rotation - last_rotation)
    return angle_difference / time_interval
        
        
def main(d):
    last_rotation = None
    last_time = None
    speed_threshold = 1  # Define a threshold for significant speed
    stop_duration_threshold = 5  # Number of seconds to stop recording after panning stops
    stop_timer = 0
    speed_buffer = deque(maxlen=8)  # Buffer to store the last 8 speed measurements

    
    while True:
        time.sleep(0.01)
        
        if commands.camera_calibrate_origin:        # Calibrate the camera origin coordinate
            print("CALIBRATE ORIGIN")
            commands.camera_calibrate_origin = False
            avg_lat, avg_lon = calibrationCoordsCal()
            gps_points.camera_origin['latitude'] = avg_lat
            gps_points.camera_origin['longitude'] = avg_lon
            
        elif commands.camera_calibrate_heading:       # Calibrate the camera heading coordinate
            commands.camera_calibrate_heading = False
            avg_lat, avg_lon = calibrationCoordsCal()
            gps_points.camera_heading_coords['latitude'] = avg_lat
            gps_points.camera_heading_coords['longitude'] = avg_lon
            cam_position = Location(gps_points.camera_origin['latitude'], gps_points.camera_origin['longitude'])
            cam_heading = Location(gps_points.camera_heading_coords['latitude'], gps_points.camera_heading_coords['longitude'])
            gps_points.camera_heading_angle = utils.get_angle_between_locations(cam_position, cam_heading)
            
            print("Camera Heading Calibration Complete")
        
        if commands.tracking_enabled:
            if gps_points.new_reading:
                gps_points.new_reading = False
                panAngle = panCalculations()
                tiltAngle = tiltCalculations()
                PTController.setPanServoAngle(panAngle)
                PTController.setTiltServoAngle(tiltAngle + gps_points.tilt_offset)
                print(f"Pan: {panAngle} ; Tilt: {tiltAngle + gps_points.tilt_offset}")
        
        

                if last_rotation is not None and last_time is not None and cam_state.enable_auto_recording:
                    # Calculate the time interval
                    current_time = time.time()
                    time_interval = current_time - last_time
                    
                    # Calculate pan speed
                    pan_speed = calculate_pan_speed(panAngle, last_rotation, time_interval)
                    speed_buffer.append(pan_speed)
                    if len(speed_buffer) > 1:
                        average_speed = sum(speed_buffer) / len(speed_buffer)
                    else:
                        average_speed = pan_speed
                    
                    # Check if speed exceeds the threshold
                    if average_speed > speed_threshold:
                        # Start recording if not already started
                        if not cam_state.start_recording:
                            cam_state.start_recording = True
                            print("Start recording due to increased pan speed")
                        stop_timer = 0  # Reset stop timer
                    elif cam_state.start_recording:
                        # Increment stop timer if already recording
                        stop_timer += time_interval
                        if stop_timer >= stop_duration_threshold:
                            cam_state.start_recording = False
                            print("Stop recording due to lack of pan movement")
                    
                    # Update last rotation and time
                    last_rotation = panAngle
                    last_time = current_time
        
        else:
            PTController.setPanServoAngle(0)
            PTController.setTiltServoAngle(-10)
            if cam_state.start_recording:
                cam_state.start_recording = False
                print("Stop recording due to tracking disabled")
            
    
            
                
if __name__ == "__main__":
    main({"stop": False})