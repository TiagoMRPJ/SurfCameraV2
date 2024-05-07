import db
import math
import utils
import time
import numpy as np
import PanTilt
from utils import Location

conn = db.get_connection()
gps_points = db.GPSData(conn)
commands = db.Commands(conn)

PTController = PanTilt.PanTiltController()


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

class TrackingController:
    def __init__(self):
        self.calibrationBuffer = np.array([], [], np.float64)
    

def calibrationCoordsCal():
    '''
    Saves 25 gps samples and returns the average lat and lon
    '''
    calibrationBuffer = np.array([], [])    #[lats], [lons]
    while len(calibrationBuffer[0]) < 25:   # 5 seconds at 5 Hz to fill the buffer with samples
        time.sleep(0.01)
        if gps_points.new_reading:          # For every new_reading that comes in
            gps_points.new_reading = False
            np.append(calibrationBuffer[0], gps_points.latest_gps_data['latitude'])
            np.append(calibrationBuffer[1], gps_points.latest_gps_data['longitude'])
        
    avg_lat = np.average(calibrationBuffer[0])
    avg_lon = np.average(calibrationBuffer[1])
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
        
        
def main(d):
    
    while True:
        time.sleep(0.01)
        
        if commands.camera_calibrate_origin:        # Calibrate the camera origin coordinate
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
        
        if commands.tracking_enabled:
            if gps_points.new_reading:
                gps_points.new_reading = False
                panAngle = panCalculations()
                tiltAngle = tiltCalculations()
                PTController.setPanServoAngle(panAngle)
                PTController.setTiltServoAngle(tiltAngle + gps_points.tilt_offset)
        else:
            PTController.setPanServoAngle(0)
            PTController.setTiltServoAngle(10)
                