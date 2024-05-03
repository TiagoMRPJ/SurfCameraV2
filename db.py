import redis
import pickle
import json

def get_connection():
    return redis.Redis()


class RedisClient:

    def __init__(self, connection):
        """Initialize client."""
        self.r = connection

    def set(self, key, value, **kwargs):
        """Store a value in Redis."""
        return self.r.set(key, pickle.dumps(value), **kwargs)

    def set_initial(self, key, value):
        """Store a value in Redis."""
        if not self.get(key):
            self.set(key, value)

    def get(self, key):
        """Retrieve a value from Redis."""
        val = self.r.get(key)
        if val:
            return pickle.loads(val)
        return None

    def dump(self, keys, filename):
        data = {}
        for k in keys:
            data[k] = self.get(k)
        print("storing configuration: %s" % json.dumps(data, indent=2))
        with open(filename, "w") as fp:
            json.dump(data, fp, indent=2)

    def load(self, filename):
        data = {}
        with open(filename) as fp:
            data = json.load(fp)
        print("configuration loaded: %s" % json.dumps(data, indent=2))
        for k in data:
            self.set(k, data[k])
            

class GPSData:
    def  __init__(self, connection):
        self.client = RedisClient(connection)
        self.client.set_initial("camera_origin", { "latitude": 0, "longitude": 0 })          # Coordinates of the camera's location -> Calibrate to change this
        self.client.set_initial("camera_heading_coords", { "latitude": 0, "longitude": 0})   # Coordinates of camera's 0º heading -> Calibrate to change this
        self.client.set_initial("latest_gps_data", { "latitude": 0, "longitude": 0})         # Latest coordinates received from the Tracker

        self.client.set_initial("gps_fix", False)                                            # Flag to indicate th
        
    @property
    def camera_origin(self):
        return self.client.get("camera_origin")

    @camera_origin.setter
    def camera_origin(self, value):
        self.client.set("camera_origin", value)
        
    @property
    def camera_heading(self):
        return self.client.get("camera_heading")

    @camera_heading.setter
    def camera_heading(self, value):
        self.client.set("camera_heading", value)
        
    @property
    def latest_gps_data(self):
        return self.client.get("latest_gps_data")

    @latest_gps_data.setter
    def latest_gps_data(self, value):
        self.client.set("latest_gps_data", value)
        
    @property
    def gps_fix(self):
        return self.client.get("gps_fix")

    @gps_fix.setter
    def gps_fix(self, value):
        self.client.set("gps_fix", value)

class Commands:
    def  __init__(self, connection):
        self.client = RedisClient(connection)
        self.client.set_initial("camera_calibrate_origin", False)    # Flag utilized to start the origin calibration process
        self.client.set_initial("camera_calibrate_heading", False)   # Flag utilized to start the heading calibration process
        self.client.set_initial("camera_autofocus", False)           # Flag utilized to auto focus the camera
        self.client.set_initial("camera_zoom_value", 50)             # Zoom value to apply between 0 and 100
        
    @property
    def camera_calibrate_origin(self):
        return self.client.get("camera_calibrate_origin")

    @camera_calibrate_origin.setter
    def camera_calibrate_origin(self, value):
        self.client.set("camera_calibrate_origin", value)
        
    @property
    def camera_calibrate_heading(self):
        return self.client.get("camera_calibrate_heading")

    @camera_calibrate_heading.setter
    def camera_calibrate_heading(self, value):
        self.client.set("camera_calibrate_heading", value)
        
        
class CameraState:
    def __init__(self, connection):
        self.client = RedisClient(connection)
        self.client.set_initial("is_recording", False)

    @property
    def image(self):
        return self.client.get("state_image")

    @image.setter
    def image(self, v):
        self.client.set("state_image", v)

    @property
    def start_recording(self):
        return self.client.get("start_recording")

    @start_recording.setter
    def start_recording(self, v):
        self.client.set("start_recording", v)

    @property
    def image_focus(self):
        return self.client.get("state_image_focus")

    @image_focus.setter
    def image_focus(self, v):
        self.client.set("state_image_focus", v)