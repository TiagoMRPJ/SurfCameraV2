import json
from flask import render_template, Flask, send_from_directory, Response, jsonify, request
from flask_cors import CORS
import time

import db

def main(d):
    conn = db.get_connection()
    gps_points = db.GPSData(conn)
    commands = db.Commands(conn)
    camera_state = db.CameraState(conn)
    
    app = Flask(__name__)

    @app.route("/")
    def index():
        """Video streaming home page."""
        return render_template('index.html')

    @app.route('/start_recording', methods=["POST"])
    def start_recording():
        """Sets the db camera state to start recording"""
        print("---")
        print("Flask Start Recording")
        camera_state.start_recording = True
        return jsonify({ "success": camera_state.start_recording, "message": "OK" })

    @app.route('/stop_recording', methods=["POST"])
    def stop_recording():
        """Sets the db camera state to stop recording"""
        print("Flask Stop Recording")
        camera_state.start_recording = False
        return jsonify({ "success": camera_state.start_recording, "message": "OK" })

    @app.route('/set_autofocus', methods=["POST"])
    def set_autofocus():
        commands.camera_autofocus = True
        print("Flask AutoFocus")
        return jsonify({"success": True, "message": "FOCAR Updated!"})

    @app.route('/update_zoom_value', methods=["POST"])
    def update_zoom_value():
        zoom_val = request.json.get('zoom_value', 0)
        commands.camera_zoom_value = zoom_val
        commands.camera_apply_zoom_value = True
        print("Flask Updating Zoom")
        return jsonify({"success": True, "message": "Values Updated!"})
    
    @app.route('/increment', methods=['POST'])
    def increment():
        gps_points.camera_heading_angle += 0.0174532925
        return jsonify({"success": True, "message": "Values Updated!"})

    @app.route('/decrement', methods=['POST'])
    def decrement():
        gps_points.camera_heading_angle -= 0.0174532925
        return jsonify({"success": True, "message": "Values Updated!"})

    @app.route('/tilt_offset_plus', methods=['POST'])
    def tilt_offset_plus():
        gps_points.tilt_offset += 1
        return jsonify({"success": True, "message": "Values Updated!"})

    @app.route('/tilt_offset_minus', methods=['POST'])
    def tilt_offset_minus():
        gps_points.tilt_offset -= 1
        return jsonify({"success": True, "message": "Values Updated!"})