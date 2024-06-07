#!/bin/bash

log_file="/home/surfcamera/Documents/SurfCameraV2/surf_camera.log"
motor_test_retries=10
main_script_retries=15
delay=15

# Function to run MotorTest.py with retries
run_motor_test() {
    for i in $(seq 1 $motor_test_retries); do
        echo "Attempt $i to run MotorTest.py" >> $log_file
        sudo -u surfcamera /usr/bin/python /home/surfcamera/Documents/SurfCameraV2/MotorTest.py >> $log_file 2>&1

        if [ $? -eq 0 ]; then
            echo "MotorTest.py succeeded on attempt $i" >> $log_file
            return 0
        else:
            echo "Attempt $i to run MotorTest.py failed" >> $log_file
            sleep $delay
        fi
    done
    return 1
}

# Function to run main.py with retries
run_main_script() {
    for i in $(seq 1 $main_script_retries); do
        echo "Attempt $i to start main.py script" >> $log_file
        sudo -u surfcamera /usr/bin/python /home/surfcamera/Documents/SurfCameraV2/main.py >> $log_file 2>&1

        if [ $? -eq 0 ]; then
            echo "main.py script started successfully on attempt $i" >> $log_file
            return 0
        else:
            echo "Attempt $i to start main.py script failed" >> $log_file
            sleep $delay
        fi
    done
    return 1
}

# Run MotorTest.py first
run_motor_test
if [ $? -eq 0 ]; then
    # If MotorTest.py succeeds, run main.py
    run_main_script
    if [ $? -eq 0 ]; then
        echo "main.py script completed successfully" >> $log_file
        exit 0
    else
        echo "All attempts to start main.py script failed" >> $log_file
        exit 1
    fi
else
    echo "All attempts to run MotorTest.py failed" >> $log_file
    exit 1
fi