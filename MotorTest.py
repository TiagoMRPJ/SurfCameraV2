"""
Moves the camera around the maximum positions
"""

import pyherkulex as hx
import time

from PanTilt import PanTiltController as pt


def main():
    ptcontroller = pt()

    print("Testing Tilt ....")
    ptcontroller.setTiltServoAngle(30, 5)
    time.sleep(2)
    ptcontroller.setTiltServoAngle(0,5)
    time.sleep(2)

    print("Testing Pan ....")
    ptcontroller.setPanServoAngle(79,5)
    time.sleep(2)
    ptcontroller.setPanServoAngle(-79,10)
    time.sleep(1.5)
    ptcontroller.setPanServoAngle(0,5)


if __name__ == "__main__":
    main()

    