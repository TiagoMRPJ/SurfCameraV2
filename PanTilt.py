'''
Herkulex DRS 0201 Servo Drivers
'''

import pyherkulex as hx
import time
from GPIO import MyGPIO

class PanTiltController:
    def __init__(self):
        
        IO = MyGPIO()
        IO.SetHerkulexEnable(False) # False IS ON
        
        self.serial = hx.serial()
        broadcast_srv = hx.Servo(serial = self.serial)
        broadcast_srv.led = hx.LED_WHITE
        
        self.panServo = hx.Servo(1)
        self.tiltServo = hx.Servo(2)    
        
        self.panServo.ack_policy = hx.ACK_ALL # This allows the Servos to reply to all types of requests
        self.tiltServo.ack_policy = hx.ACK_ALL
        
        self.panServo.position_gain_p = 10
        self.tiltServo.position_gain_p = 10

        #self.tiltServo.position_gain_p = 40
        #self.tiltServo.position_gain_d = 20
        #self.tiltServo.position_gain_i = 0
        
        self.maxPanAngle = 80
        
        self.maxTiltAngle = 120
        self.minTiltAngle = 55
          
                
        # Check first if there is any error status
        if self.panServo.status[0]:
            # Print status and raise an exception
            self.panServo.print_status()
            raise Exception('Please check Pan Herkulex Servo')
        if self.tiltServo.status[0]:
            self.tiltServo.print_status()
            raise Exception('Please check Tilt Herkulex Servo')
        
        self.panServo.mode = hx.MODE_CONTROL
        self.tiltServo.mode = hx.MODE_CONTROL
        
        #self.setPanServoAngle(angle = 0, time = 3)
        #setTiltServoAngle(angle = 0, time = 3)
        
    def setPanServoAngle(self, angle=0, time=False):
        if abs(angle) > self.maxPanAngle:
            print("Maximum pan angle exceeded")
            return 
        
        if time: # IF time is provided, just update and return
            self.panServo.control_angle(angle*2, time)
            return
        
        # Use angle difference to calculate best time
        if (self.panServo.angle * angle) > 0: # If Both have same sign
            angle_diff = abs(self.panServo.angle - angle*2)
        else: angle_diff = abs(self.panServo.angle) + abs(angle*2) # Different signs
        
        if angle_diff <= 20: 
            time = 0.1
        elif angle_diff <= 45:
            time = 0.3
        elif angle_diff <= 100:
            time = 1
        else:
            time = 1.5
            
        time = 0.1
            
        self.panServo.control_angle(angle*2, time)
            
    def setTiltServoAngle(self, angle=0, time= False):
        '''
        Set the tilt angle of the Camera System. Angle should come in actual system coordinate system, where 0 corresponds to the horizon and 0 to 45º is the actual usable range (from horizon to pointing downwards)
        '''
        # Servo rotation that corresponds to the desired camera rotation
        servo_rotation = (abs(angle) + 82)
        if abs(servo_rotation) > self.maxTiltAngle or abs(servo_rotation) < self.minTiltAngle:
            print("Tilt Angle out of bounds")
            return
        if not time:
            time = 0.1
        self.tiltServo.control_angle(servo_rotation, time)
        
'''      
def main(d):
    s = PanTiltController()
    while True:
        time.sleep(0.05)
'''
     
        
if __name__ == "__main__":
    s = PanTiltController()
    start = time.time()
    s.setTiltServoAngle(angle = 0)
    print(time.time()-start)
    
