import time
import os
import RPi.GPIO as GPIO


class MyGPIO:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.first_led_pin = 6
        self.second_led_pin = 19
        self.herkulex_enable_pin = 5
        self.shutdown_pin = 17
        GPIO.setup(self.first_led_pin, GPIO.OUT)
        GPIO.setup(self.second_led_pin, GPIO.OUT)
        GPIO.setup(self.herkulex_enable_pin, GPIO.OUT)
        GPIO.setup(self.shutdown_pin, GPIO.IN)
        
    def SetFirstLED(self, state=True):
        GPIO.output(self.first_led_pin, state)
    
    def SetSecondLED(self, state=True):
        GPIO.output(self.second_led_pin, state)
        
    def SetHerkulexEnable(self, state=False): # False is ON
        GPIO.output(self.herkulex_enable_pin, state)

if __name__ == "__main__":
    g = MyGPIO()
    g.SetFirstLED(False)
    g.SetSecondLED(False)
    g.SetHerkulexEnable(False)
        