#!/usr/bin/python3

import RPi.GPIO as GPIO
import time

 
Relay = (5,  24, 23, 22, 27, 18, 17,  4)
TT    = (21, 20, 26, 16, 19, 13, 12,  6)
HV    = ( 7,  3,  2)
    

def setup():

    GPIO.setmode(GPIO.BCM)
    for R in Relay:
        GPIO.setup(R, GPIO.OUT)

    for T in TT:
        GPIO.setup(T, GPIO.IN, GPIO.PUD_UP)
    
    for H in HV:
        GPIO.setup(T, GPIO.IN, GPIO.PUD_UP)


def main():

    while True:

        GPIO.output(Relay[0], GPIO.HIGH)
        time.sleep(3)
        GPIO.output(Relay[0], GPIO.LOW)
        time.sleep(3)

setup()

try:
    main()
except KeyboardInterrupt:
    GPIO.cleanup()
