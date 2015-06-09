#!/usr/bin/python3

import RPi.GPIO as GPIO
import time

 
Relay = (5,  24, 23, 22, 27, 18, 17,  4)
TT    = (21, 20, 26, 16, 19, 13, 12,  6)
HV    = ( 7,  3,  2)
dbUpdateInterval = 600   // Seconds
    

def setup():

    GPIO.setmode(GPIO.BCM)
    RelayState = 0
    for R in Relay:
        GPIO.setup(R, GPIO.OUT)

    TTstate = 0
    for T in TT:
        GPIO.setup(T, GPIO.IN, GPIO.PUD_UP)
    
    HVstate = 0
    for H in HV:
        GPIO.setup(T, GPIO.IN, GPIO.PUD_UP)

    nextUpdateDB = time.time()

def readSensors():
    for T in TT:
        if GPIO.input(T):
            TTstateNew = TTstate | 1<<T
        else:
            TTstateNew = TTstate & ~(1<<T)
            
    for H in HV:
        if GPIO.input(H):
            HVstateNew = HVstate | 1<<H
        else:
            HVstateNew = HVstate & ~(1<<H)

    now = time.time()

def updateRelays():
    pass


def updateDb():
    nextUpdateDB = now + dbUpdateInterval


def main():

    while True:
        readSensors()
        updateRelays()
        if now > nextUpdateDb:
            updateDB()
        
        



setup()

try:
    main()
except KeyboardInterrupt:
    GPIO.cleanup()
