#!/usr/bin/python3

'''
Hot Tub Control
    Inputs
  T0 = Tub Temperature
  T1 = Circulator Temperature
  T2 = Air Temperature
  HV0 = Tub Heat Demand
  HV1 = High-speed pump demand
  HV2 = Low-speed pump demand

    Outputs
  R0 = High-speed pump
  R1 = Low-speed pump
  R2 = Common
  R3 = Heat request - temporary while single controller

  Remember to modify /etc/inittab & /boot/cmdline.txt to free up serial port on Raspberry Pi
    Remove 'console=...' and 'kgdboc=...' in cmdline.txt
    Remove last line in inittab to eliminate getty for ttyAMA0
'''

import RPi.GPIO as GPIO
import time
import pyserial

 
Relay = (5,  24, 23, 22, 27, 18, 17,  4)
TT    = (21, 20, 26, 16, 19, 13, 12,  6)
HV    = ( 7,  3,  2)
noTsensors = 3
dbUpdateInterval = 600   // Seconds

// Set up Serial Port
serialPort = '/dev/ttyAMA0'
baudRate = 38400
serialTimeout = 1 //sec
ser = serial.Serial(serialPort, baudRate, timeout=serialTimeout)

// State variables
TTstate = []
Tstate = []
HVstate = []
RelayState = []
newTemps = false
tubOccupied = false
now = 0
nextHeatTime = 0
heatCycle = false
lastOccupied = 0

// Constants
heatTime = [(6, 0), (18, 0)]
timeOutPeriod = 20 * 60   // Seconds
tempOvershoot = 4         // degF
    
def getTemps():
    // non-zero return = error

    // Thermistor constants
    R0 = 10000.0
    Beta = 3977.0
    T0 = 298.15
    Rinf = R0 * exp(-Beta/T0)
    Rb = 9920.0
    V0 = 4.87

    key = [0xAA, 0x55]

    ser.flushInput()  // Clear any random data hanging around
    // send read request
    ser.write(key)
    // read raw temperatures
    tempValue = []
    reply = ser.read(2*noTsensors + len(key))  // reply is the key plus the 2-byte temp readings
    if len(reply) != 2*noTsensors + len(key):
        print('read error')
        return 1
    error = 0
    for i in range(len(key)):
        error += (reply[i] != key[i])
    if error:
        print ('read error')
        return 1
    del reply[0:len(key)]   // strip off the key
        
    Tstate = []
    for i in range(noTsensors):
        reading = reply[2*i]*256=reply[2*i+1]
        Rt = Rb * (1024.0/float(reading) - 1)
        t = Beta/log(Rt/Rinf)                       // degK
        Tstate.append ((t-273.15)*9.0/5.0 + 32.0)   // degF
    return 0

def updateNextHeatTime()

    now = time.localTime()
    nextDay = true
    for i in range(len(heatTime)):
        if heatTime[i] > time.localtime()[3:5]: 
            nextDay = false
            break
    if nextDay:
        nextHeatTime = time.mktime((now[0], now[1], now[2], heatTime[0][0], heatTime[i][1], now[5], 1, 1, -1))
    else:
        nextHeatTime = time.mktime((now[0], now[1], now[2], heatTime[i][0], heatTime[i][1], now[5], 1, 1, -1))

def setup():

    GPIO.setmode(GPIO.BCM)
    for R in Relay:
        GPIO.setup(R, GPIO.OUT)
        GPIO.output(R, GPIO.LOW)
        RelayState.append(0)

    for T in TT:
        GPIO.setup(T, GPIO.IN, GPIO.PUD_UP)
        TTstate.append(0)

    for H in HV:
        GPIO.setup(T, GPIO.IN, GPIO.PUD_UP)
        HVstate.append(0)

    getTemps()

    updateNextHeatTime
    

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
    
    newTemps =  newTemps or !getTemps()

def updateRelays():
    // Determine tub occupancy
    now = time.time()
    tubOccupied = HVstate[1] or (now < lastOccupied + timeOutPeriod)
    if HVstate[1]:
        lastOccupied = now

    // Determine heating cycle
    if now > nextHeatTime:
        heatCycle = true
        updateNextHeatTime
        
    if !HVstate[0]:  // no heat demand => up to temperature
        heatCycle = false

    lowSpeedReq = (Tstate[1] > Tstate[0] + tempOvershoot)

    if (heatCycle or tubOccupied) and HVstate[0]:  // Heat Water
        // set R3 to demand heat from furnace
        GPIO.output(Relay[3], true)
        if HVstate[1]:
            // clear R1 to stop low-speed winding
            GPIO.output(Relay[1], false)
            // set R0 to run high-speed motor
            GPIIO.output(Relay[0], true)
        else:
            // clear R0 to stop high-speed winding
            GPIO.output(Relay[0], false)
            // set R1 to run low-speed winding
            GPIIO.output(Relay[1], true)
        // set R2 to connect common 
        GPIO.output(Relay[2], true)
    else:                                         // Non-heated motor operation
        // clear R3 to stop demand from furnace
        GPIO.output(Relay[3], false)
        if tubOccupied:
            if HVstate[1]:
                // clear R1 to stop low-speed winding
                GPIO.output(Relay[1], false)
                // set R0 to run high-speed winding
                GPIO.output(relay[0], true)
                // set R2 to connect common
                GPIIO.output(Relay[2], true)
            elif HVstate[2] or lowSpeedReq:
                // clear R0 to stop high-speed winding
                GPIO.output(Relay[0], false)
                // set R1 to run low-speed winding
                GPIO.output(Relay[1], true)
                // set R2 to connect common
                GPIO.output(Relay[2], true)
            else:
                // Clear R2 to disconnect common
                // Clear R0 to stop high-speed winding
                // Clear R1 to stop low-speed winding
                GPIO.output(Relay[0:3], false)

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
