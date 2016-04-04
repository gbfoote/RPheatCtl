#!/usr/bin/python3
'''Calculate amount of oil in 2000 Gal Tank'''

import sys, math

def main():
    empty = 4.2
    full = 19.93
    capacity = 2000
    pi = 3.14159
    argv = sys.argv #Strip off first result which is the program name
    print (argv)
    reading = float(argv[1])
    h = 2.0 * (reading - empty)/(full - empty)  # actually h/R
    vol = capacity / pi * (math.acos(1-h) - h * (1-h) * math.sqrt(2/h -1))
    print ('Tank contains {0:6.0f} gallons'.format(vol))

if __name__== '__main__':
    main()
