#!/usr/bin/env python

import wiringpi2

setupres = wiringpi2.wiringPiSPISetup(0, 5000)

while True:
    command = raw_input("Data to write?")
    written = wiringpi2.wiringPiSPIDataRW(0, command+'\n')
