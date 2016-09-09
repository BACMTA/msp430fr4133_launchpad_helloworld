portname = 'COM37'

import serial
import time

with serial.Serial(portname,9600) as port:

    cmd = time.strftime('t%H%M%S\n').encode('ascii')
    port.write(cmd)
