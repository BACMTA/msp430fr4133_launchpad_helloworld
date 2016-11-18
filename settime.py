portname = 'COM53'

import serial
import time

with serial.Serial(portname,9600) as port:

    cmd = time.strftime('t%H%M%S\ns0600\nd2030\n').encode('ascii')
    port.write(cmd)
