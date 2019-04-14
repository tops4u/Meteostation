#!/usr/bin/python3
import serial
import time
import requests
from time import localtime, strftime

port = serial.Serial("/dev/serial0", baudrate=9600, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, bytesize = serial.EIGHTBITS, timeout=3)
start = [104, 1, 1, 150] 
stop = [104, 1, 2, 149]
read = [104, 1, 4, 147]
diAS = [104, 1, 32, 119]

def main():
    if port.isOpen():
         port.close()
    port.open()
    port.write(diAS)
    port.read(2)
    port.write(start)
    port.read(2)
    time.sleep(30)
    port.write(read)
    head = ord(port.read());
    len = ord(port.read());
    if head == 0x40 and len == 0x05:
         cmd = ord(port.read());
         df1 = ord(port.read());
         df2 = ord(port.read());
         df3 = ord(port.read());
         df4 = ord(port.read());
         CRC = ord(port.read());
         calcCRC = (65536 - head - len - cmd -df1 - df2 - df3 - df4) % 256
         if calcCRC == CRC:
            PM25 = df1 * 256 + df2
            PM10 = df3 * 256 + df4
            print ("PM2.5: ",PM25,"ug/m3")
            payload = {'value':PM25}
            requests.get('http://192.168.85.22:8087/set/hm-rega.0.15075',params=payload)
            print ("PM10: ",PM10,"ug/m3")	
            payload = {'value':PM10}
            requests.get('http://192.168.85.22:8087/set/hm-rega.0.15076',params=payload)
         else:
            print("CRC Missmatch!")
    else:
         print("Invalid or no data!")
    time.sleep(0.5)
    port.write(stop)
    port.read(2)
    port.close()

if __name__=="__main__":
	while ( 1 == 1 ):
		main()
		time.sleep(260)

