import serial
import time
import string
import pynmea2

while True:
	ser=serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=1)
	dataout = pynmea2.NMEAStreamReader()
	newdata=ser.readline()
	# print(newdata)
	if '$GPRMC' in str(newdata):
		print(newdata.decode('utf-8'))
		newmsg=pynmea2.parse(newdata.decode('utf-8'))
		lat=newmsg.latitude
		lng=newmsg.longitude
		gps = "Latitude=" + str(lat) + " and Longitude=" + str(lng)
		print(gps)
		#return (lat, lng)
