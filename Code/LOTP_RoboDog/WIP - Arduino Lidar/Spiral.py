import graph
import math
import time
import serial

graph.setup_screen((1000,1000),4,4)
ard = serial.Serial("COM7", baudrate = 115200, timeout = 1)
graph.color="red"

def check_prime(n):
    return all(n%j for j in range(2, int(n**0.5)+1)) and n>1

def sin(deg):
    deg%=360
    if(deg<0): deg+=360
    if(deg>90): deg=180-deg
    elif(deg>180): deg+=180        
    return(math.sin(math.radians(deg)))

def cos(deg):
    deg%=360
    if(deg<0): deg+=360
    if(deg==90 or deg==270): return(0)
    elif(deg>180): deg=180-(deg-180)      
    return(math.cos(math.radians(deg)))

def main(prime):
	graph.clear_screen()
	map = ard.readline().decode("ascii").split(" ")
	if(map[0] != "0" and len(map) != 1):
		print(map)
		for i in range(0, len(map)):
			if(map[i] == ""):
				map[i] == 0
		for i in range(0, 30):
			p = 12
			if(map[i] != '0'):
				graph.draw_brush((-cos(i * p)*int(map[i]), sin(i * p)*int(map[i])), 7)
		graph.update_screen('lines')
		time.sleep(0.1)

while(1):
	main(0)