from time import sleep
# from drive import RosAriaDriver
import math
import json

# robot=RosAriaDriver('/PIONIER6')

import matplotlib.pyplot as plt
import sys

from lab5 import world_map

scan = []
theta = []
data = list()

outputPath = "./output/"
inputPath = "./datafiles/"


def pol2cart(rho, phi, ang=0):
    if rho == 0:
        return (0, 0)
    else:
        x = rho * math.cos(phi + ang);
        y = rho * math.sin(phi + ang);
    return [x, y]


def cart2pol(x, y):
    r = math.sqrt(x ** 2 + y ** 2)
    phi = math.atan2(y, x)
    return [r, phi]


def readJSON(filename):
    json_file = open(filename)
    data = json.load(json_file)
    return data

def saveJSON(filename, data):
    with open(filename, 'w') as outfile:
        json.dump(data, outfile)




def lab21():
    global scan, robot, theta, data
    data = readJSON(inputPath + "map_boxes_0.json")

    x = np.arange(0, 512)
    theta = (np.pi / 512) * x

    pointsHit = list()

    for iteration in data:
        print iteration['pose']

        pointsHitTemp = list()
        for scanIndex in range(len(iteration['scan'])):
            scanPoint = [ theta[scanIndex], iteration['scan'][scanIndex] ]

            if not np.isinf(scanPoint[1]) and not np.isnan(scanPoint[1]):
                pointsHitTemp.append(pol2cart(scanPoint[0], scanPoint[1]))


        pointsHit.append(pointsHitTemp)

    saveJSON(outputPath + "temp.JSON", pointsHit)

    print "Lenght of sublists in each iteration: "
    print len(pointsHit[0])
    print len(pointsHit[1])
    print len(pointsHit[2])
    print len(pointsHit[3])

    # scan=robot.ReadLaser()

# if the cell size is small then we need to consider that the sensor is a bit to the front of the robot.




if __name__ == '__main__':
	wm=world_map(7,7,0.5)
	for row in wm.initialize_map():
		print row
	
	for row in wm.update_map(0,0,1.0):
		print row
    	lab21()
