from time import sleep
# from drive import RosAriaDriver
import math
import json

# robot=RosAriaDriver('/PIONIER6')

import matplotlib.pyplot as plt
import sys

import numpy as np

from lab5 import world_map

scan = []
theta = []
data = list()

outputPath = "./output/"
inputPath = "./datafiles/"


def readJSON(filename):
    json_file = open(filename)
    data = json.load(json_file)
    return data

def saveJSON(filename, data):
    with open(filename, 'w') as outfile:
        json.dump(data, outfile)

def convertDataToGlobalCoordinates(data):
    x = np.arange(0, 512)
    theta = (np.pi / 512) * x

    pointsHit = list()

    for iteration in data:
        pointsHitTemp = list()
        for scanIndex in range(len(iteration['scan'])):
            if not np.isinf(iteration['scan'][scanIndex]) and not np.isnan(iteration['scan'][scanIndex]):
                scanPointX = iteration['pose'][0] + 0.18 * math.cos(iteration['pose'][2]) + iteration['scan'][
                    scanIndex] * math.cos(iteration['pose'][2] + theta[scanIndex])
                scanPointY = iteration['pose'][1] + 0.18 * math.sin(iteration['pose'][2]) + iteration['scan'][
                    scanIndex] * math.sin(iteration['pose'][2] + theta[scanIndex])
                pointsHitTemp.append([scanPointX, scanPointY])

        pointsHit.append(pointsHitTemp)
    return pointsHit



if __name__ == '__main__':
    wm=world_map(7,7,0.5)
    for row in wm.initialize_map().mapa:
        print row

    for row in wm.update_map(0,0,1.0).mapa:
        print row

    data = readJSON(inputPath + "map_boxes_0.json")
    pointsHit = convertDataToGlobalCoordinates(data)
    saveJSON(outputPath + "temp.JSON", pointsHit)
