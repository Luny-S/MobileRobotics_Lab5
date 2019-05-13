#!/usr/bin/env python
# -*- coding: utf-8 -*-

# from drive import RosAriaDriver
# robot=RosAriaDriver('/PIONIER6')

import json
import time
from OccupancyGrid import world_map

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


def robotScanData():
    data = []
    for i in range(6):
        dataPoint = {}
        dataPoint['pose'] = robot.GetPose()
        dataPoint['scan'] = robot.ReadLaser()
        data.append(dataPoint)
        time.sleep(1)

    saveJSON(inputPath + "robotScan.JSON", data)


if __name__ == '__main__':
    wm = world_map(12, 12, 0.05)
    wm.initialize_map()

    data = readJSON(inputPath + "robotTrap_01.JSON")
    # data = readJSON(inputPath + "map_boxes_0.json")
    # data = readJSON(inputPath + "map_boxes_1.json")
    # data = readJSON(inputPath + "map_round.json")
    # data = readJSON(inputPath + "robotWandering_01.JSON")

    for iteration in data:
        wm.updateHitCells(iteration)

    # for i in range(3):
    #     wm.updateHitCells(data[i])
    wm.displayMap()

    # Podczas skanowania w ruchu mogą pojawić się opóźnienia między getPose,a getScan, więć mapa może się troche rozjechac.
    # Lepiej np. wziąć pozycje przed skanem, skan, po skanie i estymować pozycję, w której był robiony skan.
    # Można też zrobić samemu subscribera do pozycji i skanów. Ten z RosAriaDriver czasem się nie nadaje.
