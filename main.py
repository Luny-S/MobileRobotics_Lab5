#!/usr/bin/env python
# -*- coding: utf-8 -*-

# from drive import RosAriaDriver
# robot=RosAriaDriver('/PIONIER6')

import json
import time
import copy 
from OccupancyGrid import world_map
import PathPlanning as pp
import RobotControl

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
    wm = world_map(20, 20, 0.1)
    wm.initialize_map()

    #data = readJSON(inputPath + "robotTrap_01.JSON")
    # data = readJSON(inputPath + "map_boxes_0.json")
    # data = readJSON(inputPath + "map_boxes_1.json")
    # data = readJSON(inputPath + "map_round.json")
    data = readJSON(inputPath + "robotWandering_01.JSON")

    for iteration in data:
        wm.updateHitCells(iteration)
        #print(iteration[u'pose'])
    # for i in range(3):
    #     wm.updateHitCells(data[i])
    #print(data[-1][u'pose'])

    pp.robotRadius=0.05
    pp_wm=copy.deepcopy(wm)
    pp.enlargeObstacles(pp_wm)
	
    startPoint = [(x//pp_wm.cell_size)*pp_wm.cell_size for x in data[-1][u'pose'][0:2]]
    goalPoint = [(x//pp_wm.cell_size)*pp_wm.cell_size for x in data[0][u'pose'][0:2]]
    print(startPoint,goalPoint)

    RobotControl.ControlRobot(probabilityMap, robotPosition, goalPoint)
    
    WaveMap = pp.blastWave(pp_wm, startPoint, goalPoint)
    	
    PathPoints = pp.findPathPoints(WaveMap, startPoint, goalPoint)
    print(PathPoints)
    wm.displayMap()
    pp.plotPathPoints(PathPoints,wm.plot)
    wm.plot.show()
    
    # Podczas skanowania w ruchu mogą pojawić się opóźnienia między getPose,a getScan, więć mapa może się troche rozjechac.
    # Lepiej np. wziąć pozycje przed skanem, skan, po skanie i estymować pozycję, w której był robiony skan.
    # Można też zrobić samemu subscribera do pozycji i skanów. Ten z RosAriaDriver czasem się nie nadaje.
