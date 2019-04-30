#!/usr/bin/env python
# -*- coding: utf-8 -*-

# from drive import RosAriaDriver
# robot=RosAriaDriver('/PIONIER6')


import math
import json

import numpy as np

import matplotlib.pyplot as plt
import time

from map import world_map

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
    wm = world_map(10, 10, 0.05)
    wm.initialize_map()
    # robot obracal sie w lewo
    data = readJSON(inputPath + "robotTrap_01.JSON")
    # data = readJSON(inputPath + "map_boxes_0.json")
    # data = readJSON(inputPath + "map_boxes_1.json")
    # data = readJSON(inputPath + "map_round.json")
    # data = readJSON(inputPath + "robotWandering_01.JSON")

    for iteration in data:
        wm.updateHitCells(iteration)
    # for i in range(1):
    #     wm.updateHitCells(data[i])

    wm.updateProbabilityMap()
    mapa = wm.convertProbabilityMapToNpArray()

    # Podczas skanowania w ruchu mogą pojawić się opóźnienia między getPose,a getScan, więć mapa może się troche rozjechac.
    # Lepiej np. wziąć pozycje przed skanem, skan, po skanie i estymować pozycję, w której był robiony skan.
    # Można też zrobić samemu subscribera do pozycji i skanów. Ten z RosAriaDriver czasem się nie nadaje.

    plt.imshow(mapa, interpolation="nearest", cmap='Blues')
    plt.grid(True, 'both')
    subticks = 10
    plt.xticks(np.arange(0, len(mapa[0]) + 1, len(mapa[0]) / subticks), (round(x * wm.cell_size, 2) for x in
                                                                         np.arange(-len(mapa[0]) / 2, len(mapa[0]) / 2,
                                                                                   len(mapa[0]) / subticks)))
    plt.yticks(np.arange(len(mapa) + 1, 0, -len(mapa) / subticks),
               (round(x * wm.cell_size, 2) for x in np.arange(-len(mapa) / 2, len(mapa) / 2, len(mapa) / subticks)))
    plt.colorbar()
    plt.show()
    pass