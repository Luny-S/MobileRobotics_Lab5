#!/usr/bin/env python
# -*- coding: utf-8 -*-

from drive import RosAriaDriver
robot=RosAriaDriver('/PIONIER6')

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
    wm = world_map(20, 20, 0.25)
    wm.initialize_map()

    robotScanData()

    exit(0)

    data = readJSON(inputPath + "robotScanRotating_02.JSON")
    # data = readJSON(inputPath + "map_boxes_0.json")
    # data = readJSON(inputPath + "map_boxes_1.json")
    # data = readJSON(inputPath + "map_round.json")
    # saveJSON(outputPath + "pointsHitGlobal.JSON", pointsHitGlobal)

    for iteration in data:
        wm.updateHitCells(iteration)
    # wm.updateHitCells(data[0])

    wm.updateProbabilityMap()
    mapa = np.asarray(wm.probabilityMap[::-1], dtype=np.float32)

    # Podczas skanowania w ruchu mogą pojawić się opóźnienia między getPose,a getScan, więć mapa może się troche rozjechac.
    # Lepiej np. wziąć pozycje przed skanem, skan, po skanie i estymować pozycję, w której był robiony skan.
    # Można też zrobić samemu subscribera do pozycji i skanów. Ten z RosAriaDriver czasem się nie nadaje.
    #
    # Z tym opcjonalnym taskiem:
    # pojawia sie problem, jak masz duzego cella, w  którego częsci jest obstacle to przy danym skanie
    # kilka wiazek go wykryje, a kilka nie. No i teraz dodawanie/odejmowanie dla kazdego beama jest bez sensu.
    # lLepiej dla kazdego skanu tworzyc mape temporary i jesli chociaz 1 / kilka wiazek wykryje, ze to jest przeszkoda
    # to oznaczac to jako przeszkode. (zdjecie na tel)

    # saveJSON(outputPath + "temp.JSON", wm.mapa)

    plt.imshow(mapa, interpolation="nearest", cmap='Blues')
    plt.grid(True, 'both')
    #
    subticks = 10
    plt.xticks(np.arange(0, len(mapa[0]) + 1, len(mapa[0]) / subticks), (round(x * wm.cell_size, 2) for x in
                                                                         np.arange(-len(mapa[0]) / 2, len(mapa[0]) / 2,
                                                                                   len(mapa[0]) / subticks)))
    plt.yticks(np.arange(len(mapa) + 1, 0, -len(mapa) / subticks),
               (round(x * wm.cell_size, 2) for x in np.arange(-len(mapa) / 2, len(mapa) / 2, len(mapa) / subticks)))
    # plt.axes().plot([pos[0] for pos in poses], [pos[1] for pos in poses])
    plt.colorbar()
    plt.show()
