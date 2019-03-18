from time import sleep

# from drive import RosAriaDriver
# robot=RosAriaDriver('/PIONIER6')

import math
import json


import matplotlib.pyplot as plt
import sys

import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

import time


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
    x = np.arange(0, 512) # z prawej do lewej [ 0 - 180 ]
    theta = (np.pi / 512) * x - (np.pi/2)


    pointsHit = list()

    for iteration in data:
        pointsHitTemp = list()

        iteration['pose'][2] = np.deg2rad(iteration['pose'][2])

        for scanIndex in range(len(iteration['scan'])):

            if not np.isinf(iteration['scan'][scanIndex]) and not np.isnan(iteration['scan'][scanIndex]):
                scanPointX = iteration['pose'][1] + \
                             iteration['scan'][scanIndex] * math.sin(iteration['pose'][2] + theta[scanIndex]) + \
                             0.18 * math.sin(iteration['pose'][2])
                scanPointY = iteration['pose'][0] + \
                             iteration['scan'][scanIndex] * math.cos(iteration['pose'][2] + theta[scanIndex]) + \
                             0.18 * math.cos(iteration['pose'][2])

                scanPointX = -scanPointX

                pointsHitTemp.append([scanPointX, scanPointY])

        pointsHit.append(pointsHitTemp)
    return pointsHit

def robotScanData():
    data = []
    for i in range(10):
        dataPoint = {}
        dataPoint['pose'] = robot.GetPose()
        dataPoint['scan'] = robot.ReadLaser()
        data.append(dataPoint)
        time.sleep(1)

    saveJSON(inputPath + "robotScan.JSON", data)


if __name__ == '__main__':
    wm=world_map(20,20,0.05)
    wm.initialize_map()

    # data = readJSON(inputPath + "robotScanRotating_01.JSON")
    data = readJSON(inputPath + "robotWandering_01.JSON")

    # robotScanData()

    # poses = [item['pose'] for item in data]

    pointsHit = convertDataToGlobalCoordinates(data)

    saveJSON(outputPath + "pointsHit.JSON", pointsHit)

    for iter in pointsHit:
        wm.updateHitCells(iter)

    # wm.updateHitCells(pointsHit[0])
    # wm.updateHitCells(pointsHit[1])
    # wm.updateHitCells(pointsHit[2])
    # wm.updateHitCells(pointsHit[3])
    # wm.updateHitCells(pointsHit[4])

    wm.updateProbabilityMap()
    mapa = np.asarray(wm.probabilityMap[::-1], dtype=np.float32)

    # TODO Skalowanie mapy z indeksow na wlasciwe koordynaty
    # TODO Przejrzec przetwarzanie danych, bo cos nie pasuje z wartosciami. Sa jakby ucinane

    # Podczas skanowania w ruchu mogą pojawić się opóźnienia między getPose,a getScan, więć mapa może się troche rozjechac.
    # Lepiej np. wziąć pozycje przed skanem, skan, po skanie i estymować pozycję, w której był robiony skan.
    # Można też zrobić samemu subscribera do pozycji i skanów. Ten z RosAriaDriver czasem się nie nadaje.
    #
    # Z tym opcjonalnym taskiem:
    # pojawia sie problem, jak masz duzego cella, w  którego częsci jest obstacle to przy danym skanie
    #kilka wiazek go wykryje, a kilka nie. No i teraz dodawanie/odejmowanie dla kazdego beama jest bez sensu.
    #lLepiej dla kazdego skanu tworzyc mape temporary i jesli chociaz 1 / kilka wiazek wykryje, ze to jest przeszkoda
    # to oznaczac to jako przeszkode. (zdjecie na tel)



    # saveJSON(outputPath + "temp.JSON", wm.mapa)
    # W drugije iteracji powinien sie wykres tylko przesunac w prawo/lewo o 0.5m

    # xs = [x[0] for x in pointsHit[0]]
    # ys = [x[1] for x in pointsHit[0]]
    # plt.scatter(xs, ys, s=1, c="red")
    #
    # xs = data[0]['pose'][1]
    # ys = data[0]['pose'][0]
    # plt.scatter(xs, ys, s=100, c="red", marker='x')
    #
    #
    # xs = [x[0] for x in pointsHit[1]]
    # ys = [x[1] for x in pointsHit[1]]
    # plt.scatter(xs, ys, s=1, c="green")
    # xs = data[1]['pose'][1]
    # ys = data[1]['pose'][0]
    # plt.scatter(xs, ys, s=100, c="green", marker='x')
    #
    # xs = [x[0] for x in pointsHit[2]]
    # ys = [x[1] for x in pointsHit[2]]
    # plt.scatter(xs, ys, s=1, c="blue")
    # xs = data[2]['pose'][1]
    # ys = data[2]['pose'][0]
    # plt.scatter(xs, ys, s=100, c="blue", marker='x')



    #
    #
    plt.imshow(mapa, interpolation="nearest", cmap='Blues')
    plt.grid(True, 'both')
    #
    subticks = 10
    plt.xticks(np.arange(0, len(mapa[0]) + 1, len(mapa[0]) / subticks), (round(x * wm.cell_size, 2) for x in
                                                                         np.arange(-len(mapa[0]) / 2, len(mapa[0]) / 2,
                                                                                   len(mapa[0]) / subticks)))
    plt.yticks(np.arange(len(mapa) + 1,0, -len(mapa) / subticks),
               (round(x * wm.cell_size, 2) for x in np.arange(-len(mapa) / 2, len(mapa) / 2, len(mapa) / subticks)))
    # plt.axes().plot([pos[0] for pos in poses], [pos[1] for pos in poses])
    plt.colorbar()
    plt.show()