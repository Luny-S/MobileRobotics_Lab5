#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import numpy as np
import matplotlib.pyplot as plt
import time

from map import world_map

# TODO
# 1. Create wave that assigns numbers to map
# 2. Generate path as list of cells to pass through
# 3. Enlarge obstacles
# 3. optimize cell visiting
# 4*. Diagonal movement
# 5*. Some crazy shit


#obstacles = "-inf"
#goal = "NaN"
#start = "inf"
#initial = 0

probabilityThreshold = 0.75

def updatePoint(Map,Point, iteration, probabilityMap):
	if(Point[0]>=0 and Point[0]<Map.world_latitude and \
	   Point[1]>=0 and Point[1]<Map.world_longitude):
		if(probabilityMap.getProbability([Point[0],Point[1]]) >= probabilityThreshold):
			Map.update_map(Point[0],Point[1],float("-inf"))
		else:
			if(Map.get_cell(Point[0],Point[1]) == 0):
				Map.update_map(Point[0], Point[1], iteration)

def updateNeighbours(Map,currentPoint,iteration, futurePointList, probabilityMap):
	updatePoint(Map,[currentPoint[0]-1, currentPoint[1]], iteration, probabilityMap)
	futurePointList.append([currentPoint[0]-1,currentPoint[1]])

	updatePoint(Map,[currentPoint[0]+1, currentPoint[1]], iteration, probabilityMap)
	futurePointList.append([currentPoint[0]+1,currentPoint[1]])

	updatePoint(Map,[currentPoint[0], currentPoint[1]-1], iteration, probabilityMap)
	futurePointList.append([currentPoint[0],currentPoint[1]-1])

	updatePoint(Map,[currentPoint[0], currentPoint[1]+1], iteration, probabilityMap)
	futurePointList.append([currentPoint[0],currentPoint[1]+1])


def blastWave(probabilityMap, startPoint, goalPoint):
	goalDistanceMap = world_map(probabilityMap.world_latitude, \
				   probabilityMap.world_longitude,\
                                   probabilityMap.cell_size)
	goalDistanceMap.initialize_map()
	
	goalDistanceMap.update_map(startPoint[0], startPoint[1], float("inf"))
	goalDistanceMap.update_map(goalPoint[0], goalPoint[1], float("NaN"))
	
	currentPointList = []
	currentPointList.append(goalPoint)
	
	futurePointList = []

	iteration = 1

	for currentPoint in currentPointList:
		updateNeighbours(goalDistanceMap,currentPoint,iteration, futurePointList, probabilityMap)

	print futurePointList
	
	return goalDistanceMap

def planPath(goalDistanceMap):



	return pathPoints

#wm.update_map(0,0,100)
#print wm.get_cell(0,0)

#wm.updateProbabilityMap()


if __name__ == '__main__':
    wm = world_map(10, 10, 1)
    wm.initialize_map()

    wm.update_map(2,2,10)
    wm.updateProbabilityMap()


    WaveMap = blastWave(wm, [0,0], [1,3])
    
    mapa = np.asarray(WaveMap.mapa[::-1], dtype=np.float32)

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
