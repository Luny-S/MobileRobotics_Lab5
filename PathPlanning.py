#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import numpy as np
import matplotlib.pyplot as plt
import enum
import time

from map import world_map

# TODO
# 1. Create wave that assigns numbers to map (/)
# 2. Generate path as list of cells to pass through (check previous direction)
# 3. Enlarge obstacles
# 3. optimize cell visiting
# 4*. Diagonal movement
# 5*. Some crazy shit


#obstacles = "inf"
#goal = 0
#start = "-inf"
#initial = "NaN"

probabilityThreshold = 0.75

class direction(enum.Enum):
	vertical = 0
	horizontal = 1

def updatePoint(Map, Point, iteration, futurePointList, probabilityMap):
	if(Point[0] >= -Map.world_latitude/2 and Point[0] < Map.world_latitude/2 and
	   Point[1] >= -Map.world_latitude/2 and Point[1] < Map.world_longitude/2):
		if(probabilityMap.getProbability([Point[0], Point[1]]) >= probabilityThreshold):
			Map.update_map(Point[0], Point[1], float("inf"))
		else:
			if(math.isnan(Map.get_cell(Point[0], Point[1])) or Map.get_cell(Point[0], Point[1]) == float("-inf")):
				Map.update_map(Point[0], Point[1], iteration)
				futurePointList.append([Point[0], Point[1]])


def updateNeighbours(Map, currentPoint, iteration, futurePointList, probabilityMap):
	updatePoint(Map, [currentPoint[0]-1, currentPoint[1]],
	            iteration, futurePointList, probabilityMap)
	updatePoint(Map, [currentPoint[0]+1, currentPoint[1]],
	            iteration, futurePointList, probabilityMap)
	updatePoint(Map, [currentPoint[0], currentPoint[1]-1],
	            iteration, futurePointList, probabilityMap)
	updatePoint(Map, [currentPoint[0], currentPoint[1]+1],
	            iteration, futurePointList, probabilityMap)


def startFound(goalDistanceMap, currentPointList, startPoint):
	for currentPoint in currentPointList:
		if(currentPoint[0] == startPoint[0] and currentPoint[1] == startPoint[1]):
			return True
	return False


def blastWave(probabilityMap, startPoint, goalPoint):
	goalDistanceMap = world_map(probabilityMap.world_latitude,
                             probabilityMap.world_longitude,
                             probabilityMap.cell_size)
	goalDistanceMap.initialize_map()

	goalDistanceMap.updateWholeMap(float("NaN"))
	goalDistanceMap.update_map(startPoint[0], startPoint[1], float("-inf"))
	goalDistanceMap.update_map(goalPoint[0], goalPoint[1], 0)

	currentPointList = []
	currentPointList.append(goalPoint)
	
	futurePointList = []

	iteration = 1

	while (not startFound(goalDistanceMap, currentPointList, startPoint)):
		for currentPoint in currentPointList:
			updateNeighbours(goalDistanceMap, currentPoint, iteration,
			                 futurePointList, probabilityMap)
		iteration += 1
		currentPointList = futurePointList
		futurePointList = []

	goalDistanceMap.update_map(startPoint[0], startPoint[1], iteration)
	goalDistanceMap.update_map(goalPoint[0], goalPoint[1], 0)
	return goalDistanceMap

def chooseNeighbour(goalDistanceMap, currentPoint, preferredDirection):
	chosenNeighbour = []
	currentCell = goalDistanceMap.get_cell(currentPoint[0], currentPoint[1])
	leftNeighbour = goalDistanceMap.get_cell(currentPoint[0]-1, currentPoint[1])
	rightNeighbour = goalDistanceMap.get_cell(currentPoint[0]+1, currentPoint[1])
	bottomNeighbour = goalDistanceMap.get_cell(currentPoint[0], currentPoint[1]-1)
	topNeighbour = goalDistanceMap.get_cell(currentPoint[0], currentPoint[1]+1)
	
	if(preferredDirection == direction.vertical):
		if(bottomNeighbour <= currentCell - 1):
			chosenNeighbour = [currentPoint[0], currentPoint[1]-1]
			preferredDirection = direction.vertical
		elif(topNeighbour <= currentCell - 1):
			chosenNeighbour = [currentPoint[0], currentPoint[1]+1]
			preferredDirection = direction.vertical
		else:
			if(leftNeighbour <= currentCell - 1):
				chosenNeighbour = [currentPoint[0]-1, currentPoint[1]]
				preferredDirection = direction.horizontal
			elif(rightNeighbour <= currentCell - 1):
				chosenNeighbour = [currentPoint[0]+1, currentPoint[1]]
				preferredDirection = direction.horizontal
	else:
		if(leftNeighbour <= currentCell - 1):
			chosenNeighbour = [currentPoint[0]-1, currentPoint[1]]
			preferredDirection = direction.horizontal
		elif(rightNeighbour <= currentCell - 1):
			chosenNeighbour = [currentPoint[0]+1, currentPoint[1]]
			preferredDirection = direction.horizontal
		else:
			if(bottomNeighbour <= currentCell - 1):
				chosenNeighbour = [currentPoint[0], currentPoint[1]-1]
				preferredDirection = direction.vertical
			elif(topNeighbour <= currentCell - 1):
				chosenNeighbour = [currentPoint[0], currentPoint[1]+1]
				preferredDirection = direction.vertical
	return chosenNeighbour, preferredDirection

def findPathPoints(goalDistanceMap, startPoint, goalPoint):
	pathPoints = []
	preferredDirection = direction.vertical
	currentPoint = startPoint
	pathPoints.append(currentPoint)

	while(not (currentPoint[0] == goalPoint[0] and currentPoint[1] == goalPoint[1])):
		currentPoint, preferredDirection = chooseNeighbour(goalDistanceMap, currentPoint, preferredDirection)
		pathPoints.append(currentPoint)

	print (pathPoints)
	raw_input("Press Enter to continue...")
	return pathPoints


def planPath(pathPoints):

	return path

#wm.update_map(0,0,100)
#print wm.get_cell(0,0)

#wm.updateProbabilityMap()


if __name__ == '__main__':
	wm = world_map(10, 10, 1)
	wm.initialize_map()

	wm.update_map(2, 2, 10)
	wm.updateProbabilityMap()

	startPoint = [0, 0]
	goalPoint = [-4, 2]

	WaveMap = blastWave(wm, startPoint, goalPoint)
	PathPoints = findPathPoints(WaveMap, startPoint, goalPoint)

	print(WaveMap.get_cell(0, 0))
	print(WaveMap.get_cell(2, 2))
	print(WaveMap.get_cell(1, 3))

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