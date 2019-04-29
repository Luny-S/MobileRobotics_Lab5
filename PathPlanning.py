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
# 2. Generate path as list of cells to pass through (check previous direction) (/)
# 3. Enlarge obstacles
# 3. optimize cell visiting
# 4*. Diagonal movement (/)
# 5*. Some crazy shit (/)


#obstacles = "inf"
#goal = 0
#start = "-inf"
#initial = "NaN"

probabilityThreshold = 0.75
robotRadius = 0.1

class direction(enum.Enum):
	vertical = 0
	horizontal = 1
	slash = 2
	backslash = 3

def updatePoint(Map, Point, iteration, futurePointList, probabilityMap):
	if(Point[0] > -float(Map.world_latitude)/2 and Point[0] < float(Map.world_latitude)/2 and
	   Point[1] > -float(Map.world_longitude)/2 and Point[1] < float(Map.world_longitude)/2):
		if(probabilityMap.getProbability([Point[0], Point[1]]) >= probabilityThreshold):
			Map.update_map(Point[0], Point[1], float("inf"))
		else:
			if(math.isnan(Map.get_cell(Point[0], Point[1])) or Map.get_cell(Point[0], Point[1]) == float("-inf")):
				Map.update_map(Point[0], Point[1], iteration)
				futurePointList.append([Point[0], Point[1]])


def updateNeighbours(Map, currentPoint, iteration, futurePointList, probabilityMap):
	updatePoint(Map, [currentPoint[0]-Map.cell_size, currentPoint[1]],
	            iteration, futurePointList, probabilityMap)
	updatePoint(Map, [currentPoint[0]+Map.cell_size, currentPoint[1]],
	            iteration, futurePointList, probabilityMap)
	updatePoint(Map, [currentPoint[0], currentPoint[1]-Map.cell_size],
	            iteration, futurePointList, probabilityMap)
	updatePoint(Map, [currentPoint[0], currentPoint[1]+Map.cell_size],
	            iteration, futurePointList, probabilityMap)


def startFound(goalDistanceMap, currentPointList, startPoint):
	for currentPoint in currentPointList:
		if((abs(currentPoint[0] - startPoint[0]) < 0.1*goalDistanceMap.cell_size) and (abs(currentPoint[1] - startPoint[1]) < 0.1*goalDistanceMap.cell_size)):
			return True
	return False

def enlargeObstacles(probabilityMap):
	orthoCellDistance = int(math.ceil(float(robotRadius) / probabilityMap.cell_size))
	for i in range(0,len(probabilityMap.mapa)):
		for j in range(0,len(probabilityMap.mapa[0])):
			if(probabilityMap.getProbabilityFromMap(i,j) > probabilityThreshold):
				cellX = i
				cellY = j
				for ii in range(-orthoCellDistance, orthoCellDistance+1):
					for jj in range(-orthoCellDistance, orthoCellDistance+1):
						if(cellX+ii < len(probabilityMap.mapa) and cellY+jj < len(probabilityMap.mapa) and cellX+ii >= 0 and cellY+jj >= 0):
							if(probabilityMap.getProbability([cellX+ii,cellY+jj],True) < probabilityThreshold):
								if(math.sqrt((ii)**2 + (jj)**2) <= orthoCellDistance):
									probabilityMap.update_map(cellX+ii,cellY+jj,8,True)
	probabilityMap.updateProbabilityMap


def blastWave(probabilityMap, startPoint, goalPoint):
	
	goalDistanceMap = world_map(probabilityMap.world_latitude,
                             probabilityMap.world_longitude,
                             probabilityMap.cell_size)
	goalDistanceMap.initialize_map()

	goalDistanceMap.updateWholeMap(float("NaN"))
	goalDistanceMap.update_map(startPoint[0], startPoint[1],float("-inf"))
	goalDistanceMap.update_map(goalPoint[0], goalPoint[1],0)

	currentPointList = []
	currentPointList.append(goalPoint)
	
	futurePointList = []

	iteration = 1

	while (not startFound(goalDistanceMap, currentPointList, startPoint)):
		if(not currentPointList):
			return goalDistanceMap
		for currentPoint in currentPointList:
			updateNeighbours(goalDistanceMap, currentPoint, iteration,
			                 futurePointList, probabilityMap)
		iteration += 1
		currentPointList = futurePointList
		futurePointList = []

	goalDistanceMap.update_map(startPoint[0], startPoint[1], iteration)
	goalDistanceMap.update_map(goalPoint[0], goalPoint[1], 0)
	return goalDistanceMap

def chooseNeighbourOrthogonal(goalDistanceMap, currentPoint, preferredDirection):
	chosenNeighbour = []
	currentCell = goalDistanceMap.get_cell(currentPoint[0], currentPoint[1])
	leftNeighbour = goalDistanceMap.get_cell(currentPoint[0]-goalDistanceMap.cell_size, currentPoint[1])
	rightNeighbour = goalDistanceMap.get_cell(currentPoint[0]+goalDistanceMap.cell_size, currentPoint[1])
	bottomNeighbour = goalDistanceMap.get_cell(currentPoint[0], currentPoint[1]-goalDistanceMap.cell_size)
	topNeighbour = goalDistanceMap.get_cell(currentPoint[0], currentPoint[1]+goalDistanceMap.cell_size)
	
	if(preferredDirection == direction.vertical):
		if(bottomNeighbour < currentCell):
			chosenNeighbour = [currentPoint[0], currentPoint[1]-goalDistanceMap.cell_size]
			preferredDirection = direction.vertical
		elif(topNeighbour < currentCell):
			chosenNeighbour = [currentPoint[0], currentPoint[1]+goalDistanceMap.cell_size]
			preferredDirection = direction.vertical
		else:
			if(leftNeighbour < currentCell):
				chosenNeighbour = [currentPoint[0]-goalDistanceMap.cell_size, currentPoint[1]]
				preferredDirection = direction.horizontal
			elif(rightNeighbour < currentCell):
				chosenNeighbour = [currentPoint[0]+goalDistanceMap.cell_size, currentPoint[1]]
				preferredDirection = direction.horizontal
	else:
		if(leftNeighbour < currentCell):
			chosenNeighbour = [currentPoint[0]-goalDistanceMap.cell_size, currentPoint[1]]
			preferredDirection = direction.horizontal
		elif(rightNeighbour < currentCell):
			chosenNeighbour = [currentPoint[0]+goalDistanceMap.cell_size, currentPoint[1]]
			preferredDirection = direction.horizontal
		else:
			if(bottomNeighbour < currentCell):
				chosenNeighbour = [currentPoint[0], currentPoint[1]-goalDistanceMap.cell_size]
				preferredDirection = direction.vertical
			elif(topNeighbour < currentCell):
				chosenNeighbour = [currentPoint[0], currentPoint[1]+goalDistanceMap.cell_size]
				preferredDirection = direction.vertical
	return chosenNeighbour, preferredDirection

def chooseNeighbourDiagonal(goalDistanceMap, currentPoint, preferredDirection):
	chosenNeighbour = []
	currentCell = goalDistanceMap.get_cell(currentPoint[0], currentPoint[1])
	topleftNeighbour = goalDistanceMap.get_cell(currentPoint[0]-goalDistanceMap.cell_size, currentPoint[1]+goalDistanceMap.cell_size)
	toprightNeighbour = goalDistanceMap.get_cell(currentPoint[0]+goalDistanceMap.cell_size, currentPoint[1]+goalDistanceMap.cell_size)
	bottomleftNeighbour = goalDistanceMap.get_cell(currentPoint[0]-goalDistanceMap.cell_size, currentPoint[1]-goalDistanceMap.cell_size)
	bottomrightNeighbour = goalDistanceMap.get_cell(currentPoint[0]+goalDistanceMap.cell_size, currentPoint[1]-goalDistanceMap.cell_size)
	if(preferredDirection == direction.slash):
		if(bottomleftNeighbour < currentCell):
			chosenNeighbour = [currentPoint[0]-goalDistanceMap.cell_size, currentPoint[1]-goalDistanceMap.cell_size]
			preferredDirection = direction.slash
		elif(toprightNeighbour < currentCell):
			chosenNeighbour = [currentPoint[0]+goalDistanceMap.cell_size, currentPoint[1]+goalDistanceMap.cell_size]
			preferredDirection = direction.slash
		else:
			if(bottomrightNeighbour < currentCell):
				chosenNeighbour = [currentPoint[0]+goalDistanceMap.cell_size, currentPoint[1]-goalDistanceMap.cell_size]
				preferredDirection = direction.backslash
			elif(topleftNeighbour < currentCell):
				chosenNeighbour = [currentPoint[0]-goalDistanceMap.cell_size, currentPoint[1]+goalDistanceMap.cell_size]
				preferredDirection = direction.backslash
	elif(preferredDirection == direction.backslash):
		if(bottomrightNeighbour < currentCell):
			chosenNeighbour = [currentPoint[0]+goalDistanceMap.cell_size, currentPoint[1]-goalDistanceMap.cell_size]
			preferredDirection = direction.backslash
		elif(topleftNeighbour < currentCell):
			chosenNeighbour = [currentPoint[0]-goalDistanceMap.cell_size, currentPoint[1]+goalDistanceMap.cell_size]
			preferredDirection = direction.backslash
		else:
			if(bottomleftNeighbour < currentCell):
				chosenNeighbour = [currentPoint[0]-goalDistanceMap.cell_size, currentPoint[1]-goalDistanceMap.cell_size]
				preferredDirection = direction.slash
			elif(toprightNeighbour < currentCell):
				chosenNeighbour = [currentPoint[0]+goalDistanceMap.cell_size, currentPoint[1]+goalDistanceMap.cell_size]
				preferredDirection = direction.slash
	else:
		if(bottomleftNeighbour < currentCell):
			chosenNeighbour = [currentPoint[0]-goalDistanceMap.cell_size, currentPoint[1]-goalDistanceMap.cell_size]
			preferredDirection = direction.slash
		elif(toprightNeighbour < currentCell):
			chosenNeighbour = [currentPoint[0]+goalDistanceMap.cell_size, currentPoint[1]+goalDistanceMap.cell_size]
			preferredDirection = direction.slash
		else:
			if(bottomrightNeighbour < currentCell):
				chosenNeighbour = [currentPoint[0]+goalDistanceMap.cell_size, currentPoint[1]-goalDistanceMap.cell_size]
				preferredDirection = direction.backslash
			elif(topleftNeighbour < currentCell):
				chosenNeighbour = [currentPoint[0]-goalDistanceMap.cell_size, currentPoint[1]+goalDistanceMap.cell_size]
				preferredDirection = direction.backslash
	return chosenNeighbour, preferredDirection

def chooseNeighbour(goalDistanceMap, currentPoint, preferredDirection):
	chosenNeighbour = []

	chosenNeighbourDiagonal, preferredDirectionDiagonal = chooseNeighbourDiagonal(goalDistanceMap, currentPoint, preferredDirection)
	chosenNeighbourOrthogonal, preferredDirectionOrthogonal = chooseNeighbourOrthogonal(goalDistanceMap, currentPoint, preferredDirection)

	if(not chosenNeighbourDiagonal):
		return chosenNeighbourOrthogonal, preferredDirectionOrthogonal
	if(not chosenNeighbourOrthogonal):
		return chosenNeighbourDiagonal, preferredDirectionDiagonal

	if(goalDistanceMap.get_cell(chosenNeighbourDiagonal[0],chosenNeighbourDiagonal[1]) <= goalDistanceMap.get_cell(chosenNeighbourOrthogonal[0],chosenNeighbourOrthogonal[1])):
		return chosenNeighbourDiagonal, preferredDirectionDiagonal
	else:
		return chosenNeighbourOrthogonal, preferredDirectionOrthogonal

def findPathPoints(goalDistanceMap, startPoint, goalPoint):
	pathPoints = []
	preferredDirection = direction.vertical
	currentPoint = startPoint
	pathPoints.append(currentPoint)

	while(not (abs(currentPoint[0] - goalPoint[0]) < 0.1*goalDistanceMap.cell_size and abs(currentPoint[1] - goalPoint[1]) < 0.1*goalDistanceMap.cell_size)):
		currentPoint, preferredDirection = chooseNeighbour(goalDistanceMap, currentPoint, preferredDirection)
		if(not currentPoint):
			return pathPoints
		pathPoints.append(currentPoint)

	return pathPoints


def planPath(pathPoints):
	return polyPath

def plotWavePath(WaveMap, pathPoints):
	fig = plt.figure(figsize=(8,8))
	for i in range(0, len(WaveMap.mapa)):
		for j in range(0, len(WaveMap.mapa[0])):
			point = WaveMap.get_cell_coords(i,j)
			pointValue = WaveMap.get_cell(i,j,True) 
			if math.isnan(pointValue):
				pointValue = 0.2
				color = 'k'
			elif math.isinf(pointValue):
				pointValue = 1
				color = 'k'
			else:
				pointValue = float(pointValue) / (2 * WaveMap.world_latitude / WaveMap.cell_size - 2)
				color = 'b'
			size = fig.get_size_inches()*fig.dpi
			plt.scatter(point[0],point[1], c=color, s = size[0]/(0.085*WaveMap.world_latitude/WaveMap.cell_size)**2 , marker='s', cmap='hsv',alpha=pointValue)

	npathPoints = np.array(pathPoints)
	npathPoints[:,0]
	plt.plot(npathPoints[:,0],npathPoints[:,1],'ro-')
	plt.show()

def addObstacles(wm):
	wm.update_map(-2, 0, 10)
	wm.update_map(-2, 1, 10)
	wm.update_map(-2, 2, 10)
	wm.update_map(-2, 3, 10)
	wm.update_map(-2, 4, 10)
	wm.update_map(-2, 5, 10)
	wm.update_map(-2, 6, 10)
	wm.update_map(-3, 0, 10)
	wm.update_map(-3, 1, 10)
	wm.update_map(-3, 2, 10)
	wm.update_map(-3, 3, 10)
	wm.update_map(-3, 4, 10)
	wm.update_map(-3, 5, 10)
	wm.update_map(-3, 6, 10)

	wm.update_map(-4, -1, 10)
	wm.update_map(-4, -2, 10)
	wm.update_map(-1, 7, 10)
	wm.update_map(-1, 8, 10)
	wm.update_map(-5, 0, 10)

	wm.update_map(-9, 4, 10)
	wm.update_map(-9, 3, 10)
	wm.update_map(-9, 2, 10)
	wm.update_map(-9, 1, 10)
	wm.update_map(-8, 1, 10)
	wm.update_map(-7, 1, 10)
	wm.update_map(-6, 1, 10)
	wm.update_map(-6, 2, 10)
	wm.update_map(-6, 3, 10)
	wm.update_map(-6, 4, 10)

	
	wm.update_map(-6, -1, 10)
	wm.update_map(-6, -2, 10)
	wm.update_map(-6, -3, 10)
	wm.update_map(-6, -4, 10)
	wm.update_map(-6, -5, 10)
	wm.update_map(-6, -6, 10)
	wm.update_map(-10, 6, 10)
	wm.update_map(-10, 7, 10)
	wm.update_map(-10, 8, 10)

	wm.updateProbabilityMap()


if __name__ == '__main__':
	wm = world_map(30, 30, 1)
	wm.initialize_map()

	addObstacles(wm)

	wm.updateProbabilityMap()

	enlargeObstacles(wm)

	startPoint = [0, 0]
	goalPoint = [-6, -6]

	WaveMap = blastWave(wm, startPoint, goalPoint)
	PathPoints = findPathPoints(WaveMap, startPoint, goalPoint)

	plotWavePath(WaveMap,PathPoints)
