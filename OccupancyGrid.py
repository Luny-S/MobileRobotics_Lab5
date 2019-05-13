#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import math
import numpy as np

np.seterr(divide='ignore', invalid='ignore')

import json
import matplotlib.pyplot as plt


class world_map:
    cell_size = 1
    world_longitude = 0
    world_latitude = 0
    initialProbability = 0
    initialLog = 0
    debug = False
    mapa = []
    probabilityMap = []
    plot = plt

    def __init__(self, longitude, latitude,
                 cell_size):
        # constructor intializes class with basic info about world
        # [in] longitude - world longitude in physical units e.g. 10m
        # [in] latitude - world latitude in physical units e.g. 5m
        # [in] cell_size - size of map cell in physical units e.g. 0.1m
        self.cell_size = cell_size
        self.world_longitude = longitude
        self.world_latitude = latitude

    def initialize_map(self):
        # function initializing world map with given value
        # [in] value - value which is written to all the cells in map

        initialProbabilityHit = 0.5
        initialProbabilityMiss = 1 - initialProbabilityHit
        value = math.log(initialProbabilityHit / initialProbabilityMiss)

        self.initialProbability = initialProbabilityHit
        self.initialLog = value
        self.mapa = []
        for i in range(int(math.ceil(self.world_longitude / self.cell_size))):
            rowLog = []
            rowProb = []

            for j in range(int(math.ceil(self.world_latitude / self.cell_size))):
                rowLog.append(value)
                rowProb.append(self.initialProbability)

            self.mapa.append(rowLog)
            self.probabilityMap.append(rowProb)
        return self

    def updateWholeMap(self, value):
        for mapRow in self.mapa:
            for i in range(0, len(mapRow)):
                mapRow[i] = value

    def update_map(self, x, y, value, isIndex=False):
        # function updating map with given value at given position
        # [in] x - latitude in robot coordinates e.g -21
        # [in] y - longitude in robot coordinates e.g .37
        # [in] value - value which is written to the cell
        if isIndex:
            self.mapa[x][y] = value
            return self
        else:
            index = self.get_cell_index(x, y)
            self.mapa[index[0]][index[1]] = value
            return self

    def get_cell(self, x, y, isIndex=False):
        # function to read cell value at given position
        # [in] x - latitude in robot coordinates e.g -21
        # [in] y - longitude in robot coordinates e.g .37
        # [out] - value of the cell
        if isIndex:
            return self.mapa[x][y]
        else:
            index = self.get_cell_index(x, y)
            return self.mapa[index[0]][index[1]]

    def get_cell_index(self, x, y):
        longitude = int(math.ceil(round((float(self.world_longitude) / 2.0 + x) / self.cell_size, 5))) - 1
        latitude = int(math.ceil(round((float(self.world_latitude) / 2.0 + y) / self.cell_size, 5))) - 1
        return [longitude, latitude]

    def get_cell_coords(self, ix, iy):
        # longitude = self.world_longitude / 2.0 + iy * self.cell_size
        # latitude = self.world_latitude / 2.0 + ix * self.cell_size
        zeroCoord = self.get_cell_index(0, 0)
        longitude = (ix - zeroCoord[0]) * self.cell_size
        latitude = (iy - zeroCoord[1]) * self.cell_size
        return [longitude, latitude]

    def logToProbability(self, logValue):
        return 1 - 1 / (1 + math.exp(logValue))

    def getProbability(self, coordinates, isIndex=False):
        return self.logToProbability(self.get_cell(coordinates[0], coordinates[1], isIndex))

    def getProbabilityFromMap(self, x, y):
        return self.probabilityMap[x][y]

    def updateProbabilityMap(self):
        for longitude in range(len(self.probabilityMap)):
            for latitude in range(len(self.probabilityMap[longitude])):
                self.probabilityMap[longitude][latitude] = self.logToProbability(
                    self.mapa[longitude][latitude])
        return self

    def updateProbabilityMapPlanning(self):
        for longitude in range(len(self.probabilityMap)):
            for latitude in range(len(self.probabilityMap[longitude])):
                self.probabilityMap[longitude][latitude] = self.mapa[longitude][latitude]
        return self

    def convertProbabilityMapToNpArray(self):
        tempMap = np.asarray(self.probabilityMap, dtype=np.float32).transpose()
        tempMap = np.flipud(tempMap)
        return tempMap

    def inverseSensorModel(self, hit=1):
        prob = 0
        scannerDoubt = 0.1
        if (hit == 1):
            prob = 1 - scannerDoubt
        else:
            prob = 0 + scannerDoubt
        return math.log(prob / (1 - prob))

    def measurementInPerceptionField(self, iMeasurement):
        return True

    def convertScanData(self, data):
        x = np.arange(0, 512)  # z prawej do lewej [ 0 - 180 ]
        theta = (np.pi / 512) * x - (np.pi / 2)
        scanData = list()
        for index in range(len(data)):
            scanData.append([theta[index], data[index]])
        return scanData

    def getGlobalHitpoints(self, sensorPosition, scanData):
        pointsHitGlobal = list()

        for scanIndex in range(len(scanData)):
            if not np.isinf(scanData[scanIndex][1]) and not np.isnan(scanData[scanIndex][1]):
                scanPointX = sensorPosition[0] + \
                             scanData[scanIndex][1] * math.cos(sensorPosition[2] + scanData[scanIndex][0])
                scanPointY = sensorPosition[1] + \
                             scanData[scanIndex][1] * math.sin(sensorPosition[2] + scanData[scanIndex][0])

                pointsHitGlobal.append([scanPointX, scanPointY])
        return pointsHitGlobal

    def getLaserPath(self, LaserPath, startPoint, finalPoint):
        # key = str(fix) + "." + str(fiy)
        diff = [int(float(finalPoint[0] - startPoint[0]) / 2), int(float(finalPoint[1] - startPoint[1]) / 2)]
        midPoint = [finalPoint[0] - diff[0], finalPoint[1] - diff[1]]

        key = str(midPoint[0]) + "." + str(midPoint[1])
        if LaserPath.get(key, 0) != 1:
            LaserPath[key] = 0;

        if abs(diff[0]) + abs(diff[1]) > 0:
            self.getLaserPath(LaserPath, startPoint, midPoint)
            self.getLaserPath(LaserPath, midPoint, finalPoint)
        else:
            return LaserPath

    def updateHitCells(self, data):
        global colors
        # function to update log map
        # [in] data - list of data from one iterations. Has sublists 'pose' and 'scan'.

        sensorPosition = data['pose']
        sensorPosition[0] += 0.18 * math.cos(sensorPosition[2])
        sensorPosition[1] += 0.18 * math.sin(sensorPosition[2])
        sensorPosition[2] = np.deg2rad(sensorPosition[2])

        scanData = self.convertScanData(data['scan'])
        globalPoints = self.getGlobalHitpoints(sensorPosition, scanData)
        pathPoints = dict()  # points through which the beam passes
        six, siy = self.get_cell_index(sensorPosition[0], sensorPosition[1])  # sensor position indexes

        for globalPoint in globalPoints:
            # path from sensorPosition to globalPoint
            fix, fiy = self.get_cell_index(globalPoint[0], globalPoint[1])  # final point indexes
            point = [six, siy]

            key = str(fix) + "." + str(fiy)
            pathPoints[key] = 1

            self.getLaserPath(pathPoints, point[:], [fix, fiy])

        for key, value in pathPoints.iteritems():
            pIndex = key.split('.')
            logValue = self.get_cell(int(pIndex[0]), int(pIndex[1]), True) + self.inverseSensorModel(
                value) - self.initialLog
            self.update_map(int(pIndex[0]), int(pIndex[1]), logValue, True)

        self.updateProbabilityMap()
        return True

    def displayMap(self):

        # pathListX = [0, 0 ,1, 1]
        # pathListY = [0, 1, 1, 0]


        pointsList = self.mapToPointsList()

        self.plot.scatter(pointsList[0], pointsList[1], c=pointsList[2], cmap='Blues', marker='s', s=np.sqrt(self.cell_size)*10, alpha=1)
        self.plot.grid(True, 'both')
        self.plot.show()

    def mapToPointsList(self):
        pointsList = [[],[],[]]
        for longitude in range(len(self.probabilityMap)):
            for latitude in range(len(self.probabilityMap[longitude])):
                if self.probabilityMap[longitude][latitude] != self.initialProbability:
                    point = self.get_cell_coords(longitude, latitude)
                    pointsList[0].append(point[0])
                    pointsList[1].append(point[1])
                    pointsList[2].append(self.probabilityMap[longitude][latitude])

        return pointsList
