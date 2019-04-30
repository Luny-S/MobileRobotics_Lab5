#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import math
import numpy as np

np.seterr(divide='ignore', invalid='ignore')

import json

# latitude - szerokość - x
# longitude - dlugosc geo. - y

# test!
import matplotlib.pyplot as plt2


class world_map:
    cell_size = 1
    world_longitude = 0
    world_latitude = 0
    initialProbability = 0
    initialLog = 0
    debug = False
    mapa = []
    probabilityMap = []

    def __init__(self, longitude, latitude,
                 cell_size):
        # constructor intializes class with basic info about world
        # [in] longitude - world longitude in physical units e.g. 10m
        # [in] latitude - world latitude in physical units e.g. 5m
        # [in] cell_size - size of map cell in physical units e.g. 0.1m
        self.cell_size = cell_size
        self.world_latitude = latitude
        self.world_longitude = longitude

    def initialize_map(self):
        # function initializing world map with given value
        # [in] value - value which is written to all the cells in map

        initialProbabilityHit = 0.5
        initialProbabilityMiss = 1 - initialProbabilityHit
        value = math.log(initialProbabilityHit / initialProbabilityMiss)

        self.initialProbability = initialProbabilityHit
        self.initialLog = value
        for i in range(int(math.ceil(self.world_latitude / self.cell_size))):
            rowLog = []
            rowProb = []

            for j in range(int(math.ceil(self.world_longitude / self.cell_size))):
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
        longitude = int(math.ceil(round((float(self.world_longitude) / 2.0 + y) / self.cell_size, 5))) - 1
        latitude = int(math.ceil(round((float(self.world_latitude) / 2.0 + x) / self.cell_size, 5))) - 1
        return [latitude, longitude]

    def get_cell_coords(self, ix, iy):
        # longitude = self.world_longitude / 2.0 + iy * self.cell_size
        # latitude = self.world_latitude / 2.0 + ix * self.cell_size
        zeroCoord = self.get_cell_index(0, 0)
        longitude = (iy - zeroCoord[0]) * self.cell_size
        latitude = (ix - zeroCoord[1]) * self.cell_size
        return [latitude, longitude]

    def logToProbability(self, logValue):
        return 1 - 1 / (1 + math.exp(logValue))

    def getProbability(self, coordinates, isIndex=False):
        return self.logToProbability(self.get_cell(coordinates[0], coordinates[1], isIndex))

    def getProbabilityFromMap(self, x, y):
        return self.probabilityMap[x][y]

    def updateProbabilityMap(self):
        for latitude in range(len(self.probabilityMap)):
            for longitude in range(len(self.probabilityMap[latitude])):
                self.probabilityMap[latitude][longitude] = self.logToProbability(
                    self.mapa[latitude][longitude])
        return self

    def updateProbabilityMapPlanning(self):
        for latitude in range(len(self.probabilityMap)):
            for longitude in range(len(self.probabilityMap[latitude])):
                self.probabilityMap[latitude][longitude] = self.mapa[latitude][longitude]
        return self

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
            # localHitTemp.append([theta[scanIndex], iteration['scan'][scanIndex]])
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

        # Robot coordinate X is Y coordinate on our map. Same for Robot Y => Our X
        sensorPosition = data['pose']
        sensorPosition[2] = np.deg2rad(sensorPosition[2])
        tempSensorPosition = sensorPosition[:]

        sensorPosition[0] += 0.18 * math.cos(sensorPosition[2])
        sensorPosition[1] += 0.18 * math.sin(sensorPosition[2])

        scanData = self.convertScanData(data['scan'])
        globalPoints = self.getGlobalHitpoints(sensorPosition, scanData)
        # #######################
        # TODO Delete after tests
        # #######################
        plt2.scatter([x[0] for x in globalPoints], [x[1] for x in globalPoints], c='blue')
        plt2.scatter(sensorPosition[0], sensorPosition[1], c='red')
        plt2.scatter(tempSensorPosition[0], tempSensorPosition[1], c='green')
        plt2.xlabel('x')
        plt2.ylabel('y')
        axes = plt2.gca()
        axes.set_xlim([-4, 4])
        axes.set_ylim([-4, 4])
        plt2.grid()
        plt2.show()
        #########################

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

        return True