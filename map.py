#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import math
import numpy as np

np.seterr(divide='ignore', invalid='ignore')


# latitude - szerokość - x
# longitude - dlugosc geo. - y


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
        self.mapa = []
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
        # print(math.floor((self.world_latitude/2+x)/self.cell_size) , math.floor((self.world_longitude/2+y)/self.cell_size))
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
        # print(math.floor((self.world_latitude/2+x)/self.cell_size) , math.floor((self.world_longitude/2+y)/self.cell_size))

        if isIndex:
            return self.mapa[x][y]
        else:
            index = self.get_cell_index(x, y)
            return self.mapa[index[0]][index[1]]

    def get_cell_index(self, x, y):
        longitude = int(
            round((self.world_longitude / 2.0 + y) / self.cell_size))
        latitude = int(round((self.world_latitude / 2.0 + x) / self.cell_size))
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

    def getProbability(self, coordinates):
        return self.logToProbability(self.get_cell(coordinates[0], coordinates[1]))

    def updateProbabilityMap(self):
        for latitude in range(len(self.probabilityMap)):
            for longitude in range(len(self.probabilityMap[latitude])):
                self.probabilityMap[latitude][longitude] = self.logToProbability(
                    self.mapa[latitude][longitude])
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

    def getGlobalHitpoints(self, robotPosition, scanData):
        pointsHitGlobal = list()

        for scanIndex in range(len(scanData)):
            # localHitTemp.append([theta[scanIndex], iteration['scan'][scanIndex]])
            if not np.isinf(scanData[scanIndex][1]) and not np.isnan(scanData[scanIndex][1]):
                scanPointX = robotPosition[1] + \
                    scanData[scanIndex][1] * math.sin(robotPosition[2] + scanData[scanIndex][0]) + \
                    0.18 * math.sin(robotPosition[2])
                scanPointY = robotPosition[0] + \
                    scanData[scanIndex][1] * math.cos(robotPosition[2] + scanData[scanIndex][0]) + \
                    0.18 * math.cos(robotPosition[2])

                scanPointX = -scanPointX
                pointsHitGlobal.append([scanPointX, scanPointY])

        return pointsHitGlobal

    def checkLineCellIntersect(self, cell, beam):
        # intersects = [False, False, False]

        halfCellSize = self.cell_size / 2.0
        cellCoords = self.get_cell_coords(cell[0], cell[1])

        p1 = [cellCoords[0] - halfCellSize, cellCoords[1] - halfCellSize]
        p2 = [cellCoords[0] - halfCellSize, cellCoords[1] + halfCellSize]
        p3 = [cellCoords[0] + halfCellSize, cellCoords[1] + halfCellSize]
        p4 = [cellCoords[0] + halfCellSize, cellCoords[1] - halfCellSize]

        sline = list()
        sline.append(np.polyfit([p1[0], p3[0]], [p1[1], p3[1]], 1))
        sline.append(np.polyfit([p2[0], p3[0]], [p2[1], p3[1]], 1))
        sline.append(np.polyfit([p1[0], p4[0]], [p1[1], p4[1]], 1))

        for i in range(3):
            A = np.array([[-1 * beam[0], 1], [-1 * sline[i][0], 1]])
            B = np.array([beam[1], sline[i][1]])
            try:
                x, y = np.linalg.solve(A, B)

                if p1[0] <= x <= p3[0]:
                    # intersects[i] = True
                    return True
                    break
            except np.linalg.linalg.LinAlgError as error:
                if self.debug:
                    print("Algebra error")

        return False
        # if all(x is False for x in intersects):
        #     return False
        # else:
        #     if intersects[0] is True and intersects[1] is False and intersects[2] is False:
        #         return ["left", "right"]
        #     elif intersects[0] is True and intersects[1] is True and intersects[2] is False:
        #         return ["up", "right"]
        #     elif intersects[0] is True and intersects[1] is False and intersects[2] is True:
        #         return ["down", "left"]
        #     elif intersects[0] is True and intersects[1] is True and intersects[2] is True:
        #         return ["up", "down"]

    def updateHitCells(self, data):
        # function to update log map
        # [in] data - list of data from one iterations. Has sublists 'pose' and 'scan'.
        sensorPosition = data['pose']
        sensorPosition[2] = np.deg2rad(sensorPosition[2])
        scanData = self.convertScanData(data['scan'])
        globalPoints = self.getGlobalHitpoints(sensorPosition, scanData)

        # TODO sprawdzic, czy w dobra strone sie dodalo :)
        sensorPosition[0] += 0.18 * math.sin(sensorPosition[2])
        sensorPosition[1] += 0.18 * math.cos(sensorPosition[2])

        pathPoints = dict()  # points through which the beam passes

        for globalPoint in globalPoints:
            # path from sensorPosition to globalPoint
            fix, fiy = self.get_cell_index(
                globalPoint[0], globalPoint[1])  # final point indexes
            six, siy = self.get_cell_index(
                sensorPosition[0], sensorPosition[1])  # sensor position indexes
            point = [six, siy]

            key = str(fix) + "." + str(fiy)
            pathPoints[key] = 1

            try:
                beam = np.polyfit([sensorPosition[0], globalPoint[0]], [
                                  sensorPosition[1], globalPoint[1]], 1)
                # direction of iteration through cells on thupdateHitCellse map
                dPos = ["", ""]

                if globalPoint[0] < sensorPosition[0]:
                    dPos[0] = -1
                else:
                    dPos[0] = 1

                if globalPoint[1] >= sensorPosition[1]:
                    dPos[1] = 1
                else:
                    dPos[1] = -1

                tempPoint = list(point)
                while tempPoint[0] != fix or tempPoint[1] != fiy:
                    tempPoint = list(point)
                    tempPoint[0] += dPos[0]
                    ifHit = self.checkLineCellIntersect(tempPoint, beam)
                    if ifHit is False:
                        tempPoint = list(point)
                        tempPoint[1] += dPos[1]
                        ifHit = self.checkLineCellIntersect(tempPoint, beam)

                    if ifHit is True:
                        point = list(tempPoint)
                        key = str(tempPoint[0]) + "." + str(tempPoint[1])
                        if key not in pathPoints:
                            pathPoints[key] = 0
                    else:
                        if self.debug:
                            print("Solution not found after point : ")
                            print(point)
                            print("For global point : ")
                            print(globalPoint)
                            print([fix, fiy])
                            print("And sensor position : ")
                            print(sensorPosition)
                        break

            except np.linalg.linalg.LinAlgError as error:
                if self.debug:
                    print ("Algebra error")

        for key, value in pathPoints.iteritems():
            pIndex = key.split('.')
            logValue = self.get_cell(int(pIndex[0]), int(pIndex[1]), True) + self.inverseSensorModel(
                value) - self.initialLog
            self.update_map(int(pIndex[0]), int(pIndex[1]), logValue, True)

        return True
        # if self.measurementInPerceptionField(globalPoint):
        #     logValue = self.get_cell(globalPoint[0], globalPoint[1]) + self.inverseSensorModel() - self.initialLog
        #     self.update_map(globalPoint[0], globalPoint[1], logValue)

# class hit_map(world_map):
#     def __init__(self, longitude, latitude, cell_size):
#         self.cell_size = cell_size
#         self.world_longitude = longitude
#         self.world_latitude = latitude
#
#     def initialize_map(self):
#         self.mapa = []
#         self.probabilityMap = []
#         for i in range(int(math.ceil(self.world_longitude / self.cell_size))):
#             row = []
#             for j in range(int(math.ceil(self.world_latitude / self.cell_size))):
#                 row.append(0)
#             self.mapa.append(row)
#         return self
