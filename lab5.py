import math


class world_map:
    cell_size = 1
    world_longitude = 0
    world_latitude = 0
    initialProbability = 0;
    initialLog = 0
    mapa = []
    probabilityMap = []

    def __init__(self, longitude, latitude, cell_size):
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

    def update_map(self, x, y, value):
        # function updating map with given value at given position
        # [in] x - latitude in robot coordinates e.g -21
        # [in] y - longitude in robot coordinates e.g .37
        # [in] value - value which is written to the cell
        # print(math.floor((self.world_latitude/2+x)/self.cell_size) , math.floor((self.world_longitude/2+y)/self.cell_size))
        longitude = int(math.floor((self.world_longitude / 2 + y) / self.cell_size))
        latitude = int(math.floor((self.world_latitude / 2 + x) / self.cell_size))
        self.mapa[longitude][latitude] = value
        return self

    def get_cell(self, x, y):
        # function to read cell value at given position
        # [in] x - latitude in robot coordinates e.g -21
        # [in] y - longitude in robot coordinates e.g .37
        # [out] - value of the cell
        # print(math.floor((self.world_latitude/2+x)/self.cell_size) , math.floor((self.world_longitude/2+y)/self.cell_size))

        longitude = int(math.floor((self.world_longitude / 2 + y) / self.cell_size))
        latitude = int(math.floor((self.world_latitude / 2 + x) / self.cell_size))

        return self.mapa[longitude][latitude]

    def logToProbability(self, logValue):
        return 1 - 1 / (1 + math.exp(logValue))


    def getProbability(self, coordinates):
        return self.logToProbability(self.get_cell(coordinates[0], coordinates[1]))

    def updateProbabilityMap(self):
        for longitude in range(len(self.probabilityMap)):
            for latitude in range(len(self.probabilityMap[longitude])):
                self.probabilityMap[longitude][latitude] = self.logToProbability(self.mapa[longitude][latitude]);
        return self

    def inverseSensorModel(self):
        scannerTrust = 0.9
        scannerDoubt = 1 - scannerTrust
        return math.log(scannerTrust / scannerDoubt)

    def measurementInPerceptionField(self, iMeasurement):
        return True

    def updateHitCells(self, measurements):

        for iMeasurement in measurements:
            if self.measurementInPerceptionField(iMeasurement):
                logValue = self.get_cell(iMeasurement[0], iMeasurement[1]) + self.inverseSensorModel() - self.initialLog
                self.update_map(iMeasurement[0], iMeasurement[1], logValue)
