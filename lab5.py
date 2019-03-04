import math

class world_map:
	cell_size=1
	world_longitude=0
	world_latitude=0
	mapa=[]

	def __init__(self, longitude, latitude, cell_size):		
		self.cell_size=cell_size
		self.world_longitude=int(math.ceil(longitude/cell_size))
		self.world_latitude=int(math.ceil(latitude/cell_size))
	
	def initialize_map(self):
		initialProbabilityHit = 0.5
		initialProbabilityMiss = 1 - initialProbabilityHit
		value = math.log(initialProbabilityHit/initialProbabilityMiss)
		self.mapa=[]
		for i in range(self.world_longitude):
			row=[]
			for j in range(self.world_latitude): 
				row.append(value)
			self.mapa.append(row)
		print(self.mapa)
		return self.mapa

	def update_map(self, x, y, value):
		longitude=int(math.ceil(self.world_longitude/2+(y/self.cell_size)))
		latitude=int(math.ceil(self.world_latitude/2+(x/self.cell_size)))
		print((longitude,latitude))
		self.mapa[longitude][latitude]=value
		return self.mapa
		
	def get_cell(self, x,y):
		longitude=int(math.ceil(self.world_longitude/2+(y/self.cell_size)))
		latitude=int(math.ceil(self.world_latitude/2+(x/self.cell_size)))
		return self.mapa[longitude][latitude]
	
	def getProbability(self, coordinates):
		return 1 - 1/(1+math.exp(self.getCell(coordinates[0],coordinates[1])))
		
	def inverseSensorModel():
		scannerTrust = 0.9
		scannerDoubt = 1 - scannerTrust
		return log(scannerDoubt/sannerTrust)
		
	def measurementInPerceptionField(iMeasurement):
		return True
		
	def updateHitCells(self, measurements):
		for iMeasurement in measurements:
			if self.measurementInPerceptionField(iMeasurement):
				LogValue = self.get_cell(iMeasurement[0], iMeasurement[1]) + self.inverseSensorModel() - initialLog
				self.update_map(iMeasurement[0], iMeasurement[1], logValue)
