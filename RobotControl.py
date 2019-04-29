
# 0. Enlarge obstacles by robot radius (/)
# 1. Check robot position in cell cooordinates
# 2. Plan path (StartPoint = RobotPosition, GoalPoint = any) (/)
# 3. Rotate from robot orientation to first segment direction
# 4. Move forward until direction changes
# 5. Rotate from orientation to next segment direction
# 6. Move forward
# 7. Repeat until goal reached or the world ends
# *** Optimise path to turn while driving

from map import world_map
import PathPlanning
import numpy as np
from drive import RosAriaDriver
robot=RosAriaDriver('/PIONIER6')

probabilityThreshold = 0.75
robotRadius = 0.1

def ControlRobot(probabilityMap, robotPosition, goalPoint):

	PathPlanning.enlargeObstacles(probabilityMap)
	startPoint = robotPosition
	WaveMap=PathPlanning.blastWave(probabilityMap, startPoint, goalPoint)
	PathPoints=PathPlanning.findPathPoints(WaveMap, startPoint, goalPoint)



	print(PathPoints)
	p=PathPoints[::-1]

	next_position=p.pop()
	actual_position=next_position
	rotation=0
	last_rotation=robot.getPose()[2]
	while p:
		actual_position = next_position
		next_position=p.pop()
		x=(next_position[0]-actual_position[0])/wm.cell_size
		y=(next_position[1]-actual_position[1])/wm.cell_size
		distance = wm.cell_size*(1 + 0.41421356237*(x!=0)*(y!=0))
		last_rotation=rotation
		if y==0:
			rotation=(np.abs(x - 1) * 90)

		else:
			rotation=((y==-1)*360+y*(np.abs(y)+np.abs(x-1))*45)	#bez ujemnych
			rotation=(y * (np.abs(y) + np.abs(x - 1)) * 45)  # z ujemnymi

		# setting speed
		robot.SetSpeed(distance,rotation-last_rotation,1)
		# setting distances
		if (rotation-last_rotation!=0):
			robot.Rotate(rotation-last_rotation)
		robot.GoTo(distance)

	#3,4,5,6,7,...

if __name__ == '__main__':
	wm = world_map(30, 30, 0.5)
	wm.initialize_map()

	PathPlanning.addObstacles(wm)

	wm.updateProbabilityMap()

	PathPlanning.enlargeObstacles(wm)

	startPoint = [0, -10]
	goalPoint = [-14, 14]

	WaveMap = PathPlanning.blastWave(wm, startPoint, goalPoint)
	PathPoints = PathPlanning.findPathPoints(WaveMap, startPoint, goalPoint)
	PathPlanning.plotWavePath(WaveMap,PathPoints)

	ControlRobot(wm,startPoint,goalPoint)
