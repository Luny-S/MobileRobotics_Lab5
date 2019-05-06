# 0. Enlarge obstacles by robot radius (/)
# 1. Check robot position in cell cooordinates
# 2. Plan path (StartPoint = RobotPosition, GoalPoint = any) (/)
# 3. Rotate from robot orientation to first segment direction
# 4. Move forward until direction changes
# 5. Rotate from orientation to next segment direction
# 6. Move forward
# 7. Repeat until goal reached or the world ends
# *** Optimise path to turn while driving

from OccupancyGrid import world_map
import PathPlanning

probabilityThreshold = 0.75
robotRadius = 0.1

def ControlRobot(probabilityMap, goalPoint, robotPosition):

    PathPlanning.enlargeObstacles(probabilityMap)
    #startPoint = getPose, poseToCell

	WaveMap = PathPlanning.blastWave(probabilityMap, startPoint, goalPoint)
	PathPoints = PathPlanning.findPathPoints(WaveMap, startPoint, goalPoint)

