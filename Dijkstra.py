import numpy as np
import math
from collections import defaultdict
from heapq import *
import cv2


class Node(object):

	def __init__(robot, start, dest, location, clearance, radius):
		robot.start = start
		robot.dest = dest
		robot.location = location
		robot.clearance = clearance
		robot.radius = radius
		robot.cost = 0
		robot.y = 0
		robot.x = 0
		robot.totalNrows = 250
		robot.totalNCols = 400

	def check(robot, other):
		return robot.location == other.location

	def Validity(robot, presentRow, presentCol):
		sum_rc = robot.clearance + robot.radius
		return (presentRow >= (1 + sum_rc) and presentRow <= (250 - sum_rc) and presentCol >= (1 + sum_rc) and presentCol <= (400 - sum_rc))


	def obstacle(robot, row, col):
		sum_rc = robot.clearance + robot.radius

		# check circle
		dist1 = ((row - 185) * (row - 185) + (col - 300) * (col - 300)) - ((20*sum_rc) * (20*sum_rc)) 

		# check polygon 
		first = (row - ((0.316)* col) - 173.6 )
		fourth = (row - (0.857* col) - 111.42)
		fifth = (row + (0.1136*col) - 189.09)
		dist3 = 1 
		if(first <= 0 and fourth >=0 and fifth >=0):
			dist3 = 0
        
		second = (row + (1.23 * col) -229.34 )
		fifth = (row + (0.1136*col) - 189.09)
		third = (row + ( 3.2 * col) - 436)
		dist4 = 1
		if(fifth <= 0 and third <=0 and second >=0):
			dist4 = 0
        
		#check hexagon (tried my best but due to an issue printed rombus)
		fst = (row - 0.577*col - 24.97)
		thr = (col - 235)
		six = (col - 165)
		fou = (row - 0.577*col + 55.82)
		dist7 = 1
		dist8 = 1

		if(fst <= 0 and thr <= 0 and six >= 0 and fou >= 0):
			dist7 = 0
			dist8 = 0
        
		if(dist1 <= 0 or dist3 == 0 or dist4 == 0 or dist7 == 0 or dist8 == 0 ):
			return True
		return False

	def MovU(robot, presentRow, presentCol):
		if(robot.Validity(presentRow - 1, presentCol) and robot.obstacle(presentRow - 1, presentCol) == False):
			robot.x = 0
			robot.y = 1
			robot.cost = 1
			return True
		else:
			return False

	def MovUR(robot, presentRow, presentCol):
		if(robot.Validity(presentRow+1, presentCol+1) and robot.obstacle(presentRow +1, presentCol + 1) == False):
			robot.x = 1
			robot.y = 1
			robot.cost = np.sqrt(2)
			return True
		else:
			return False

	def MovR(robot, presentRow, presentCol):
		if(robot.Validity(presentRow+1, presentCol) and robot.obstacle(presentRow +1, presentCol) == False):
			robot.x = 1 
			robot.y = 0
			robot.cost = 1
			return True
		else:
			return False

	def MovDR(robot, presentRow, presentCol):
		if(robot.Validity(presentRow+1, presentCol-1) and robot.obstacle(presentRow + 1, presentCol - 1) == False):
			robot.x = 1 
			robot.y = -1
			robot.cost = np.sqrt(2)
			return True
		else:
			return False
	def MovD(robot, presentRow, presentCol):
		if(robot.Validity(presentRow, presentCol - 1) and robot.obstacle(presentRow, presentCol - 1) == False):
			robot.x = 0 
			robot.y = -1
			robot.cost = 1
			return True
		else:
			return False

	def MovDL(robot, presentRow, presentCol):
		if(robot.Validity(presentRow - 1, presentCol - 1) and robot.obstacle(presentRow - 1, presentCol - 1) == False):
			robot.x = -1 
			robot.y = -1
			robot.cost = np.sqrt(2)
			return True
		else:
			return False

	def MovL(robot, presentRow, presentCol):
		if(robot.Validity(presentRow - 1, presentCol) and robot.obstacle(presentRow - 1, presentCol) == False):
			robot.x = -1 
			robot.y = 0
			robot.cost = 1
			return True
		return False

	def MovUL(robot, presentRow, presentCol):
		if(robot.Validity(presentRow - 1, presentCol+1) and robot.obstacle(presentRow - 1, presentCol + 1) == False):
			robot.x = -1 
			robot.y = 1
			robot.cost = np.sqrt(2)
			return True
		else:
			return False

	def checkDest(robot,presentRow, presentCol):
		if(presentRow== robot.dest[0] and presentCol == robot.dest[1]):
			print("Reached Destination")
			return True
		else:
			return False


	def main(robot):
		cost_Map = {}
		reached_nodes = {}
		track = {}

		for row in np.arange(1, robot.totalNrows + 1, 1):
			for col in np.arange(1, robot.totalNCols + 1, 1): 
				cost_Map[(row, col)] = float('inf')
				reached_nodes[(row, col)] = False
				track[(row, col)] = -1

		verified = []
		queue = []

		heappush(queue, (0, robot.start))
		cost_Map[robot.start] = 0

		while(len(queue)) > 0:
			heapify(queue)
			_, PresentNode = heappop(queue)
			reached_nodes[PresentNode] = True
			verified.append(PresentNode)
			

			if(robot.checkDest(PresentNode[0], PresentNode[1]) == True):
				break

			if(robot.MovU(PresentNode[0],PresentNode[1]) and reached_nodes[(PresentNode[0] + robot.x , PresentNode[1] + robot.y)] == False and (cost_Map[(PresentNode[0]+ robot.x, PresentNode[1] + robot.y)] > cost_Map[(PresentNode)] + robot.cost)):
				cost_Map[(PresentNode[0] + robot.x, PresentNode[1] + robot.y)] = cost_Map[(PresentNode)] + robot.cost
				track[(PresentNode[0] + robot.x, PresentNode[1] + robot.y)] = PresentNode
				heappush(queue,(cost_Map[(PresentNode[0] + robot.x, PresentNode[1] + robot.y)], (PresentNode[0] + robot.x, PresentNode[1] + robot.y)))

			if(robot.MovUR(PresentNode[0],PresentNode[1]) and reached_nodes[(PresentNode[0] + robot.x ,PresentNode[1] + robot.y)] == False and (cost_Map[(PresentNode[0]+ robot.x, PresentNode[1] + robot.y)] > cost_Map[(PresentNode)] + robot.cost)):
				cost_Map[(PresentNode[0] + robot.x, PresentNode[1] + robot.y)] = cost_Map[(PresentNode)] + robot.cost
				track[(PresentNode[0] + robot.x, PresentNode[1] + robot.y)] = PresentNode
				heappush(queue,(cost_Map[(PresentNode[0] + robot.x, PresentNode[1] + robot.y)], (PresentNode[0] + robot.x, PresentNode[1] + robot.y)))

			if(robot.MovR(PresentNode[0],PresentNode[1]) and reached_nodes[(PresentNode[0] + robot.x ,PresentNode[1] + robot.y)] == False and (cost_Map[(PresentNode[0]+ robot.x, PresentNode[1] + robot.y)] > cost_Map[(PresentNode)] + robot.cost)):
				cost_Map[(PresentNode[0] + robot.x, PresentNode[1] + robot.y)] = cost_Map[(PresentNode)] + robot.cost
				track[(PresentNode[0] + robot.x, PresentNode[1] + robot.y)] = PresentNode
				heappush(queue,(cost_Map[(PresentNode[0] + robot.x, PresentNode[1] + robot.y)], (PresentNode[0] + robot.x, PresentNode[1] + robot.y)))

			if(robot.MovDR(PresentNode[0],PresentNode[1]) and reached_nodes[(PresentNode[0] + robot.x ,PresentNode[1] + robot.y)] == False and (cost_Map[(PresentNode[0]+ robot.x, PresentNode[1] + robot.y)] > cost_Map[(PresentNode)] + robot.cost)):
				cost_Map[(PresentNode[0] + robot.x, PresentNode[1] + robot.y)] = cost_Map[(PresentNode)] + robot.cost
				track[(PresentNode[0] + robot.x, PresentNode[1] + robot.y)] = PresentNode
				heappush(queue,(cost_Map[(PresentNode[0] + robot.x, PresentNode[1] + robot.y)], (PresentNode[0] + robot.x, PresentNode[1] + robot.y)))

			if(robot.MovD(PresentNode[0],PresentNode[1]) and reached_nodes[(PresentNode[0] + robot.x ,PresentNode[1] + robot.y)] == False and (cost_Map[(PresentNode[0]+ robot.x, PresentNode[1] + robot.y)] > cost_Map[(PresentNode)] + robot.cost)):
				cost_Map[(PresentNode[0] + robot.x, PresentNode[1] + robot.y)] = cost_Map[(PresentNode)] + robot.cost
				track[(PresentNode[0] + robot.x, PresentNode[1] + robot.y)] = PresentNode
				heappush(queue,(cost_Map[(PresentNode[0] + robot.x, PresentNode[1] + robot.y)], (PresentNode[0] + robot.x, PresentNode[1] + robot.y)))

			if(robot.MovDL(PresentNode[0],PresentNode[1]) and reached_nodes[(PresentNode[0] + robot.x ,PresentNode[1] + robot.y)] == False and (cost_Map[(PresentNode[0]+ robot.x, PresentNode[1] + robot.y)] > cost_Map[(PresentNode)] + robot.cost)):
				cost_Map[(PresentNode[0] + robot.x, PresentNode[1] + robot.y)] = cost_Map[(PresentNode)] + robot.cost
				track[(PresentNode[0] + robot.x, PresentNode[1] + robot.y)] = PresentNode
				heappush(queue,(cost_Map[(PresentNode[0] + robot.x, PresentNode[1] + robot.y)], (PresentNode[0] + robot.x, PresentNode[1] + robot.y)))

			if(robot.MovL(PresentNode[0],PresentNode[1]) and reached_nodes[(PresentNode[0] + robot.x ,PresentNode[1] + robot.y)] == False and (cost_Map[(PresentNode[0]+ robot.x, PresentNode[1] + robot.y)] > cost_Map[(PresentNode)] + robot.cost)):
				cost_Map[(PresentNode[0] + robot.x, PresentNode[1] + robot.y)] = cost_Map[(PresentNode)] + robot.cost
				track[(PresentNode[0] + robot.x, PresentNode[1] + robot.y)] = PresentNode
				heappush(queue,(cost_Map[(PresentNode[0] + robot.x, PresentNode[1] + robot.y)], (PresentNode[0] + robot.x, PresentNode[1] + robot.y)))

			if(robot.MovUL(PresentNode[0],PresentNode[1]) and reached_nodes[(PresentNode[0] + robot.x ,PresentNode[1] + robot.y)] == False and (cost_Map[(PresentNode[0]+ robot.x, PresentNode[1] + robot.y)] > cost_Map[(PresentNode)] + robot.cost)):
				cost_Map[(PresentNode[0] + robot.x, PresentNode[1] + robot.y)] = cost_Map[(PresentNode)] + robot.cost
				track[(PresentNode[0] + robot.x, PresentNode[1] + robot.y)] = PresentNode
				heappush(queue,(cost_Map[(PresentNode[0] +robot.x, PresentNode[1] + robot.y)], (PresentNode[0] + robot.x, PresentNode[1] + robot.y)))


		verify = []
		destx = robot.dest[0]
		desty = robot.dest[1]

		for i in np.arange(destx - 1, destx + 1, 1):
			for j in np.arange(desty - 1, desty + 1, 1):
				verify.append(cost_Map[i,j])


		sol = float('inf')
		for x in range(len(verify)):
			if(verify[x] != sol):
				print("Way is present")

			NoWay = 1
			break
		if (NoWay == 0):
			print("No way")
			return (verified, [], cost_Map[destx,desty])

		print(cost_Map[destx, desty], "solution")
		outcome = (destx, desty)


		backtrack = []
		node = outcome
		while(track[node] != -1):
			backtrack.append(node)
			node = track[node]
		backtrack.append(robot.start)
		backtrack = list(reversed(backtrack))

		print(backtrack)
		return (verified, backtrack, cost_Map[destx, desty])



	def visualize(robot, verified, backtrack, track):
		fourcc = cv2.VideoWriter_fourcc(*'XVID')
		out = cv2.VideoWriter(str(track), fourcc, 20.0, (robot.totalNCols , robot.totalNrows))
		image = np.zeros((robot.totalNrows, robot.totalNCols , 3), dtype=np.uint8)
		count = 0
		for state in verified:
			image[int(robot.totalNrows - state[0]), int(state[1] - 1)] = (255, 150, 0)
			if(count%75 == 0):
				out.write(image)
			count = count + 1
			cv2.imshow('verified', image)
		count = 0
		for row in range(1, robot.totalNrows + 1):
			for col in range(1, robot.totalNCols  + 1):
				if(image[int(robot.totalNrows - row), int(col - 1), 0] == 0 and image[int(robot.totalNrows - row), int(col - 1), 1] == 0 and image[int(robot.totalNrows - row), int(col - 1), 2] == 0):
					if(robot.Validity(row, col) and robot.obstacle(row, col) == False):
						image[int(robot.totalNrows - row), int(col - 1)] = (154, 250, 0)
						if(count%75 == 0):
							out.write(image)
						count = count + 1
            
		if(len(backtrack) > 0):
			for state in backtrack:
				image[int(robot.totalNrows - state[0]), int(state[1] - 1)] = (0, 0, 255)
				out.write(image)
				cv2.imshow('outcome', image)
				cv2.waitKey(5)
                
		cv2.waitKey(0)
		cv2.destroyAllWindows()

# Requesting values from user 
print("Coordinates should be in b/w 250 x 400 range")
initialRow = int(input("Give the x coordinate of initial node "))
initialCol = int(input("Give the y coordinate of initial node "))
destRow = int(input("Give the y coordinate of destination node "))
destCol = int(input("Give the y coordinate of destination node "))
radius = int(input("Give the radius of robot "))
clearance = int(input("Give the clearance for robot "))

# take start and goal node as input
start = (initialRow, initialCol)
dest = (destRow, destCol)
final = Node(start, dest, None, clearance, radius)

if(final.Validity(start[0], start[1])):
    if(final.Validity(dest[0], dest[1])):
        if(final.obstacle(start[0],start[1]) == False):
            if(final.obstacle(dest[0], dest[1]) == False):
                (verified, backtrack, T_path) = final.main()
                final.visualize(verified, backtrack, "./program.avi")
                if(total_distance == float('inf')):
                    print("\nMinimal path found")
                else:
                    print("\nDistance of minimal path " + str(T_path))
            else:
                print("Destinantion node given is an obstacle ")
                
        else:
            print("Initial node given is an obstacle ")
            
    else:
        print("Goal node was outranged")
       	   
else:
  print("Initial node was outranged")
   
       
