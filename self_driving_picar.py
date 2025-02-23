import picar_4wd as fc
import numpy as np
import math
import heapq
import time
import threading
from vilib import Vilib

np.set_printoptions(threshold=np.inf, linewidth=np.inf)

### Some global variables

ANGLE_RANGE = 60                        # Range of measurements.
ANGLE_INCREMENT = 4                     # Angle traversed between measurements.
N = ANGLE_RANGE // ANGLE_INCREMENT + 1  # Number of measurements.
SENSOR_RANGE = 50                       # Distance sensor can reliably measure in cm.
MAP_SIZE = 200                          # Map size in cm. Map will be MAP_SIZE*MAP_SIZE.
OBSTACLE_PADDING = 8                    # Padding (in cm) used to mark on all sides of obstacles.
carPos = (MAP_SIZE // 2, 0)             # Car position (x,y).
carDirection = (0, 1)                   # Car direction as in (dx, dy).
map = np.zeros((MAP_SIZE, MAP_SIZE)     # Initialize empty map
                , dtype=np.int32)
pathTaken = []
stopSignsThread = None
stopSignFound = False
status = 'waiting'


# Function to print a condensed map on terminal
def printMap(path=[], goal=None):
    tmp = np.copy(map)
    for i, point in enumerate(path):
        tmp[point[1], point[0]] = 2
    for i, point in enumerate(pathTaken):
        tmp[point[1], point[0]] = 3
    if goal:
        tmp[goal[1], goal[0]] = 4
    tmp[carPos[1], carPos[0]] = 5
    condensedMap = np.max(tmp.reshape(50, 4, 50, 4), axis=(1, 3)).astype(int)
    #print('\n', condensedMap)

    # Convert the array to string and replace 0 with "."
    for row in condensedMap:
        print(''.join(['. ' if cell == 0 else f"{cell}{cell}" for cell in row]))

def getDistanceReadings():
    # Initialize numpy array
    readings = np.zeros(N, dtype=np.int32) 

    # Give servo enough time to move to initial angle
    fc.servo.set_angle(-ANGLE_RANGE // 2) 
    time.sleep(0.5) 

    # Do a sweep of surrounding measurements
    for index in range(0, N):
        angle = index*ANGLE_INCREMENT-ANGLE_RANGE//2
        fc.servo.set_angle(angle)
        time.sleep(0.1)
        sensorReading = fc.us.get_distance()
        readings[index] = min(sensorReading, SENSOR_RANGE) if sensorReading >= 0 else SENSOR_RANGE

    return readings

# Helper function to calculate obstacle coordinate based on angle, distance, and direction
def getObstacleCoords(distance, angle):
    angleRAD = math.radians(angle)
    dx = int(distance * math.sin(angleRAD))
    dy = int(distance * math.cos(angleRAD))

    if carDirection == (1, 0):
        dx, dy = dy, -dx
    elif carDirection == (-1, 0):
        dx, dy = -dy, dx
    elif carDirection == (0, -1):
        dx, dy = -dx, -dy

    x, y = carPos[0] + dx, carPos[1] + dy

    return max(0, min(MAP_SIZE-1, x)), max(0, min(MAP_SIZE-1, y))
    
# Helper function to mark points on the map with a padding
def addPointWithPadding(x, y, marker = 1, padding=OBSTACLE_PADDING):
    map[max(0,y-padding) : min(MAP_SIZE-1,y+padding),
        max(0,x-padding) : min(MAP_SIZE-1,x+padding)] = marker

def drawLineBetweenPoints(x1, x2, y1, y2, marker=1, padding=OBSTACLE_PADDING):
    # Numpy doc about .linspace: https://numpy.org/doc/stable/reference/generated/numpy.linspace.html
    xs = np.linspace(x1, x2, max(abs(y2-y1), abs(x2-x1)))
    ys = np.linspace(y1, y2, max(abs(y2-y1), abs(x2-x1)))
    
    # Mark each point on the line with marker and padding
    for i in range(len(xs)):
        addPointWithPadding(int(xs[i]), int(ys[i]), marker, padding)


def updateMap(readings):
    # The map will be a 2D array of intergers,
    # where the car is at the middle of row 0 
    # and obstacles appear on positive y-axis.


    # Used angles
    angles = np.arange(-ANGLE_RANGE // 2, ANGLE_RANGE // 2 + 1, ANGLE_INCREMENT)
    #print(angles)

    # Tmp variables for filling edges between obstacles
    prevObsX, prevObsY = -1, -1

    # Fill map with obstacles as '1'
    for i, distance in enumerate(readings):
        # Get obstacle coordinates
        angle = angles[i]
        x, y = getObstacleCoords(distance, angle)
        
        # Clear any obstacles from map first
        if -20 <= angle <= 20:
            drawLineBetweenPoints(carPos[0], x, carPos[1], y, marker = 0, padding=2)

        # If no is obstacle detected, prevent line from 
        # being drawn by setting previous obstacle coords to -1.
        # But do empty the map with zeroes from the sensor reading area.
        if distance == SENSOR_RANGE:
            prevObsX, prevObsY = -1, -1
            continue
        
        # If obstacle found
        if distance > 0:
            # Mark obstacle with ones, including padding
            addPointWithPadding(x, y)
            #print(f"({x}, {y}, {angle}°)  ", end="")
     
            # Draw line between current and previous obstacle points
            if prevObsX != -1:
                drawLineBetweenPoints(prevObsX, x, prevObsY, y)
            prevObsX, prevObsY = x, y 


# Function for Manhattan distance between two points
def manhattanDistance(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def directionCost(targetDirection, distance):
    # Calculate the difference in direction and apply a distance-based scaling factor
    if carDirection != targetDirection:
        scalingFactor = max(0, 3 - (distance / 20))
        return scalingFactor
    return 0

# Function for A* algorithm
# Returns a path of nodes to target if path found
def astar(target):
    rows, cols = map.shape
    openSet = [(0, carPos)]
    heapq.heapify(openSet)
    pathSet = {}
    gScore = {carPos: 0}
    fScore = {carPos: manhattanDistance(carPos, target)}
    
    while openSet:
        current = heapq.heappop(openSet)[1]
        
        if current == target:
            path = []
            while current in pathSet:
                path.append(current)
                current = pathSet[current]
            path.append(carPos)
            return path[::-1]
        
        neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        for dx, dy in neighbors:
            neighbor = (current[0] + dx, current[1] + dy)
            distance = manhattanDistance(carPos, neighbor)
            tentativeGScore = gScore[current] + 1 \
                + directionCost((dx, dy), distance) # Add a cost for changing direction immidiately
            if 0 <= neighbor[0] < cols and 0 <= neighbor[1] < rows and map[neighbor[1], neighbor[0]] != 1:
                if neighbor not in gScore or tentativeGScore < gScore[neighbor]:
                    pathSet[neighbor] = current
                    gScore[neighbor] = tentativeGScore
                    fScore[neighbor] = tentativeGScore + manhattanDistance(neighbor, target)
                    if neighbor not in (item[1] for item in openSet):
                        heapq.heappush(openSet, (fScore[neighbor], neighbor))
    
    return []

def turn(nextMove):
    global carDirection

    # Calculate and perform turn if needed
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    a = directions.index(carDirection)
    b = directions.index(nextMove)
    turnDiff = (b - a) % 4

    if turnDiff == 1: 
        # Turn left
        print("Turn left")
        fc.turn_left(75)
        time.sleep(0.6)
    elif turnDiff == 3:
        # Turn right
        print("Turn right")
        fc.turn_right(75)
        time.sleep(0.6)
    elif turnDiff == 2:
        # Turn left
        print("Turn 180")
        fc.turn_left(75)
        time.sleep(1.2)
    fc.stop()
    
    # Update car's new direction
    carDirection = nextMove
        
def stopSignRecognition():
    global stopSignFound
    print("Started to look for stop signs")
    while True:
        time.sleep(0.01)
        if status == 'done':
            break

        if status != 'moving':
            continue

        if Vilib.traffic_sign_obj_parameter['t'] == 'stop':
            stopSignFound = True
            time.sleep(5)

def followPath(path, steps=10):
    global carPos, stopSignFound

    fc.servo.set_angle(0)
    prevDiff = carDirection

    for i in range(min(len(path)-1,steps-1)):
        dx = path[i+1][0] - path[i][0]
        dy = path[i+1][1] - path[i][1]
        diff = (dx, dy)
        if prevDiff != diff:
            turn(diff) # Turn if direction changes
            break # Break from loop after turn to re-scan environment
        else:

            # If stop sign is no longer in view, stop for 2 seconds
            if stopSignFound:
                print("Stopping for a STOP sign", end=' ')
                fc.stop()
                time.sleep(1)
                print("...")
                time.sleep(1)
                stopSignFound = False
            
            # Move forward roughly 1cm if no obstacle
            fc.forward(10)
            time.sleep(0.03)
            pathTaken.append(carPos)
            carPos = (carPos[0] + dx, carPos[1] + dy)
    fc.stop()
    
def carAtTarget(target):
    if target[0]-5 <= carPos[0] <= target[0]+5 and target[1]-5 <= carPos[1] <= target[1]+5:
        return True
    return False

def main():
    global map, carPos, carDirection, pathTaken, stopSignsThread, status

    # Initialize camera and traffic sign detection
    Vilib.camera_start(vflip=True, hflip=True) # , size=(640, 480) , size=(1280, 720)
    Vilib.show_fps()
    Vilib.traffic_detect_switch(True)
    Vilib.display(local=False, web=True)
    time.sleep(1)
    stopSignsThread = threading.Thread(target=stopSignRecognition, name="Thread1")
    stopSignsThread.start()
    time.sleep(0.05)

    targets = [(MAP_SIZE - 160, MAP_SIZE - 80), (MAP_SIZE - 100, MAP_SIZE - 80)]
    target = targets[0]
    i = 0
    path = []
    failedNavigationAttempts = 0

    while status != 'done':
        if status == 'scanning':
            print("Scanning surroundings...")
            readings = getDistanceReadings()
            #print(readings)
            updateMap(readings)
            printMap(path, target)
            status = 'navigating'

        elif status == 'navigating':
            print("Finding route...")
            path = astar(target)
            if path == []:
                status = 'no-path'
                failedNavigationAttempts += 1
                continue
            failedNavigationAttempts = 0
            printMap(path, target)
            status = 'moving'

        elif status == 'moving':
            print("Following route...")
            followPath(path, 40)
            if carAtTarget(target):
                printMap()
                print("MADE IT TO THE TARGET!!!")
                status = 'waiting'
            else:
                status = 'scanning'
        
        elif status == 'waiting' and i < 2:
            pathTaken = []
            map = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.int32)
            carPos = (MAP_SIZE // 2, 0)
            carDirection = (0, 1)
            target = targets[i]
            printMap(goal=target)
            input("Press ENTER to start navigating.")
            status = 'scanning'
            i += 1

        elif status == 'no-path':
            if failedNavigationAttempts == 3:
                status = 'waiting'
                continue
            print("NO PATH FOUND. Turning 180.")
            turn((-carDirection[0], -carDirection[1]))
            status = 'scanning'

        else:
            status = 'done'
    
    print("DONE!")



if __name__ == "__main__":
    
    try: 
        main()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"\033[31mERROR: {e}\033[m")
    finally: 
        fc.stop()
        Vilib.camera_close()
        status = 'done'
        stopSignsThread.join()