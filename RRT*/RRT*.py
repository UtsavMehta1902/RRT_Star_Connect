# FOR THE FIRST IMAGE:
#green dot is at 50,50 with color combination BGR: 30,220,7
#and red dot at 550,550 with color combination: 9,10,250

# FOR THE SECOND IMAGE:
# corner red at 50,55 -- right red at 50,170 -- bottom red at 445,55
# corner green at 525,615 -- left green at -- 535,355 -- top green at 185,630

import random as rand
import cv2 as cv
import sys
import math
import time

# img = cv.imread("./img1.png")
img = cv.imread("./img2.png")
#also update the obstacle check function for the second image (on line 61)

if img is None:
    sys.exit("Could not read the image.")

SAFE_DISTANCE_FROM_OBSTACLE = 5
COST_OF_EDGE = 4
PRECISION_VALUE = 5000
IMG_WIDTH = img.shape[0]
IMG_HEIGHT = img.shape[1]

allowed_color = (0,0,0)
red_range_low = (0,0,0)
red_range_high = (50,50,255)
green_range_low = (0,0,0)
green_range_high = (50,255,50)

nodesFromStart = []
nodesFromEnd = []

#a class to store nodes evaluated and stores all edges related to it, also stores the path to reach that node
class nodes_evaluated():
    def __init__(self, abcissa = None, ordinate = None ,cost = None, parentNode = None):
        self.abcissa = abcissa
        self.ordinate = ordinate
        self.cost = cost
        self.parentNode = parentNode

class mainCode():

    #a function to start from starting point
    def start(self,startAbcissa, startOrdinate, cost):
        startNode = nodes_evaluated(startAbcissa, startOrdinate, cost)
        startNode.parentNode = None
        self.nodes.append(startNode)

    def __init__(self,nodes=[]):
        self.nodes = nodes

    def checkPoint(self, x, y):
        if(((img[x,y] == allowed_color).all()) == False):
            if((((img[x,y] <= green_range_high).all() and (img[x,y] >= green_range_low).all()) or ((img[x,y] <= red_range_high).all() and (img[x,y] >= red_range_low).all())) == False):
                return False
        return True

    def obstacleCheck(self,x,y):

        check = True
        for i in range(SAFE_DISTANCE_FROM_OBSTACLE):
            for j in range(SAFE_DISTANCE_FROM_OBSTACLE):
                check = self.checkPoint(x+i-1,y-1+j)
                if(check == False):
                    break 
            if(check == False):
                break

        return check

    #find the euclidistally nearest node
    def findNearestNode(self,randomAbcissa, randomOrdinate):
        gotcha = None
        condition = True
        minDist = 99999
        for aNode in self.nodes:
            distance = math.sqrt((randomAbcissa - aNode.abcissa)**2 + (randomOrdinate - aNode.ordinate)**2)
            if(distance < minDist): #can also put a condition for angle < 270 or so
                minDist = distance
                gotcha = aNode
        
        #then we find the advanced node i.e., a node in direction of random node, from the nearest node
        if(randomOrdinate - gotcha.ordinate != 0):
            thetha = math.atan((randomAbcissa - gotcha.abcissa)/(randomOrdinate - gotcha.ordinate))
            if(randomAbcissa - gotcha.abcissa >= 0 and randomOrdinate - gotcha.ordinate > 0):
                newNodeAbcissa = int(gotcha.abcissa + COST_OF_EDGE*math.sin(thetha))
                newNodeOrdinate = int(gotcha.ordinate + COST_OF_EDGE*math.cos(thetha))
            elif(randomAbcissa - gotcha.abcissa >= 0 and randomOrdinate - gotcha.ordinate < 0):
                newNodeAbcissa = int(gotcha.abcissa - COST_OF_EDGE*math.sin(thetha))
                newNodeOrdinate = int(gotcha.ordinate - COST_OF_EDGE*math.cos(thetha))
            elif(randomAbcissa - gotcha.abcissa <= 0 and randomOrdinate - gotcha.ordinate > 0):
                newNodeAbcissa = int(gotcha.abcissa + COST_OF_EDGE*math.sin(thetha))
                newNodeOrdinate = int(gotcha.ordinate + COST_OF_EDGE*math.cos(thetha))
            elif(randomAbcissa - gotcha.abcissa <= 0 and randomOrdinate - gotcha.ordinate < 0):
                newNodeAbcissa = int(gotcha.abcissa - COST_OF_EDGE*math.sin(thetha))
                newNodeOrdinate = int(gotcha.ordinate - COST_OF_EDGE*math.cos(thetha))
        else:
            newNodeAbcissa = gotcha.abcissa
            newNodeOrdinate = gotcha.ordinate + COST_OF_EDGE
        
        for aNode in self.nodes:
            if(newNodeAbcissa == aNode.abcissa and newNodeOrdinate == aNode.ordinate):
                condition = False
        
        newNodeCost = gotcha.cost + COST_OF_EDGE
        
        if(condition and (self.obstacleCheck(newNodeAbcissa, newNodeOrdinate) == True)):
            newNode = nodes_evaluated(newNodeAbcissa, newNodeOrdinate, newNodeCost)
            self.nodes.append(newNode)
            newNode.parentNode = gotcha
            return True, newNode
        else:
            return False, None
    
    # find a better base node, basically going for RRT*, this function also colours the nodes
    def findBetterNode(self, newNode, count):
        possible = None
        for aNode in self.nodes:
            if(abs((aNode.abcissa - newNode.abcissa) < 7) and (abs(aNode.ordinate - newNode.ordinate) < 7)):
                if(aNode.cost + math.sqrt((aNode.abcissa - newNode.abcissa)**2 + (aNode.ordinate - newNode.ordinate)**2) < newNode.cost):
                    newNode.cost = aNode.cost + int(math.sqrt((aNode.abcissa - newNode.abcissa)**2 + (aNode.ordinate - newNode.ordinate)**2))
                    possible = aNode

        if count%2 == 0:
            img[newNode.abcissa, newNode.ordinate] = [32,156,8]
        else:
            img[newNode.abcissa, newNode.ordinate] = [3,1,255]

        if(possible is not None):
            newNode.parentNode = possible
        
        return newNode

    def backTrackFunction(self, startingNode):

        fromNode = startingNode
        toNode = fromNode.parentNode
        cv.circle(img,(fromNode.ordinate,fromNode.abcissa), 5, (255,0,255), -1)

        while toNode is not None:
            num = 10
            check1 = True
            check2 = True

            for j in range (2):
                for i in range(num):
                    toNode = toNode.parentNode
                    if(toNode==None or toNode.parentNode == None):
                        toNode = fromNode.parentNode
                        while toNode.parentNode is not None:
                            cv.line(img,(fromNode.ordinate, fromNode.abcissa), (toNode.ordinate, toNode.abcissa), (255,255,255), 1)
                            fromNode = toNode
                            toNode = toNode.parentNode
                        break
                if(toNode.parentNode == None):
                    break
          
                oneThird_x = int((fromNode.abcissa*2+toNode.abcissa)/3)
                oneThird_y = int((fromNode.ordinate*2 + toNode.ordinate)/3)
                twoThird_x = int((fromNode.abcissa+toNode.abcissa*2)/3)
                twoThird_y = int((fromNode.ordinate + toNode.ordinate*2)/3)

                check1 = self.checkPoint(oneThird_x, oneThird_y)
                check2 = self.checkPoint(twoThird_x, twoThird_y)

                if(check1 == True and check2 == True):
                    cv.line(img,(fromNode.ordinate, fromNode.abcissa), (toNode.ordinate, toNode.abcissa), (255,255,255), 1)
                    fromNode = toNode
                    break
                elif (num>8):
                    num = 5
                    toNode = fromNode.parentNode
                
            if(check1 == True and check2 == True):
                cv.line(img,(fromNode.ordinate, fromNode.abcissa), (fromNode.parentNode.ordinate, fromNode.parentNode.abcissa), (255,255,255), 1)
                fromNode = fromNode.parentNode

            toNode = toNode.parentNode
        
#this function is the starting function and will be used to initiate the trees   
def startFunction():
    letsGoFromStart = mainCode(nodesFromStart)
    letsGoFromEnd = mainCode(nodesFromEnd) 
    # letsGoFromStart.start(50,50,0) 
    # letsGoFromEnd.start(550,550,0)
    letsGoFromStart.start(50,170,0)
    letsGoFromEnd.start(185,630,0)
    count = 0

    random_x = 0
    random_y = 0
    while True:
        print(count)
        count +=1

        if(count > PRECISION_VALUE):
            break
        
        if count%2 == 0:
            while True:
                randomCheck = True
                random_x = rand.randint(1, IMG_WIDTH -SAFE_DISTANCE_FROM_OBSTACLE - 1)
                random_y= rand.randint(1,IMG_HEIGHT -SAFE_DISTANCE_FROM_OBSTACLE - 1)
                for aNode in letsGoFromStart.nodes: 
                    if(random_y == aNode.ordinate and random_x == aNode.abcissa):
                        randomCheck = False
                        break
                if(randomCheck == True):
                    break
        
            returnValue = letsGoFromStart.findNearestNode(random_x, random_y)
            
            if(returnValue[0] is not True):
                count -= 1
                continue

            letsGoFromStart.findBetterNode(returnValue[1], count)

        else: 
            while True:
                randomCheck = True
                random_x = rand.randint(1, IMG_WIDTH-SAFE_DISTANCE_FROM_OBSTACLE -1)
                random_y= rand.randint(1,IMG_HEIGHT - SAFE_DISTANCE_FROM_OBSTACLE -1)
                for aNode in letsGoFromEnd.nodes: 
                    if(random_y == aNode.ordinate and random_x == aNode.abcissa):    
                        randomCheck = False
                        break
                if(randomCheck == True):
                    break
        
            returnValue = letsGoFromEnd.findNearestNode(random_x, random_y)
            
            if(returnValue[0] is not True):
                count -= 1
                continue

            letsGoFromEnd.findBetterNode(returnValue[1], count)

    check1 = 99999
    check2 = 99999
    node1 = None
    node2 = None
    for aNode in nodesFromStart:
        if(aNode.abcissa < 150 or aNode.ordinate<150 or aNode.abcissa > 450 or aNode.ordinate > 450):
            continue
        for anotherNode in nodesFromEnd:
            if(anotherNode.abcissa < 150 or anotherNode.ordinate<150 or anotherNode.abcissa > 450 or anotherNode.ordinate > 450):
                continue
            for i in range(COST_OF_EDGE):
                for j in range(COST_OF_EDGE):
                    if (aNode.abcissa -(COST_OF_EDGE/2) + i == anotherNode.abcissa and aNode.ordinate -(COST_OF_EDGE/2) + j == anotherNode.ordinate):
                        if ((aNode.cost) + (anotherNode.cost)) < check1 + check2:
                            node1 = aNode
                            node2 = anotherNode
                            check1 = anotherNode.cost
                            check2 = aNode.cost

    print("Started BackTracking")
    letsGoFromStart.backTrackFunction(node1)
    letsGoFromEnd.backTrackFunction(node2)

print("Starting")
start_time=time.time()
startFunction()
print("time: %s seconds" % (time.time() - start_time))
print("Winner Winner Chicken Dinner")
cv.imshow("Display window", img) 
k = cv.waitKey(0) 