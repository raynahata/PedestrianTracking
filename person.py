#person file
from contextlib import nullcontext
import os
from pprint import PrettyPrinter
import sys
import optparse
import random
import math


if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary 
import traci 

class Person:
    def __init__(self):
        self.ID='p_0'
        self.xPos=traci.person.getPosition(self.ID)[0]
        self.yPos=traci.person.getPosition(self.ID)[1]
        self.angle=traci.person.getAngle(self.ID)
        self.speed=traci.person.getSpeed(self.ID)
        self.laneID=traci.person.getLaneID(self.ID)


    def getSpeed(self):
        return self.speed
    
    def setNewSpeed(self):
        self.speed=random.uniform(0,1.3)
        traci.person.setSpeed(self.ID,self.speed)

    def getAngle(self):
        return self.angle
    
    def setNewAngle(self):
        self.angle+=random.uniform(-5,5)
    
    def getPosX(self):
        return self.xPos
    
    def setNewX(self,newAngle,newSpeed):
        print(newAngle)
        print(newSpeed)
        self.xPos+=abs((math.cos(newAngle)*newSpeed))
       
    
    def getPosY(self):
        return self.yPos
    
    def setNewY(self,newAngle,newSpeed):
        self.yPos+=abs((math.cos(newAngle)*newSpeed))
    
    def movePerson(self,x,y,a):
          traci.person.moveToXY(self.ID,self.laneID , x, y, angle=a, keepRoute=0, matchThreshold=100)


        

        

