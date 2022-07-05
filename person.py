#person file
from contextlib import nullcontext
import os
from pprint import PrettyPrinter
import sys
import optparse
import random
import math
from turtle import pos
from scipy.spatial import distance

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary 
import traci 

class Person:
    def __init__(self, default_speed = 0.3, angle_bounds = [-1,0]):
        self.ID='p_0'
        self.default_speed = default_speed
        self.angle_bounds = angle_bounds
        self.pos=traci.person.getPosition(self.ID)
        self.xPos=traci.person.getPosition(self.ID)[0]
        self.yPos=traci.person.getPosition(self.ID)[1]
        self.angle=traci.person.getAngle(self.ID)
        self.speed=traci.person.getSpeed(self.ID)
        self.laneID=traci.person.getLaneID(self.ID)
        self.scenario=0
        '''
        0= person walking straight no off movement 
        1= person drifts up 
        2=person drifts down
        
        
        '''

    def getPos(self):
        return self.pos

    
    def getSpeed(self):
        return self.speed
    
    def setNewSpeed(self):
        #self.speed=random.uniform(0,1)
        self.speed = self.default_speed
        traci.person.setSpeed(self.ID,self.speed)

    def getAngle(self):
        return self.angle
    
    def setNewAngle(self):
        self.angle+=random.uniform(self.angle_bounds[0],self.angle_bounds[1])
    
    def getPosX(self):
        return self.xPos
    
    def setNewX(self,newAngle,newSpeed):
        
        self.xPos+=abs((math.cos(newAngle)*newSpeed))
        
        
    
    def getPosY(self):
        return self.yPos
    
    def setNewY(self,newAngle,newSpeed):
        
        if self.scenario==0:
            self.yPos+=abs(math.cos(newAngle)*newSpeed)
        elif self.scenario==1:
            self.yPos-=abs(math.cos(newAngle)*newSpeed)
        
            
    
    def movePerson(self,x,y,a):
          traci.person.moveToXY(self.ID,self.laneID , x, y, angle=a, keepRoute=2, matchThreshold=100)
          self.xPos=x
          self.yPos=y
          self.pos=(x,y)


    def getDistance(self,device1):
        dist=distance.euclidean(self.pos,device1)
        return dist


