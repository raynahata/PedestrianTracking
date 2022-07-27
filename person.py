#person file
from contextlib import nullcontext
import os
from pprint import PrettyPrinter
import sys
import optparse
import random
import math
from this import s
from turtle import pos
from scipy.spatial import distance
import numpy as np

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary 
import traci 

class Person:
    def __init__(self, scenario,default_speed = 0.3, angle_bounds = [-2,-1]):
        self.ID='p_0'
        self.default_speed = default_speed
        self.angle_bounds = angle_bounds
        self.pos=traci.person.getPosition(self.ID)
        self.xPos=traci.person.getPosition(self.ID)[0]
        self.yPos=traci.person.getPosition(self.ID)[1]
        self.angle=traci.person.getAngle(self.ID)
        self.speed=traci.person.getSpeed(self.ID)
        self.laneID=traci.person.getLaneID(self.ID)
        self.scenario=scenario

        '''
        
        0= person drifts up 
        1=person drifts down
        2=person keeps driting an acute angles 
        
        
        
        '''

    def getPos(self):
        return self.pos

    
    def getSpeed(self):
        return self.speed
    
    def setNewSpeed(self):
        self.speed = self.default_speed
        traci.person.setSpeed(self.ID,self.speed)

    def getAngle(self):
        return self.angle
    
    def setNewAngle(self):
        self.angle+=random.uniform(self.angle_bounds[0],self.angle_bounds[1])
    
    def setCorrectedAngle(self,angle):
        self.angle=angle

    def getPosX(self):
        return self.xPos
    
    def setNewX(self,newAngle,newSpeed):
        
        self.xPos+=abs((math.cos(newAngle)*newSpeed))
        
        
    
    def getPosY(self):
        return self.yPos
    
    def setNewY(self,newAngle,newSpeed):
        
        if self.scenario==0:
            newRad=math.radians(newAngle)
            self.yPos+=(math.cos(newRad)*newSpeed) 
        elif self.scenario==1:
            newRad=math.radians(newAngle)
            self.yPos-=math.cos(newRad)*newSpeed 
       

            
    
    def movePerson(self,x,y,a):
          traci.person.moveToXY(self.ID,self.laneID , x, y, angle=a, keepRoute=2, matchThreshold=100)
          self.xPos=x
          self.yPos=y
          self.pos=(x,y)


    def getDistance(self,device1):
        dist=distance.euclidean(self.pos,device1)
        return dist
    
 
    def predictAngle(self,dist1,dist2,dist3):
        '''
            dist 1 is the distance between singals (c)
            dist 2 is the distance between pedestrian and the signal they are leaving from (a)
            dist 3 is the distance between pedestrian and the signal they are going towards (b)

            maybe wrong??
       
        '''
        dist1sq=dist1**2
        dist2sq=dist2**2
        dist3sq=dist3**2

        a=((dist1sq+dist2sq)-dist3sq)/(2*dist1*dist2)

        radians=math.acos(a)
        angle=math.degrees(radians)

        
        return angle

        
        