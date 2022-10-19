from contextlib import nullcontext
import os
from pprint import PrettyPrinter
from re import I
from ssl import PEM_HEADER
import sys
import optparse
import csv 
import random
import math
from tracemalloc import start
from scipy.spatial import distance
from person import Person
import numpy as np
from sumolib import checkBinary 
import traci 
 
 
#importing some python modules from the SUMO HOME tools directory 
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
 
""" 
    adding noise to the distance readings
    if the distnace between the person and the singal is less than half of the length of the 
    intersection, then delta is random number between 0 and 0.05 and if it is more, then delta is between 0.05 and 1
"""
def addNoise(distance,interLength):
    half=interLength/2
    if(distance>half):
        #delta=random.uniform(0.0025,0.005)
        delta=random.uniform(0.05,0.1)
        #delta=random.uniform(0.1,0.2)
        newDist=distance+delta
    else:
        #delta=random.uniform(0,0.0025)
        delta=random.uniform(0.00,0.05)
        #delta=random.uniform(0.0,0.1)
        newDist=distance+delta
    return newDist,delta
 
def get_options():
    opt_parser=optparse.OptionParser()
    opt_parser.add_option("--nogui",action="store_true",default=False,help="run the commandline version of sumo")
    options, args = opt_parser.parse_args()
    return options
 
def run():
 
    step=0
    dist1=0
    dist2=0
    signalDists=0
    distOnePrime=0
    distTwoPrime=0
    delta_one=0
    delta_two=0
    angle1=0
    angle2=0
    x1Prime=0
    y1Prime=0
    x2Prime=0
    y2Prime=0
    averagePrimeX=0
    averagePrimeY=0
    geoX1=0
    geoY1=0
    geoX2=0
    geoY2=0
    geoAveX=0
    geoAveY=0
    posGeo=[0,0]
    median=0
    detectedStep=0
    slopeAve=0
 
    originalX=0
    originalY=0
  
 
    #timestep 
    time=[]
 
    #pedestrain ground truth parameters
    ped_x=[]
    ped_y=[]
    ped_speed=[]
    ped_lane_ID=[]
 
    #ground truth euclidian distances 
    signalDist=[] #between the two singals
    A1_to_P=[] #from anchor one 
    A2_to_P=[] #from anchor two 
 
    #distances with noise added to them
    distance_one_prime=[]
    distance_two_prime=[]
 
    #delta 
    noise_one=[]
    noise_two=[]
 
    #angle predicted using d primes 
    angle_predict_one=[] #from anchor one 
    angle_predict_two=[] #from anhor two 
 
    #pedestrian coordinate calcuated using distances with noise,anchor one 
    xPosPrime1=[]
    yPosPrime1=[]
 
    #pedestrian coordiante calculated using distnace with noise, anchor 2
    xPosPrime2=[]
    yPosPrime2=[]
 
    xPosPrimeAve=[]
    yPosPrimeAve=[]
    slope=[]
 
    
    originalX=0
    originalY=0
    #coordinates of the two anchors 
 
    device_one_coord=[2284.5986,968.5631431]
    device_two_coord=[2293.396508,967.8745266]
 
    device1Geo=traci.simulation.convertGeo(device_one_coord[0],device_one_coord[1],fromGeo=False)
    device2Geo=traci.simulation.convertGeo(device_one_coord[0],device_one_coord[1],fromGeo=False)
 
   # print(device1Geo,device2Geo)
    #print(device_one_coord,device_two_coord)
 
   
    pedposGeo=[]
    geoCoordX1=[]
    geoCoordY1=[]
 
    geoCoordX2=[]
    geoCoordY2=[]
 
    geoAveXPos=[]
    geoAveYPos=[]
 
    detected=False
    startCorrection=False
   
    slopeMedian=[]
    warnCounter=0
 
 
    while traci.simulation.getMinExpectedNumber()>0: #when we have exahused all of our route files 
        traci.simulationStep() #advance the simulation one timestep 
        current_sim_time=traci.simulation.getTime()
        ped=traci.person.getIDList()
        ped1=Person(0) #create the person to access person class
        detectedPerson=traci.multientryexit.getLastStepVehicleIDs('e3_0')
        ped1.setNewSpeed()
             
        #move the person once they reach the entry detector
        if(len(detectedPerson)==1):
            detected=True
            
            #getting the distance between the person and anchors
            dist1=ped1.getDistance(device_one_coord) 
            dist2=ped1.getDistance(device_two_coord)
            signalDists=distance.euclidean(device_one_coord,device_two_coord)
 
            #adding noise to distances 
            newDist1tuple=addNoise(dist1,signalDists) #distance from original signal 
            distOnePrime=newDist1tuple[0]
            #print(distOnePrime)
            delta_one=newDist1tuple[1]
            newDist2tuple=addNoise(dist2,signalDists) #distance from singal you are advancing to 
            distTwoPrime=newDist2tuple[0]
            delta_two=newDist2tuple[1]
            
            
            #new coordinate calculations 
            #prediction from anchor 1 to ped 
            angle1=ped1.predictAngle(signalDists,distOnePrime,distTwoPrime)
            angleRad1=math.radians(angle1)
            x1Prime=device_one_coord[0]+(distOnePrime*math.cos(angleRad1))
            y1Prime=device_one_coord[1]+(distOnePrime*math.sin(angleRad1))
            
            #prediction from anchor 2 to ped 
            angle2=ped1.predictAngle(signalDists,distTwoPrime,distOnePrime)
            angleRad2=math.radians(angle2)
            #print(math.cos(angleRad2))
            x2Prime=device_two_coord[0]-(distTwoPrime*math.cos(angleRad2))
            y2Prime=device_two_coord[1]+(distTwoPrime*math.sin(angleRad2))
            
            #average x and y pos 
            averagePrimeX=(x1Prime+x2Prime)/2
            averagePrimeY=(y1Prime+y2Prime)/2
 
            #storing the calculated location right before starting movement 
            
            if(detectedStep==0):
                originalX=averagePrimeX
                originalY=averagePrimeY
                print("oX:",originalX)
                print("OY",originalY)
            
            if(detectedStep>=1):
                slopeAve=(averagePrimeY-originalY)/(averagePrimeX-originalX)
 
            if(detectedStep>5):
                median=np.median(slope)
                print("median",median)
                slope.pop(0)
                for i in slope:
                    print("slope2",i)
                
                #if the median is greater than 0.1 off the original slope, start counting warning
                
                if (median<(slopeMedian[0]-0.1) or median>(slopeMedian[0]+0.1)):
                    warnCounter+=1
                    
                else:
                    warnCounter=0 #reset coutner 
                    startCorrection=False
                print(warnCounter)
                if(warnCounter==5):
                    startCorrection=True
                    warnCounter-=1
                    if(median<slopeMedian[0]):
                        
                        y=averagePrimeY+0.01
                        ped1.setNewYCoordinate(y)
                            
                        ped1.setNewX(ped1.getAngle(),ped1.getSpeed())
                        new_X=ped1.getPosX()
                        new_Y=ped1.getPosY()
 
                        print("turn left")
 
                    elif(median>slopeMedian[0]):
                        
                        y=averagePrimeY-0.01
                        ped1.setNewYCoordinate(y)
                        print(y)
                        ped1.setNewX(ped1.getAngle(),ped1.getSpeed())
                        print(ped1.getPosX())
                        new_X=ped1.getPosX()
                        new_Y=ped1.getPosY()
                        print("turn right")
                    print("turn wrong direction fool")
 
            #converting average to geo location 
            converted1=traci.simulation.convertGeo(x1Prime,y1Prime,fromGeo=False)
            converted2=traci.simulation.convertGeo(x2Prime,y2Prime,fromGeo=False)
            geoX1=converted1[1]
            geoY1=converted1[0]
 
            geoX2=converted2[1]
            geoY2=converted2[0]
 
            geoAveX=(geoX1+geoX2)/2
            geoAveY=(geoY1+geoY2)/2
 
            posGeo=traci.simulation.convertGeo(ped1.xPos,ped1.yPos,fromGeo=False)
            
            
            
            detectedStep+=1
            print("newx",new_X)
            print("newy",new_Y)
            ped1.movePerson(new_X,new_Y,new_angle)
            
        if(startCorrection==False):
                print("why am i here ")
                #move the person 
                ped1.setNewAngle()
                new_angle=ped1.getAngle()
                ped1.setNewX(new_angle,ped1.getSpeed())
                ped1.setNewY(new_angle,ped1.getSpeed())
                new_X=ped1.getPosX()
                new_Y=ped1.getPosY()
            
        pedposGeo.append(posGeo)
        time.append(current_sim_time)
        ped_x.append(ped1.xPos)
        ped_y.append(ped1.yPos)
        ped_lane_ID.append(ped1.laneID)
        ped_speed.append(ped1.speed)
        A1_to_P.append(dist1)
        A2_to_P.append(dist2)
        signalDist.append(signalDists)
        distance_one_prime.append(distOnePrime)
        distance_two_prime.append(distTwoPrime)
        noise_one.append(delta_one)
        noise_two.append(delta_two)
        angle_predict_one.append(angle1)
        angle_predict_two.append(angle2)
        xPosPrime1.append(x1Prime)
        yPosPrime1.append(y1Prime)
        xPosPrime2.append(x2Prime)
        yPosPrime2.append(y2Prime)
        geoCoordX1.append(geoX1)
        geoCoordY1.append(geoY1)
        geoCoordX2.append(geoX2)
        geoCoordY2.append(geoY2)
        xPosPrimeAve.append(averagePrimeX)
        yPosPrimeAve.append(averagePrimeY)
        geoAveXPos.append(geoAveX)
        geoAveYPos.append(geoAveY)
        slope.append(slopeAve)
        slopeMedian.append(median)
        if(detected==False):
            slope.pop(0)
            slopeMedian.pop(0)
 
       
        #write in the CSV 
        with open('pedestrianInfo.csv', 'w', encoding='UTF8') as f:
            writer = csv.writer(f)
            writer.writerow(["timestep","PositionX","PositionY","geo pos x","geo pos y","Speed","Lane ID","device one coord","device two coord","signalDistance","A1 to P",
            "A2 to P","Delta 1","Delta two","Distance One Prime","Distance 2 Prime","Angle Predict One","Angle Predict Two",
            "X Pos One Prime","Y Pos One Prime","geo Coord X1","geo Coord y1", "X Pos 2 Prime","Y Pos 2 Prime","geo Coord X2","geo Coord Y2","AverageX","Average Y",
            "Ave X Geo","Ave Y geo"])
        
            for x in range (step):
                writer.writerow([time[x],ped_x[x],ped_y[x],pedposGeo[x][1],pedposGeo[x][0],ped_speed[x],ped_lane_ID[x],device_one_coord,device_two_coord,signalDist[x],A1_to_P[x],
                A2_to_P[x],noise_one[x],noise_two[x],distance_one_prime[x],distance_two_prime[x], angle_predict_one[x],angle_predict_two[x],
                xPosPrime1[x],yPosPrime1[x],geoCoordX1[x],geoCoordY1[x],xPosPrime2[x],yPosPrime2[x],geoCoordX2[x],geoCoordY2[x],xPosPrimeAve[x],yPosPrimeAve[x],
                geoAveXPos[x],geoAveYPos[x]])  
        step+=1 #increment the step
    
    traci.close()
    sys.stdout.flush()
    
 
#main loop
if __name__== "__main__":
    options= get_options()
 
    #check binary 
    if options.nogui:
        sumoBinary=checkBinary('sumo-gui')
    else:
        sumoBinary=checkBinary('SUMO-GUI')
 
    traci.start([sumoBinary, "-c","map2.sumo.cfg","--tripinfo-output","tripinfo.xml"],1)
    run()