from contextlib import nullcontext
import os
from pprint import PrettyPrinter
from ssl import PEM_HEADER
import sys
import optparse
import csv 
import random
import math
from tracemalloc import start
from scipy.spatial import distance
from person import Person
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
        delta=random.uniform(0.05,0.1)
        newDist=distance+delta
    else:
        delta=random.uniform(0,0.05)
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

    #coordinates of the two anchors 
    device_one_coord=[2284.5986,968.5631431]
    device_two_coord=[2293.396508,967.8745266]



    while traci.simulation.getMinExpectedNumber()>0: #when we have exahused all of our route files 
        traci.simulationStep() #advance the simulation one timestep 
        current_sim_time=traci.simulation.getTime()
        ped=traci.person.getIDList()
        ped1=Person(0) #create the person to access person class
        detectedPerson=traci.multientryexit.getLastStepVehicleIDs('e3_0')
        ped1.setNewSpeed()
        count=0
              
        #move the person once they reach the entry detector
        if(len(detectedPerson)==1):
            
            
            

            #general parameters 
            xPos=ped1.getPosX()
            yPos=ped1.getPosY()
            speed=ped1.getSpeed()
            

            #getting the distance between the person and anchors
            dist1=ped1.getDistance(device_one_coord) 
            dist2=ped1.getDistance(device_two_coord)
            signalDists=distance.euclidean(device_one_coord,device_two_coord)

            

            #adding noise to distances 
            newDist1tuple=addNoise(dist1,signalDists) #distance from original signal 
            distOnePrime=newDist1tuple[0]
            print(distOnePrime)
            delta_one=newDist1tuple[1]
            newDist2tuple=addNoise(dist2,signalDists) #distance from singal you are advancing to 
            distTwoPrime=newDist2tuple[0]
            delta_two=newDist2tuple[1]
            
            

            #new coordinate calculations 
            #prediction from anchor 1 to ped 
            angle1=ped1.predictAngle(signalDists,distOnePrime,distTwoPrime)
            angleRad1=math.radians(90-angle1)
            x1Prime=device_one_coord[0]+(distOnePrime*math.cos(angleRad1))
            y1Prime=device_one_coord[1]+(distOnePrime*math.cos(angleRad1))
            
            #prediction from anchor 2 to ped 
            angle2=ped1.predictAngle(signalDists,distTwoPrime,distOnePrime)
            angleRad2=math.radians(90-angle2)
            x2Prime=device_two_coord[0]+(distTwoPrime*math.cos(angleRad2))
            y2Prime=device_two_coord[1]+(distTwoPrime*math.cos(angleRad2))
            
            

            #move the person 
            ped1.setNewAngle()
            new_angle=ped1.getAngle()
            ped1.setNewX(new_angle,ped1.getSpeed())
            ped1.setNewY(new_angle,ped1.getSpeed())
            new_X=ped1.getPosX()
            new_Y=ped1.getPosY()
            ped1.movePerson(new_X,new_Y,new_angle)
       

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
            
         #write in the CSV 
        with open('pedestrianInfo.csv', 'w', encoding='UTF8') as f:
            writer = csv.writer(f)
            writer.writerow(["timestep","PositionX","PositionY","Speed","Lane ID","signalDistance","A1 to P",
            "A2 to P","Delta 1","Delta two","Distance One Prime","Distance 2 Prime","Angle Predict One","Angle Predict Two",
            "X Pos One Prime","Y Pos One Prime", "X Pos 2 Prime","Y Pos 2 Prime"])
        
            for x in range (step):
                writer.writerow([time[x],ped_x[x],ped_y[x],ped_speed[x],ped_lane_ID[x],signalDist[x],A1_to_P[x],A2_to_P[x],
                    noise_one[x],noise_two[x],distance_one_prime[x],distance_one_prime[x], angle_predict_one[x],angle_predict_two[x],
                    xPosPrime1[x],yPosPrime1[x],xPosPrime2[x],yPosPrime2[x]])  
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



            

