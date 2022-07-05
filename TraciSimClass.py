from contextlib import nullcontext
import os
from pprint import PrettyPrinter
import sys
import optparse
import csv 
import random
import math
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



def get_options():
    opt_parser=optparse.OptionParser()
    opt_parser.add_option("--nogui",action="store_true",default=False,help="run the commandline version of sumo")
    options, args = opt_parser.parse_args()
    return options
       
def run():
    step=0 #keep track of the current step
    dist1=0
    dist2=0

    #pedestrain parameter arrays for plotting 
    ped_x=[]
    ped_y=[]
    ped_angle=[]
    ped_speed=[]
    ped_laneID=[]
    distance_one=[]
    distance_two=[]
    device_one_coord=[2284.5986,968.5631431]
    device_two_coord=[2293.396508,967.8745266]


    while traci.simulation.getMinExpectedNumber()>0: #when we have exahused all of our route files 
        traci.simulationStep() #advance the simulation one timestep 
        ped=traci.person.getIDList()
        ped1=Person()
        ped1.scenario=1
        
        ped_x.append(ped1.xPos)
        ped_y.append(ped1.yPos)
        ped_angle.append(ped1.angle)
        ped_speed.append(ped1.speed)
        ped_laneID.append(ped1.laneID)
        distance_one.append(dist1)
        distance_two.append(dist2)

        

        #CSV creater 
        with open('pedestrianInfo.csv', 'w', encoding='UTF8') as f:
            writer = csv.writer(f)
            writer.writerow(["PositionX","PositionY","Angle","Speed","Current Lane","dist 1","dist 2"])
            for x in range (step):
                writer.writerow([ped_x[x],ped_y[x],ped_angle[x],ped_speed[x],ped_laneID[x],distance_one[x+1],distance_two[x+1]])
        
        if(len(ped)!=0):
            ped1.setNewSpeed()
            detectedPerson=traci.multientryexit.getLastStepVehicleIDs('e3_0')
            ped1.setNewAngle()
            new_angle=ped1.getAngle()
        
            #move the person once they reach the entry detector
            if(len(detectedPerson)==1):
                ped1.setNewX(new_angle,ped1.getSpeed())
                ped1.setNewY(new_angle,ped1.getSpeed())
                new_X=ped1.getPosX()
                new_Y=ped1.getPosY()
                ped1.movePerson(new_X,new_Y,new_angle)
                dist1=ped1.getDistance(device_one_coord)
                dist2=ped1.getDistance(device_two_coord)
                
             
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

    