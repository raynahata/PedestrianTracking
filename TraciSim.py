from contextlib import nullcontext
import os
from pprint import PrettyPrinter
import sys
import optparse
import csv 
import random
import math


#importing some python modules from the SUMO HOME tools directory 
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary 
import traci 

def get_options():
    opt_parser=optparse.OptionParser()
    opt_parser.add_option("--nogui",action="store_true",default=False,help="run the commandline version of sumo")
    options, args = opt_parser.parse_args()
    return options
       
def run():
    step=0 #keep track of the current step 
    pedPos=[]
    pedAngle=[]
    pedSpeed=[]
    pedLaneID=[]
    posLaneList=[]

    while traci.simulation.getMinExpectedNumber()>0: #when we have exahused all of our route files 
        traci.simulationStep() #advance the simulation one timestep 
        ped=traci.person.getIDList()
        detector=traci.multientryexit.getIDList()
        

        
        if(len(ped)!=0):
            traci.person.setSpeed('p_0',0.5)
            #person specific paramters  
            pos = traci.person.getPosition('p_0') #position with regards to the X,Y coordinates 
            angle=traci.person.getAngle('p_0') #gets the angle of the person 
            speed=traci.person.getSpeed('p_0') #speed of the person 
            laneID=traci.person.getLaneID('p_0') #lane ID 
            posLane=traci.person.getLanePosition('p_0') #position with regards to the lane measured in meters 
            
            print(pos)
            pedPos.append(pos) 
            pedAngle.append(angle)
            pedSpeed.append(speed)
            pedLaneID.append(laneID)
            posLaneList.append(posLane)
            
            with open('pedestrianInfo.csv', 'w', encoding='UTF8') as f:
                writer = csv.writer(f)
                writer.writerow(["PositionX","PositionY","Angle","Speed","Current Lane"])
                for x in range (len(pedPos)):
                    writer.writerow([pedPos[x][0],pedPos[x][1],pedAngle[x],pedSpeed[x],pedLaneID[x],posLaneList[x]])

            #person control function  
            detectedPerson=traci.multientryexit.getLastStepVehicleIDs('e3_0')
            #print(len(detectedPerson))
            #move the person once they reach the entry detector
            if(len(detectedPerson)==1):
                cur_pos=traci.person.getPosition('p_0')
                #cur_speed=traci.person.getSpeed('p_0')
                cur_angle=traci.person.getAngle('p_0')
                new_speed = random.uniform(0,1.3)
                new_angle = cur_angle + random.uniform(-5,5)
                print("new angle:", new_angle)
                print("new speed", new_speed)
                
                new_posX= cur_pos[0]+abs((math.cos(new_angle)*new_speed))
                print(math.cos(new_angle)*new_speed)
                new_posY= cur_pos[1]+ abs((math.sin(new_angle)*new_speed))


                traci.person.setSpeed('p_0',new_speed)
                #traci.person.setAngle('p_0',new_angle)
                traci.person.moveToXY('p_0', ":104530304_c1_0", new_posX, new_posY, angle=new_angle, keepRoute=0, matchThreshold=100)
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

    