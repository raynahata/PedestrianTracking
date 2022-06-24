from contextlib import nullcontext
import os
from pprint import PrettyPrinter
import sys
import optparse
import csv 
import random
import math
from person import Person


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

        ped1=Person()

        print(ped1.xPos,ped1.yPos)

        
        if(len(ped)!=0):
            ped1.setNewSpeed()
        
            detectedPerson=traci.multientryexit.getLastStepVehicleIDs('e3_0')
            #print(len(detectedPerson))
            #move the person once they reach the entry detector
            if(len(detectedPerson)==1):


                ped1.setNewAngle()
                new_angle=ped1.getAngle()
                ped1.setNewX(new_angle,ped1.getSpeed())
                ped1.setNewY(new_angle,ped1.getSpeed())
                new_X=ped1.getPosX()
                new_Y=ped1.getPosY()
                ped1.movePerson(new_X,new_Y,new_angle)
                
             
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

    