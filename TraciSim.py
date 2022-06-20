from contextlib import nullcontext
import os
from pprint import PrettyPrinter
import sys
import optparse
import csv 


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
                writer.writerow(["Position","Angle","Speed","Current Lane"])
                for x in range (len(pedPos)):
                    writer.writerow([pedPos[x],pedAngle[x],pedSpeed[x],pedLaneID[x],posLaneList[x]])

            #person control function  
            detectedPerson=traci.multientryexit.getLastStepVehicleIDs('e3_0')
            #print(len(detectedPerson))
            #move the person once they reach the entry detector
            if(len(detectedPerson)==1):
                traci.person.moveToXY('p_0', "104530304_w3_0", pos[0]+1, pos[1]+1, angle=-40, keepRoute=2, matchThreshold=100)
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

    