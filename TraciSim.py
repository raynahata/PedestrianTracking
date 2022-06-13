from contextlib import nullcontext
import os
from pprint import PrettyPrinter
import sys
import optparse
from turtle import right 
import xml.etree.ElementTree as ET
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


#taken from https://www.geeksforgeeks.org/how-to-convert-lists-to-xml-in-python/

def create_xml(list):

		# we make root element
		usrconfig = ET.Element("usrconfig")

		# create sub element
		usrconfig = ET.SubElement(usrconfig, "usrconfig")

		# insert list element into sub elements
		for user in range(len( list)):

				usr = ET.SubElement(usrconfig, "usr\n")
				usr.text = str(list[user])

		tree = ET.ElementTree(usrconfig)

		# write the tree into an XML file
		tree.write("Position.xml", encoding ='utf-8', xml_declaration = True)
   
        
def run():
    step=0 #keep track of the current step 
    
    pedPos=[]
    pedAngle=[]
    pedSpeed=[]
    pedLaneID=[]
    while traci.simulation.getMinExpectedNumber()>0: #when we have exahused all of our route files 
        traci.simulationStep() #advance the simulation one timestep 
        ped=traci.person.getIDList()
        
        if(len(ped)!=0):
            #traci.person.setLateralAlignment('p_0','left') not work 
            pos = traci.person.getPosition('p_0') #position with regards to the X,Y coordinates 
            angle=traci.person.getAngle('p_0') #gets the angle of the person 
            speed=traci.person.getSpeed('p_0')
            laneID=traci.person.getLaneID('p_0')
            #pos=traci.person.getLanePosition('p_0') #position with regards to the lane measured in meters 
            pedPos.append(pos) 
            pedAngle.append(angle)
            pedSpeed.append(speed)
            pedLaneID.append(laneID)


            with open('pedestrianInfo.csv', 'w', encoding='UTF8') as f:
                writer = csv.writer(f)
                writer.writerow(["Position","Angle","Speed","lateral"])
                for x in range (len(pedPos)):
            
                    writer.writerow([pedPos[x],pedAngle[x],pedSpeed[x],pedLaneID[x]])
        step+=1 #increment the step
        traci.person.moveToXY('p_0', pedLaneID[0], pedPos[0][0], pedPos[0][1], angle=-40, keepRoute=2, matchThreshold=100)
   
        

    traci.close()
    sys.stdout.flush()


#main loop


if __name__== "__main__":
    options= get_options()

    #check binary 
    if options.nogui:
        sumoBinary=checkBinary('sumo')
    else:
        sumoBinary=checkBinary('SUMO-GUI')

    traci.start([sumoBinary, "-c","map2.sumo.cfg","--tripinfo-output","tripinfo.xml"])
    
    run()

    