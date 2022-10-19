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
import pandas as pd
from decimal import Decimal
from sumolib import checkBinary 
import traci 

 


import os, sys
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
    long=[]
    lati=[]
    file = pd.read_csv ("PedDownCorrected2.csv")
    

    coordinateX= file["AverageX_Test"].astype(float)
    

    coordinateY= file["AverageY_Test"].astype(float)

#net = sumolib.net.readNet('algorithmFixed.xml')


# network coordinates (lower left network corner is at x=0, y=0)
#x, y = net.convertLonLat2XY(lon, lat)
    for i in range(coordinateX.count()):
        #lon, lat = net.convertXY2LonLat(coordinates[i][0],coordinates[i][1])
        
        lon, lat = traci.simulation.convertGeo(Decimal(coordinateX[i]),Decimal(coordinateY[i]) )
        

        long.append(lon)
        lati.append(lat)

    with open('GeoPedDownCorrected2.csv', 'w', encoding='UTF8') as f:
        writer = csv.writer(f)
        writer.writerow(["lon","lat"])

        for x in range (coordinateX.count()):
            writer.writerow([long[x],lati[x]])  


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