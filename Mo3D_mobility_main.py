#Main file for the 
#Mo3D mobility simulation framework based on the Mo3 model as defined in:
#L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility Model for
#future generation mobile wireless networks", IEEE Access, Volume 10, April 1, 2022, pp. 34085 - 34115. DOI: 10.1109/ACCESS.2022.3161541
#and extended in:
#D. Ferretti, L. De Nardis and M.-G. Di Benedetto, "Mo3D - a Mobility Framework for Mobility Modeling in 3D Indoor Environments,"
#submitted to Software X, 2026.
from Mo3D.utils.read_data import read_params
from Mo3D.utils.plot_data import plot_3d_layout
from Mo3D.utils.scenarioGeneration import scenarioMachineDistribution, Machine
from Mo3D.mobility.mobility import Mobility
from Mo3D.mobility.mobilityUtils import instantiate_nodes, load_obstacles, distribute_nodes_outside_obstacles, save_obstacles, get_obstacles_from_layout



import os
import sys
import math
import numpy as np
import pandas as pd
from scipy import constants
import copy as cp
import random
import plotly.express as px

sys.path.append(os.path.dirname(os.getcwd()))
sys.path.append(os.path.dirname(os.path.dirname(os.getcwd())))

# Constants
c = constants.speed_of_light

# Pick-up inputs
inputs = read_params('Mo3D_params.yaml')
nodeDistribution = inputs.get('node').get('node_spatial_distribution')
inputNNodes = inputs.get('node').get('number_of_nodes')
mobilityUpdatePeriod= inputs.get('mobility').get('mobility_update_period')

#nodeList = list()

print("Simulation for", inputNNodes, "Nodes")

# Create list of nodes #
nodeList = instantiate_nodes(inputNNodes)

# Define scenario    
areaLength = inputs.get('scenario').get('length')
areaWidth = inputs.get('scenario').get('width')
areaHeight = inputs.get('scenario').get('height')
scenarioGeneration=inputs.get('scenario').get('scenarioGeneration')
machine_size=inputs.get('scenario').get('machine_size')
inter_machine_distance=inputs.get('scenario').get('inter_machine_distance')
OLfilename = inputs.get('scenario').get('OLfilename')
print("Area dimensions:",areaLength,"(L),", areaWidth,"(W),",areaHeight,"(H)")

if scenarioGeneration:
    #Generate a scenario and extract obstacles from it 
    machineArray=scenarioMachineDistribution(machine_size, inter_machine_distance, areaLength, areaWidth, areaHeight)
    obsList=get_obstacles_from_layout(machineArray);
else:
    #Load obstacles from file 
    obsList=load_obstacles(OLfilename);
# Place nodes outside obstacles.
distribute_nodes_outside_obstacles(nodeDistribution,nodeList, obsList, areaLength, areaWidth, areaHeight)

#Start mobility engine
m = Mobility(inputs, inputNNodes, areaLength, areaWidth, areaHeight, nodeList=nodeList, obstacleList=obsList)
m.generate_mobility_vectors()

# ***********************************************************************
# SIMULATION STARTS
# ***********************************************************************

print('******************************** Simulation starts ********************************')
nMaxUpdates=100000
nUpdates=1
updatePercentagePeriod=10;
for i in range(inputNNodes):
    coords=nodeList[i].get_coordinates()
    print("Coordinates start:", coords[0], coords[1], coords[2])

for t in range(0,nMaxUpdates-1):
    if abs(math.remainder(t,nMaxUpdates/updatePercentagePeriod))<1e-10:
        print("Update", t, "out of", nMaxUpdates, '- t =',t*mobilityUpdatePeriod)
    if m.endSim==0:
        m.update_mobility_vectors(t*mobilityUpdatePeriod, nodeList=nodeList)
        nUpdates=nUpdates+1  
    else:
        print("Mobility simulation over")
        break
  

for i in range(inputNNodes):
    coords=nodeList[i].get_coordinates()
    print("Coordinates end:", coords[0], coords[1], coords[2])
        #breakpoint()
if m.pathSave==True:
    for i in range(inputNNodes):
        if i==0:
        	nodeIDs=np.zeros(nUpdates)
        else:
        	nodei=i*np.ones(nUpdates)
        	nodeIDs=np.concatenate((nodeIDs,nodei),axis=0)
    #breakpoint()
    #Xrange=[0, areaLength]
    #Yrange=[0, areaWidth]
    #Zrange=[0, areaHeight]
    #df=pd.DataFrame(dict(X=m.xPath.reshape(nUpdates*inputNNodes, order='F'),Y=m.yPath.reshape(nUpdates*inputNNodes, order='F'),Z=m.zPath.reshape(nUpdates*inputNNodes, order='F'),nodes=nodeIDs))
    #fig = px.line_3d(df,x='X', y='Y', z='Z',color='nodes',range_x=Xrange,range_y=Yrange,range_z=Zrange)
    #fig = px.line_3d(x=m.xPath[:,0].reshape(nUpdates), y=m.yPath[:,0].reshape(nUpdates), z=m.zPath[:,0].reshape(nUpdates))
    #fig.show()
    plot_3d_layout(m)
print('******************************** Simulation ends ********************************')
