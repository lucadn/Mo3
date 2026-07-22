#Mo3D mobility simulation framework based on the Mo3 model as defined in:
#L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility Model for
#future generation mobile wireless networks", IEEE Access, Volume 10, April 1, 2022, pp. 34085 - 34115. DOI: 10.1109/ACCESS.2022.3161541
#and extended in:
#D. Ferretti, L. De Nardis and M.-G. Di Benedetto, "Mo3D - a Mobility Framework for Mobility Modeling in 3D Indoor Environments,"
#submitted to Software X, 2026.

"""
    Utility scripts to create the node and obstacle lists

"""

from mobility.node import Node
from utils.scenarioGeneration import Machine
from typing import List
import sys
import math
import re
import random
def fprintf(stream, format_spec, *args):
    stream.write(format_spec % args)

def instantiate_nodes(tot_number_of_nodes: int):
    node_id = 0
    node_list = []
    # Instantiate nodes that will generate the current traffic type
    for i in range(tot_number_of_nodes):
        current_node = Node(node_id)
        node_list.append(current_node)
    for node in range(len(node_list)):
        node_list[node].set_node_id(node)
    return node_list
    
def load_obstacles(OLfilename):
    #Function loading the obstacle list used by the Obstacle Avoidance module of the Mo3 mobility model, as defined in
    #L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility model for
    #future generation mobile wireless networks", submitted to IEEE Access
    print('Loading obstacle list file');
    ObsList=list();
    numObstacles=0;
    fileID=open(OLfilename)
    fLine=fileID.readline()
    while (not (not fLine)):
        if((fLine[0]=='%') or (fLine[0]=='\n')): #Skip comments and empty lines
            fLine=fileID.readline()
        else:
            if fLine[0:8]=='Obstacle':
                #regex2='[+-]?([0-9]*[.])?[0-9]+'
                regex2=r'[-+]?(?:\d*\.*\d+)';
                obsDataRE=re.findall(regex2,fLine)
                #breakpoint()
                obsData=[float(x) for x in obsDataRE]
                #breakpoint()
                if(not len(obsData)==7):
            	    print('Error in obstacle data format, skipping the line');
                else:
                    ObsList.append(obsData)
                    numObstacles=numObstacles+1
            fLine=fileID.readline()
    fileID.close()    
    print('File over,', numObstacles, 'obstacles found');
    return ObsList
    
def save_obstacles(OLfilename, ObsList):
    #Function saving the obstacle list used by the Obstacle Avoidance module of the Mo3 mobility model, as defined in
    #L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility model for
    #future generation mobile wireless networks", submitted to IEEE Access
    print('Saving obstacle list to file');
    numObstacles=len(ObsList);
    fileID=open(OLfilename,"w")
    for i in range(numObstacles):
        fprintf(fileID,'Obstacle %d',ObsList[i][0]),
        for j in range(6):
            fprintf(fileID,' %f',ObsList[i][j+1]),
        fprintf(fileID,'\n')
    fileID.close()    
    print('Done,', numObstacles, 'obstacles saved');

    
def get_obstacles_from_layout(machine_array: List[Machine]):
    nMachines=len(machine_array)
    numObstacles=nMachines;
    ObsList=list();
    for i in range(nMachines):
        mac=machine_array[i]
        machineCoords=mac.get_coordinates()
        machineSize=mac.get_machine_size()
        obsData=[1, machineCoords[0], machineCoords[1], machineCoords[2], machineSize, machineSize, machineCoords[2]*2]
        ObsList.append(obsData)
    print('Obstacles extracted from layout,', numObstacles, 'obstacles found');
    return ObsList
    
def distribute_nodes_outside_obstacles(distType, nodes: List[Node], obstacles: List, factory_length, factory_width, factory_height, min_inter_node_distance=0.0):

    nAttempts=1000
    xmin=list()
    xmax=list()
    ymin=list()
    ymax=list()
    zmin=list()
    zmax=list()
    for obstacle in range(len(obstacles)):
        obs_type = obstacles[obstacle][0]
        if obs_type == 1:
            # Parallelepiped: obstacle[4]/[5] are full sizes (dx, dy) -> halve for half-width
            x_half = obstacles[obstacle][4] / 2
            y_half = obstacles[obstacle][5] / 2
        elif obs_type == 2:
            # Elliptic cylinder: obstacle[4]/[5] are already semi-axes (a, b) -> use as-is
            x_half = obstacles[obstacle][4]
            y_half = obstacles[obstacle][5]
        else:
            sys.exit('Unknown obstacle type in distribute_nodes_outside_obstacles')
        xmin.append(obstacles[obstacle][1]-x_half);
        xmax.append(obstacles[obstacle][1]+x_half);
        ymin.append(obstacles[obstacle][2]-y_half);
        ymax.append(obstacles[obstacle][2]+y_half);

    for obstacle in range(len(obstacles)):
        zmin.append(obstacles[obstacle][3]-obstacles[obstacle][6]/2);
        zmax.append(obstacles[obstacle][3]+obstacles[obstacle][6]/2);

    placed_coords = []  # (x, y, z) of nodes already placed in this call
    for node in nodes:
        # Distribute UEs within the factory
        attempt=0
        node_inside = True
        while (node_inside is True) and (attempt<nAttempts):
            if distType == 'Uniform':
                x_coord = random.uniform(0, factory_length)
                y_coord = random.uniform(0, factory_width)
                z_coord = random.uniform(0, factory_height)
                node_inside = False
                attempt=attempt+1
                for obstacle in range(len(obstacles)):
                    if xmin[obstacle] <= x_coord <= xmax[obstacle]:
                        if ymin[obstacle] <= y_coord <= ymax[obstacle]:
                            if zmin[obstacle] <= z_coord <= zmax[obstacle]:
                                node_inside = True
                if node_inside is False and min_inter_node_distance > 0:
                    # Also reject candidate positions closer than min_inter_node_distance
                    # to any already-placed node, so the initial layout itself doesn't
                    # start two nodes inside each other's collision-avoidance safety margin.
                    for (px, py, pz) in placed_coords:
                        d = math.sqrt((x_coord-px)**2 + (y_coord-py)**2 + (z_coord-pz)**2)
                        if d < min_inter_node_distance:
                            node_inside = True
                            break

            else:
                sys.exit('The node distribution statistics is not recognized')
        if node_inside is False: #A valid position fo the node was found
            node.set_coordinates(x_coord, y_coord, z_coord)
            placed_coords.append((x_coord, y_coord, z_coord))
        else:
            sys.exit('No valid position outside obstacles and away from other nodes found for a node')

