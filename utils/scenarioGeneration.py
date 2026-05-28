#Mo3D mobility simulation framework based on the Mo3 model as defined in:
#L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility Model for
#future generation mobile wireless networks", IEEE Access, Volume 10, April 1, 2022, pp. 34085 - 34115. DOI: 10.1109/ACCESS.2022.3161541
#and extended in:
#D. Ferretti, L. De Nardis and M.-G. Di Benedetto, "Mo3D - a Mobility Framework for Mobility Modeling in 3D Indoor Environments,"
#submitted to Software X, 2026.
import sys
import math
import random
from typing import List

class Machine:
    def __init__(self, machine_size: int = None, node_density: float = None):
        # Center_coordinates
        self.x = 0
        self.y = 0
        self.z = 0
        # Coordinates of the area boundary
        self.x_max = 0
        self.x_min = 0
        self.y_max = 0
        self.y_min = 0
        if (machine_size is not None and node_density is not None):
            self.machine_size = machine_size
            self.max_number_of_nodes = math.ceil(node_density * (self.machine_size ** 3))
        else:
            self.machine_size = 0  # it will be set afterwards
            self.max_number_of_nodes = 0  # it will be set afterwards

        self.number_of_nodes = 0

    def set_coordinates(self, x_input, y_input, z_input):
        self.x = x_input
        self.y = y_input
        self.z = z_input
        self.x_max = x_input + self.machine_size / 2
        self.x_min = x_input - self.machine_size / 2
        self.y_max = y_input + self.machine_size / 2
        self.y_min = y_input - self.machine_size / 2

    def get_coordinates(self):
        return self.x, self.y, self.z, self.x_max, self.x_min, self.y_max, self.y_min

    def get_machine_size(self):
        return self.machine_size

    def set_machine_size(self, machine_size: int):
        self.machine_size = machine_size

def scenarioMachineDistribution(machine_size, inter_machine_distance, factory_length,factory_width,factory_height):
    if factory_height<machine_size:
	    sys.exit('Mismatch between machine size and factory height: doublecheck and retry.')
    number_of_machines_on_x = 1 + math.floor((factory_length-machine_size)/inter_machine_distance)
    number_of_machines_on_y = 1 + math.floor((factory_width-machine_size)/inter_machine_distance)
    number_of_machines=number_of_machines_on_x*number_of_machines_on_y
    machines = [Machine() for i in range(number_of_machines)]
    machine=0
    if number_of_machines_on_x >= number_of_machines_on_y:
        for x in range(number_of_machines_on_y):
            for y in range(number_of_machines_on_x):
                machines[machine].set_coordinates(machine_size/2 + inter_machine_distance * y,
                                                            machine_size/2 + inter_machine_distance * x,
                                                            machine_size/2
                                                            )
                machines[machine].set_machine_size(machine_size)                                                            
                machines[machine].x_min = inter_machine_distance * y
                machines[machine].x_max = machine_size + inter_machine_distance * y
                machines[machine].y_min = inter_machine_distance * x
                machines[machine].y_max = machine_size + inter_machine_distance * x
                machine += 1
    else:
        for x in range(number_of_machines_on_x):
            for y in range(number_of_machines_on_y):
                machines[machine].set_coordinates(machine_size/2 + inter_machine_distance * x,
                                                          machine_size/2 + inter_machine_distance * y,
                                                          machine_size/2
                                                          )
                machines[machine].x_min = inter_machine_distance * y
                machines[machine].x_max = machine_size + inter_machine_distance * y
                machines[machine].y_min = inter_machine_distance * x
                machines[machine].y_max = machine_size + inter_machine_distance * x
                machine += 1
    return machines
