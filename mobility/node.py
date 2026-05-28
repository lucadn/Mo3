#Mo3D mobility simulation framework based on the Mo3 model as defined in:
#L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility Model for
#future generation mobile wireless networks", IEEE Access, Volume 10, April 1, 2022, pp. 34085 - 34115. DOI: 10.1109/ACCESS.2022.3161541
#and extended in:
#D. Ferretti, L. De Nardis and M.-G. Di Benedetto, "Mo3D - a Mobility Framework for Mobility Modeling in 3D Indoor Environments,"
#submitted to Software X, 2026.
import sys
import copy as cp
import pandas as pd
import operator
from typing import List


class Node():
    def __init__(self, node_id):
        self.num_channels = []
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        #self.params = params
        self.node_id = node_id
        #self.snr_db = 0
        #self.ul_buffer = UeBuffer()
        #self.state = starting_state
        #self.vectState = starting_state
        self.color_node = None
        self.node_yes = 0
        super(Node, self).__init__()

    def set_coordinates(self, x_input, y_input, z_input):
        self.x = x_input
        self.y = y_input
        self.z = z_input

    def get_coordinates(self):
        return self.x, self.y, self.z

    #def get_state(self):
    #    return self.state

    #def set_state(self, input_state: str):
    #    self.state = input_state

    def get_node_id(self):
        return self.node_id

    def set_node_id(self, node_id):
        self.node_id = node_id
