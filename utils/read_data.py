#Mo3D mobility simulation framework based on the Mo3 model as defined in:
#L. De Nardis and M.-G. Di Benedetto, "Mo3: a Modular Mobility Model for
#future generation mobile wireless networks", IEEE Access, Volume 10, April 1, 2022, pp. 34085 - 34115. DOI: 10.1109/ACCESS.2022.3161541
#and extended in:
#D. Ferretti, L. De Nardis and M.-G. Di Benedetto, "Mo3D - a Mobility Framework for Mobility Modeling in 3D Indoor Environments,"
#submitted to Software X, 2026.
"""
    Utility script to read parameters and data from files.
"""

import yaml


def read_params(filepath):
    """
        Read yaml file and return a Python dictionary with all parameters.
    """
    with open(filepath) as file:
        params = yaml.load(file, Loader=yaml.FullLoader)
    return params
