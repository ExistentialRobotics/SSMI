#!/usr/bin/env python
from __future__ import print_function, absolute_import, division

import numpy as np
import matplotlib.pyplot as plt


def get_tricky_circular_footprint():
    """
    Gives a not-quite-circular footprint for testing, the origin is not centered in the footprint
    :return array(N, 2)[float]: points defining the footprint
    """
    return np.array(
        [[0.22674791, 0],
         [0.22129365, 0.14978179],
         [0.21026903, 0.17772615],
         [0.13216534, 0.22714073],
         [0.07048001, 0.22728987],
         [-0.06165067, 0.2245],
         [-0.12432992, 0.2102427],
         [-0.23903614, 0.16096445],
         [-0.241, 0.15896477],
         [-0.241, -0.15896477],
         [-0.23903614, -0.16096445],
         [-0.12432992, -0.2102427],
         [-0.06165067, -0.2245],
         [0.07048001, -0.22728987],
         [0.13216534, -0.22714073],
         [0.21026903, -0.17772615],
         [0.22129365, -0.14978179]]
    )


def get_tricky_oval_footprint():
    """
    gives a not-quite-oval footprint for testing, the origin is not centered in the footprint
    :return array(N, 2)[float]: points defining the footprint
    """
    return np.array([
        [1348.35, 0.],
        [1338.56, 139.75],
        [1306.71, 280.12],
        [1224.36, 338.62],
        [1093.81, 374.64],
        [-214.37, 374.64],
        [-313.62, 308.56],
        [-366.36, 117.44],
        [-374.01, -135.75],
        [-227.96, -459.13],
        [-156.72, -458.78],
        [759.8, -442.96],
        [849.69, -426.4],
        [1171.05, -353.74],
        [1303.15, -286.54],
        [1341.34, -118.37]]
    ) / 1000.
