from bluesky.core import Entity, timed_function
from bluesky.tools.aero import ft, kts, nm, fpm
from bluesky.tools.geo import *
from bluesky import stack
from bluesky import core
from bluesky import traffic
from bluesky import simulation
import time
from bluesky.tools.misc import degto180
import csv
from shapely.geometry import Point, box
from bluesky.tools import aero, areafilter, geo, datalog
from bluesky.plugins.geofence import Geofence
from shapely.geometry import Polygon, MultiPolygon, LineString, Point
from bluesky import core, stack, traf, tools, settings
import bluesky.plugins.geofence as geofence_TUD
from bluesky.tools.aero import ft, kts
from bluesky.traffic.route import Route
import random as random
import sys
from functools import partial
from shapely.ops import transform
import pyproj
import itertools
import numpy as np
import shapely
from JP1_files.ODgen import kwikqdrdist_matrix2
from JP1_files.envimport import (
    get_city_center,
    get_city_bounding_box,
    get_overpass_city_name,
    envimport,
    write_to_csv,
)

from JP1_files.ODgen import *

"""
Constants
"""
cruisealt_out = 100/ft
vcruise = 40
cruisealt_in = 50/ft
pc_lat, pc_lon = 52.32555214, 4.918080441

def init_plugin():
    # Addtional initilisation code
    amsspawn = MergeAgent()

    # Configuration parameters
    config = {
        "plugin_name": "MERGEAGENT",
        # This is a sim plugin
        "plugin_type": "sim",
        "reset": MergeAgent.reset
    }

    return config


class MergeAgent(Entity):
    def __init__(self):
        super().__init__()
        self.last_id = 0 # global index
    
    @timed_function(dt=5)
    def perform_merge(self):
        # compute all distances and headings, determine which IDs are inside the IAF sink
        qdr2pc, dist2pc = kwikqdrdist_matrix(np.full(np.shape(traf.lat), pc_lat), np.full(np.shape(traf.lat), pc_lon) ,traf.lat, traf.lon)
        print("qdr", qdr2pc)
        print("dist", dist2pc)
        print("phase", traf.phase)
        print("pad", traf.assigned_pad)

        

        # for id_drone in traf.id:
            # compute the heading needed
        


        return 

    @classmethod
    def reset(cls,self):
        self.last_id = 0
        return
    