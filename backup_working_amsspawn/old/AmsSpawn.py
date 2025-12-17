from bluesky.core import Entity, timed_function
from bluesky.tools.aero import ft, kts, nm, fpm
from bluesky.tools.geo import *
from bluesky import stack
from bluesky import core
from bluesky import traffic
from bluesky import simulation
import time
import datetime
import pickle
import pandas as pd
import geopandas as gpd
import osmnx as ox
import networkx as nx
from bluesky.tools.misc import degto180
import csv
import cbsodata
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
import shapely
from envimport import (
    get_city_center,
    get_city_bounding_box,
    get_overpass_city_name,
    envimport,
    write_to_csv,
)

def init_plugin():
    # Addtional initilisation code
    amsspawn = AmsSpawn()

    # Configuration parameters
    config = {
        "plugin_name": "AMSSPAWN",
        # This is a sim plugin
        "plugin_type": "sim",
    }

    return config

class AmsSpawn(Entity):
    def __init__(self):
        self.spawn_points2 = []
        self.radius = 0
        self.latc = 0
        self.lonc = 0
        self.density = 0
        # lat/lon bounds for the delivery areas
        self.uplat = 52.5228  # 52.4284
        self.downlat = 52.2685  # 52.3232
        self.uplon = 5.0795  # 4.9943
        self.downlon = 4.6984  # 4.7306
        self.reso_id = []  # already resolved ids
        self.ngeoreso = 0
        # timing parameters for flight
        self.t_acc_delay = 3. # [s], acceleration fudge factor
        self.t_climb_out = 21.5 # [s], time to climb (and descend) for outbound alt.
        self.t_climb_in = 11.5 # [s], time to climb (and descend) for inbound alt.
        self.t_loiter = 10. # [s], loiter times
        self.ll_int = 40 # min interval between two landings at one location [s]
        self.lt_int = 40 # minimum interval between a takeff and landing at one location [s]
        self.dt_turnaround = 60. # turnaround of drone on landing pad before other can land [s]
        self.dt_look_strat = 10 # lookeahead time for strategic CD [s]. Double of nominal 5 seconds for now.
        self.dt_landing_buffer = self.t_climb_out # lookeahead in case of landing [s] (corrected for dynamics inaccuracy)
        self.dt_takeoff_buffer = self.t_climb_in

        self.prob_del, self.prob_inf, self.prob_em = [
            0.85,
            0.12,
            0.03,
        ]  # base probabilities
        self.mdens = 0  # measured density for center measurement area circle
        self.lat_cc, self.lon_cc = 0, 0  # empty latlon for drones in central area
        self.active_del = []  # list of active deliveries
        self.active_infra = []  # list of active infrastructure obs. flights
        self.standardtime = time.time()

        # Sim boundaries
        self.simcenter = (52.4, 4.85)
        self.ams_center = [
            [52.3328, 52.3997],
            [4.7698, 4.9414],
        ]  # very center of the city bbox

    """
    -----------------------------------------------------------------
    Initiate spawn: specify density, mission type and city (from string or bounding box of latitudes and longitudes).
    This loads all delivery locations and geofences, as well as hospitals from OSM using overpass and loads them into the sim node in BlueSky.
    After using this, just use the gen_demand function.
    """
    @stack.command
    def initspawn(
        self,
        dens,
        p_del,
        p_infra,
        cityname,
        height_geofence=50,
        vcruise=40,  # 40 m/s ~ 140kph
        dtlook=5,
        cruisealt_out=100,
        cruisealt_in=50,
    ):
        self.density = dens
        self.prob_del = p_del
        self.prob_infra = p_infra  # []
        self.vcruise = 140  # [kmh] convereted to kts
        self.dtlook = dtlook  # [s]
        self.cruisealt_out = cruisealt_out / ft  # cruise alt outbound, [ft]
        self.cruisealt_in = cruisealt_in / ft  # cruise alt inbound layer [ft]
        self.r_pz = 50 #[m], protected zone radius
        
        # compute central del locations
        dest_arr = np.array(self.parcel_df)

        # stack.stack(
        #     "vis map tiledmap"
        # )  # use tiled map for city visualisation - to be replaced.
        # stack.stack("plugin load geofence")  # import GEOFENCE
        geofence_file = "bluesky/Sasha Experiments/AAAgeofences.csv"

        # Get bounding box of city for area creation
        bbox = get_city_bounding_box(get_overpass_city_name(cityname))
        print("Importing environment")
        self.hospital_df, self.geofence_df, self.delivery_df = envimport(
            cityname, height_geofence
        )  # use envimport
        print("Import done!")

        # set city as area
        stack.stack(f"BOX {cityname} {bbox[0]} {bbox[1]} {bbox[2]} {bbox[3]}")
        stack.stack(f"area {cityname}")
 
        # open geofence data
        for geofence in self.geofence_df:
            geofence_name = geofence[0]
            coordinates = " ".join(geofence[1:])
            stack.stack(f"geofence {geofence_name} {9000} {0} {coordinates}")
        stack.stack("drawgeofence")
        # stack.stack(f"PAN {52.3837} {4.9068}")
        # stack.stack("zoom 5")
        # stack.stack("op")
        stack.stack(f"pantoname {cityname}")

        # set the lookahead time and turn on asas
        stack.stack("asas on")
        # stack.stack("reso mvp")
        stack.stack(f"dtlook {15}")
        stack.stack(f"zoner {self.r_pz/nm}")
        stack.stack(f"zonedh {30/ft}")

        return
    
    @stack.command
    def pantoname(self, name):
        center = get_city_center(name)
        stack.stack(f"PAN {center[0]} {center[1]}")
        stack.stack("zoom 5")
        return