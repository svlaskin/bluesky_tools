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

# from JP1_files.envimport import (
#     get_city_center,
#     get_city_bounding_box,
#     get_overpass_city_name,
#     envimport,
#     write_to_csv,
# )

from JP1_files.ODgen import *

"""
Constants - basically the airspace design and TMA 
"""
cruisealt_out = 100/ft
vcruise = 40
cruisealt_in = 50/ft
d_faf = 200 #m, distance faf to pad
d_iaf = 1852 #m, distance iaf to faf
delta_iaf = 135-(90-np.rad2deg(np.arcsin(np.sqrt(2*(d_faf**2))/(2*d_iaf)))) # degrees, IAF angle line so that ends of cones are touching. Derived from simple trig.

def init_plugin():
    # Addtional initilisation code
    amsspawn = AmsSpawn()

    # Configuration parameters
    config = {
        "plugin_name": "AMSSPAWN",
        # This is a sim plugin
        "plugin_type": "sim",
        "reset": AmsSpawn.reset
    }

    return config


class AmsSpawn(Entity):
    def __init__(self):
        super().__init__()
        self.last_id = 0 # global index
        self.t_elapsed = 0
        self.inter_flight_margin = 30
        print("correct amsspawn")
    @stack.command
    def pantoname(self, name):
        stack.stack("zoom 7")
        # stack.stack("asas on")
        # stack.stack("plugin load traffichandler")
        # stack.stack("plugin load m22logger")
        # stack.stack("startlogs") # start logs
        # stack.stack("trails on")
        # center = get_city_center(name)
        # stack.stack(f"PAN {center[0]} {center[1]}") # need to hardcode this for offline
        return
    @stack.command
    def set_phase(self, id, phase):
        traf.phase[traf.id2idx(id)] = phase
    @stack.command
    def set_pad(self, id, pad_num):
        traf.assigned_pad[traf.id2idx(id)] = pad_num
    
    @stack.command
    def initialize_scen(self, dt, uncertainty, inter_flight_margin):
        print("dt ", dt)
        print(uncertainty, " s of uncertainty")
        # draw pads and fafs
        ipadt, ipadl, ifaf = 0, 0, 0
        self.inter_flight_margin = float(inter_flight_margin)
        # delay distribution parameters
        self.t0 = 10.0          # seconds
        self.sigma = uncertainty/10
        self.mu = np.log(100 - self.t0) - 0.5 * self.sigma**2  # mean ≈ 60 s
        bearings_mid = [0, 90, 180, 270] # middle of bearing line per pad
        for pad in takeoff_pads:
            stack.stack(f"CIRCLE T{ipadt} {pad[0]} {pad[1]} {5/nm}")
            stack.stack(f"COLOR T{ipadt} GREEN")
            ipadt+=1
        for pad in landing_pads:
            stack.stack(f"CIRCLE L{ipadl} {pad[0]} {pad[1]} {5/nm}")
            stack.stack(f"COLOR L{ipadl} RED")
            ipadl+=1
        for faf in fafs:
            # draw FAF
            stack.stack(f"CIRCLE F{ifaf} {faf[0]} {faf[1]} {2.5/nm}")
            stack.stack(f"COLOR F{ifaf} RED")
            # draw the approach lines
            line_end_left = get_point_at_distance(lat1=faf[0], lon1=faf[1], d=1.852, bearing=bearings_mid[ifaf]-delta_iaf)
            line_end_right = get_point_at_distance(lat1=faf[0], lon1=faf[1], d=1.852, bearing=bearings_mid[ifaf]+delta_iaf)
            stack.stack(f"LINE LL{ifaf} {faf[0]} {faf[1]} {line_end_left[0]} {line_end_left[1]}")
            stack.stack(f"LINE LR{ifaf} {faf[0]} {faf[1]} {line_end_right[0]} {line_end_right[1]}")
            stack.stack(f"COLOR LR{ifaf} red")
            stack.stack(f"COLOR LL{ifaf} red")

            angles = np.linspace(bearings_mid[ifaf]-delta_iaf,bearings_mid[ifaf]+delta_iaf,10)
            lat_iaf, lon_iaf = get_point_at_distance(faf[0], faf[1], 1.852, angles)

            command = f'POLYLINE FB{ifaf}'
            for i in range(0,len(lat_iaf)):
                command += ' '+str(lat_iaf[i])+' '
                command += str(lon_iaf[i])
            bs.stack.stack(command)
            stack.stack(f"COLOR FB{ifaf} red")
            # arc = 
            # draw lines
            # draw arc for the inlet
            ifaf+=1 # increment
        # stack.stack("ff")

        # load and start loggers
        stack.stack("plugin load traffichandler")
        stack.stack("plugin load m22logger")
        stack.stack("startlogs") # start logs
    
        stack.stack("pantoname Amsterdam")
    # @timed_function(dt=30)
    @stack.command
    def spawndrone(self):
        #route, spawn_loc[0], del_loc, return_loc[0], reso, ind_pad
        route, takeoff_loc, del_loc, return_loc, reso, pad_num, reso_needed, takeoff_pads, landing_pads, fafs, routelen  = quick_OD_gen()
        stack.stack(f"CRE EXP{self.last_id} M600 {takeoff_loc[0]} {takeoff_loc[1]} 0 0 0")
        stack.stack(f"set_pad {self.last_id} {pad_num}")

        faf_loc = fafs[pad_num]
        # climb out
        stack.stack(f"EXP{self.last_id} alt {cruisealt_out}")
        # once transitioned speed out
        stack.stack(f"delay 12 ,EXP{self.last_id} atalt {cruisealt_out} {f'EXP{self.last_id} spd {vcruise}'}")
        # add outbound waypoints for geofence avoid if needed
        if reso_needed:
            for wpt in reso:
                stack.stack(f"delay 5, EXP{self.last_id} atalt {cruisealt_out}, ADDWPT EXP{self.last_id} {wpt[0]} {wpt[1]} {cruisealt_out} {vcruise}")
        # add delivery destination
        stack.stack(f"EXP{self.last_id} vnav off")
        stack.stack(f"ADDWPTMODE EXP{self.last_id} FLYOVER")
        stack.stack(f"EXP{self.last_id} DEST {del_loc[0]} {del_loc[1]}")

        # compute loiter time including delay
        loiter_time =  self.t0 + np.random.lognormal(self.mu, self.sigma, size=1)
        loiter_time = np.clip(loiter_time, None, 600) # max delay

        # to do at delivery location
        do_at_delloc_1 = [
                        f"DELRTE EXP{self.last_id}",  # delete the final waypoint
                        f"EXP{self.last_id} spd 0",
                        f"set_phase EXP{self.last_id} 1", # set phase to return
                        f"ADDWPTMODE EXP{self.last_id} FLYTURN",
                        f"ADDWPTMODE EXP{self.last_id} TURNSPD 5",
                        f"EXP{self.last_id} addwpt {faf_loc[0]} {faf_loc[1]}",
                        f"EXP{self.last_id} addwpt {return_loc[0]} {return_loc[1]}",
                        # f"EXP{self.last_id} dest {return_loc[0]} {return_loc[1]}",
                        # TODO: ADD FAF

                        # f"EXP{self.last_id} atspd 0, EXP{self.last_id} alt 0", # this is the critical alt one
                        f"EXP{self.last_id} atalt 0, delay {loiter_time}, alt EXP{self.last_id} {cruisealt_in}",
                        f"EXP{self.last_id} atalt 0, delay {loiter_time}, EXP{self.last_id} atalt {cruisealt_in}, EXP{self.last_id} spd {vcruise}",
                        # f"EXP{drone} vnav off",
                        # f"EXP{drone} lnav off",
                        f"EXP{self.last_id} atdist {return_loc[0]} {return_loc[1]}, {4/nm}, spd EXP{self.last_id} 0",
                        # f"EXP{self.last_id} atdist {return_loc[0]} {return_loc[1]}, {10/nm}, EXP{self.last_id} atspd 0, alt EXP{self.last_id} 0",
                        # f"EXP{drone} atdist {spawn_loc[0]} {spawn_loc[1]}, {30/nm}, EXP{drone} vnav off",
                        # f"EXP{drone} atdist {spawn_loc[0]} {spawn_loc[1]}, {30/nm}, EXP{drone} lnav off",
                        f"EXP{self.last_id} atdist {del_loc[0]} {del_loc[1]}, {5/nm}, DELRTE EXP{self.last_id}",
                        f"EXP{self.last_id} atdist {return_loc[0]} {return_loc[1]}, {5/nm}, EXP{self.last_id} atalt 0, del EXP{self.last_id}",
                        f"EXP{self.last_id} atspd 0, EXP{self.last_id} alt 0",
                        f"EXP{self.last_id} atdist {return_loc[0]} {return_loc[1]} {60/nm}, EXP{self.last_id} atspd 5, EXP{self.last_id} alt 0",
                        f"EXP{self.last_id} atdist {return_loc[0]} {return_loc[1]} {60/nm}, EXP{self.last_id} vnav off",
                        f"EXP{self.last_id} atdist {return_loc[0]} {return_loc[1]} {20/nm}, spd EXP{self.last_id} 5",
                        f"EXP{self.last_id} atdist {return_loc[0]} {return_loc[1]} {20/nm}, EXP{self.last_id} lnav on",
                        f"EXP{self.last_id} atdist {return_loc[0]} {return_loc[1]} {20/nm}, EXP{self.last_id} vnav on",
                        f"EXP{self.last_id} atdist {return_loc[0]} {return_loc[1]} {3/nm}, spd EXP{self.last_id} 0",
                        f"EXP{self.last_id} atdist {return_loc[0]} {return_loc[1]} {5/nm}, EXP{self.last_id} atalt 5 del EXP{self.last_id}",
                        f"EXP{self.last_id} atdist {faf_loc[0]} {faf_loc[1]} {5/nm}, EXP{self.last_id} alt 30",
                        f"EXP{self.last_id} atdist {faf_loc[0]} {faf_loc[1]} {20/nm}, EXP{self.last_id} spd 16",
                        f"EXP{self.last_id} atdist {return_loc[0]} {return_loc[1]} {50/nm}, delrte EXP{self.last_id}",
                        f"EXP{self.last_id} atdist {return_loc[0]} {return_loc[1]} {2/nm}, EXP{self.last_id} atalt 0 del EXP{self.last_id}",
                        # and here it ends
                        # f"EXP{self.last_id} atdist {return_loc[0]} {return_loc[1]} {5/nm}, EXP{self.last_id} vnav off",
                        # f"EXP{self.last_id} atdist {return_loc[0]} {return_loc[1]} {5/nm}, EXP{self.last_id} lnav off",
                        # f"EXP{self.last_id} atdist {return_loc[0]} {return_loc[1]} {85/nm}, spd EXP{self.last_id} 5"
                        # f"EXP{drone} atalt {self.cruisealt_in}, spd EXP{drone} {self.vcruise}"
                    ]
        # do_at_delloc_2 = [
        #                 f"EXP{self.last_id} atdist {return_loc[0]} {return_loc[1]}, {5/nm}, EXP{self.last_id} atalt 0, del EXP{self.last_id}",
        #                 f"EXP{self.last_id} atspd 0, EXP{self.last_id} alt 0",
        #                 f"EXP{self.last_id} atdist {return_loc[0]} {return_loc[1]} {60/nm}, EXP{self.last_id} atspd 5, EXP{self.last_id} alt 0",
        #                 f"EXP{self.last_id} atdist {return_loc[0]} {return_loc[1]} {60/nm}, EXP{self.last_id} vnav off",
        #                 f"EXP{self.last_id} atdist {return_loc[0]} {return_loc[1]} {20/nm}, spd EXP{self.last_id} 5",
        #                 f"EXP{self.last_id} atdist {return_loc[0]} {return_loc[1]} {20/nm}, EXP{self.last_id} lnav on",
        #                 f"EXP{self.last_id} atdist {return_loc[0]} {return_loc[1]} {20/nm}, EXP{self.last_id} vnav on",
        #                 f"EXP{self.last_id} atdist {return_loc[0]} {return_loc[1]} {3/nm}, spd EXP{self.last_id} 0",
        #                 f"EXP{self.last_id} atdist {return_loc[0]} {return_loc[1]} {5/nm}, EXP{self.last_id} atalt 5 del EXP{self.last_id}",
        #                 f"EXP{self.last_id} atdist {faf_loc[0]} {faf_loc[1]} {5/nm}, EXP{self.last_id} alt 30",
        #                 f"EXP{self.last_id} atdist {faf_loc[0]} {faf_loc[1]} {20/nm}, EXP{self.last_id} spd 16",
        #                 f"EXP{self.last_id} atdist {return_loc[0]} {return_loc[1]} {50/nm}, delrte EXP{self.last_id}",
        #                 f"EXP{self.last_id} atdist {return_loc[0]} {return_loc[1]} {2/nm}, EXP{self.last_id} atalt 0 del EXP{self.last_id}"]
        for command_at_delloc in do_at_delloc_1:
            stack.stack(
                    f"EXP{self.last_id} atdist {del_loc[0]} {del_loc[1]}, {50/nm}, {command_at_delloc}"
                )
        # T_loiter = t0 + np.random.lognormal(mu, sigma)
        # for command_at_delloc in do_at_delloc_2:
        #     stack.stack(
        #             f"EXP{self.last_id} atdist {del_loc[0]} {del_loc[1]}, {50/nm}, delay {delay_at_parcel_center} {command_at_delloc}"
        #         )
        self.last_id+=1 # increment IDs
        return 
    # inter_flight_margin = self.inter_flight_margin 
    @timed_function(dt=5)
    def continuous_spawn(self):
        self.t_elapsed+=5
        if self.t_elapsed%self.inter_flight_margin==0:
            stack.stack("spawndrone")
        # if self.t_elapsed>=8*3600:
        #     stack.stack("hold") # stop simming
    
    # gen demand
    @timed_function(dt=1)
    def gen_demand(self):
        self.t_elapsed

    # spawn from schedule
    @timed_function(dt=1)
    def spawn_from_sched(self):
        return

    @classmethod
    def reset(self):
        self.last_id = 0
        self.t_elapsed = 0
        # datalog.reset()
        return
    