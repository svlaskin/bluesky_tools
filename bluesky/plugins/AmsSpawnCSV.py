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
import pandas as pd
from bluesky import sim

from JP1_files.ODgen import *

cruisealt_out = 100/ft
vcruise = 40
cruisealt_in = 50/ft
d_faf = 200
d_iaf = 1852
delta_iaf = 135-(90-np.rad2deg(np.arcsin(np.sqrt(2*(d_faf**2))/(2*d_iaf))))

def init_plugin():
    amsspawncsv = AmsSpawnCSV()
    config = {
        "plugin_name": "AMSSPAWNCSV",
        "plugin_type": "sim",
        "reset": AmsSpawnCSV.reset
    }
    return config

class AmsSpawnCSV(Entity):
    def __init__(self):
        super().__init__()
        self.last_id = 0
        self.t_elapsed = 0
        self.inter_flight_margin = 30
        self.schedule = []
        print("correct amsspawncsv")

    @stack.command
    def pantoname(self, name):
        stack.stack("zoom 7")
        return

    @stack.command
    def set_phase(self, id, phase):
        traf.phase[traf.id2idx(id)] = phase

    @stack.command
    def set_pad(self, id, pad_num):
        traf.assigned_pad[traf.id2idx(id)] = pad_num

    @stack.command
    def initialize_scen(self, dt, uncertainty, inter_flight_margin, scenario_id=0):
        """
        Initialize scenario with CSV schedule
        scenario_id is used to build the file path
        """
        print("dt ", dt)
        print(float(uncertainty)/10, " is the sigma value")
        print("initialization went fine")
        self.inter_flight_margin = float(inter_flight_margin)
        self.t0 = 10.0
        self.sigma = float(uncertainty)/10
        self.mu = np.log(20 - self.t0) - 0.5 * self.sigma**2

        ipadt, ipadl, ifaf = 0, 0, 0
        bearings_mid = [0, 90, 180, 270]

        for pad in takeoff_pads:
            stack.stack(f"CIRCLE T{ipadt} {pad[0]} {pad[1]} {5/nm}")
            stack.stack(f"COLOR T{ipadt} GREEN")
            ipadt += 1
        for pad in landing_pads:
            stack.stack(f"CIRCLE L{ipadl} {pad[0]} {pad[1]} {5/nm}")
            stack.stack(f"COLOR L{ipadl} RED")
            ipadl += 1
        for faf in fafs:
            stack.stack(f"CIRCLE F{ifaf} {faf[0]} {faf[1]} {2.5/nm}")
            stack.stack(f"COLOR F{ifaf} RED")
            line_end_left = get_point_at_distance(lat1=faf[0], lon1=faf[1], d=1.852, bearing=bearings_mid[ifaf]-delta_iaf)
            line_end_right = get_point_at_distance(lat1=faf[0], lon1=faf[1], d=1.852, bearing=bearings_mid[ifaf]+delta_iaf)
            stack.stack(f"LINE LL{ifaf} {faf[0]} {faf[1]} {line_end_left[0]} {line_end_left[1]}")
            stack.stack(f"LINE LR{ifaf} {faf[0]} {faf[1]} {line_end_right[0]} {line_end_right[1]}")
            stack.stack(f"COLOR LR{ifaf} red")
            stack.stack(f"COLOR LL{ifaf} red")
            angles = np.linspace(bearings_mid[ifaf]-delta_iaf, bearings_mid[ifaf]+delta_iaf, 10)
            lat_iaf, lon_iaf = get_point_at_distance(faf[0], faf[1], 1.852, angles)
            command = f'POLYLINE FB{ifaf}'
            for i in range(len(lat_iaf)):
                command += ' '+str(lat_iaf[i])+' '+str(lon_iaf[i])
            stack.stack(command)
            stack.stack(f"COLOR FB{ifaf} red")
            ifaf += 1

        stack.stack("plugin load traffichandler")
        stack.stack("plugin load m22logger")
        stack.stack("startlogs")
        stack.stack("pantoname Amsterdam")

        csv_file = f"/Users/sasha/Documents/Code/clean_bluesky/JP1_files/scens_j1/scheduled_flights_scenario_{scenario_id}.csv"
        self.load_schedule(csv_file)

    @stack.command
    def load_schedule(self, csv_file):
        self.t_elapsed = 0
        df = pd.read_csv(csv_file)
        self.schedule = []
        for _, row in df.iterrows():
            # Parse route string without commas
            route_str = row["route"].replace('[','').replace(']','').strip()
            coords = []
            for line in route_str.split('\n'):
                if line.strip():
                    parts = line.strip().split()
                    coords.append([float(parts[0]), float(parts[1])])
            route = np.array(coords)

            self.schedule.append({
                "pad": int(row["pad"]),
                "flight_index": int(row["flight_index"]),
                "route": route,
                "flight_time": float(row["flight_time"]),
                "takeoff_time": float(row["takeoff_time"]),
                "scenario_id": int(row["scenario_id"]),
                "spawned": False
            })
        self.schedule.sort(key=lambda x: x["takeoff_time"])
        print(f"Loaded {len(self.schedule)} flights from CSV.")
        # print("schedule is")
        # print(self.schedule)
        print(f"zero time time elapsed is.. {self.t_elapsed}")

    @stack.command
    def spawndrone_from_schedule(self):
        for flight in self.schedule:
            # if not flight["spawned"] and flight["takeoff_time"] <= self.t_elapsed:
            if not flight["spawned"] and flight["takeoff_time"] <= self.t_elapsed:
                route = flight["route"]
                takeoff_loc = route[0]
                del_loc = route[1] if len(route) > 1 else route[0]
                return_loc = route[-1]
                pad_num = flight["pad"]
                faf_loc = fafs[pad_num] if pad_num < len(fafs) else route[-1]

                stack.stack(f"CRE EXP{self.last_id} M600 {takeoff_loc[0]} {takeoff_loc[1]} 0 0 0")
                stack.stack(f"set_pad {self.last_id} {pad_num}")
                stack.stack(f"EXP{self.last_id} alt {cruisealt_out}")
                stack.stack(f"delay 12 ,EXP{self.last_id} atalt {cruisealt_out} {f'EXP{self.last_id} spd {vcruise}'}")

                for wpt in route[1:-1]:
                    stack.stack(f"delay 5, EXP{self.last_id} atalt {cruisealt_out}, ADDWPT EXP{self.last_id} {wpt[0]} {wpt[1]} {cruisealt_out} {vcruise}")

                loiter_time = 10 + np.random.lognormal(mean=self.mu, sigma=self.sigma, size=1)
                loiter_time = np.clip(loiter_time, None, 600)
                if self.sigma == 0:
                    loiter_time = 10.
                loiter_time = float(loiter_time)
                # print(f"LOITER TIME WAS {loiter_time}")

                # do_at_delloc_1 = [
                #     f"DELRTE EXP{self.last_id}",
                #     f"EXP{self.last_id} spd 0",
                #     f"set_phase EXP{self.last_id} 1",
                #     f"ADDWPTMODE EXP{self.last_id} FLYTURN",
                #     f"ADDWPTMODE EXP{self.last_id} TURNSPD 5",
                #     f"EXP{self.last_id} addwpt {faf_loc[0]} {faf_loc[1]}",
                #     f"EXP{self.last_id} addwpt {return_loc[0]} {return_loc[1]}",
                #     f"EXP{self.last_id} atalt 0, delay {loiter_time}, EXP{self.last_id} vnav on",
                #     f"delay {loiter_time+10} EXP{self.last_id} spd {vcruise}",
                #     f"delay {loiter_time+10} EXP{self.last_id} alt {cruisealt_in}",    
                # ]
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
                for command_at_delloc in do_at_delloc_1:
                    stack.stack(f"EXP{self.last_id} atdist {del_loc[0]} {del_loc[1]}, {50/nm}, {command_at_delloc}")

                flight["spawned"] = True
                self.last_id += 1

    @timed_function(dt=1)
    def continuous_spawn(self):
        self.t_elapsed += 1
        self.spawndrone_from_schedule()

    @classmethod
    def reset(cls):
        cls.last_id = 0
        cls.t_elapsed = 0
        cls.schedule = []
        return
