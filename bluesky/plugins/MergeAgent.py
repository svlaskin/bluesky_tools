from bluesky.core import Entity, timed_function
from bluesky.tools.aero import ft, kts, nm, fpm
from bluesky.tools.geo import *
from bluesky import stack
from bluesky import core
from bluesky import traffic
from bluesky import traf
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

from stable_baselines3 import SAC
from gymnasium import spaces

from JP1_files.ODgen import kwikqdrdist_matrix2
from JP1_files.envimport import (
    get_city_center,
    get_city_bounding_box,
    get_overpass_city_name,
    envimport,
    write_to_csv,
)

from JP1_files.ODgen import *

import code
testing = False
"""
Notes
- need to check how the loop goes. right now per pad seems sensible, but costs computational time.
- The general flow is: go per pad, comopute general observation WRT FAF, loop per ac and append to the needed arrays
- This is a pain in the butt
"""


"""
Constants
"""
cruisealt_out = 100/ft
vcruise = 40
cruisealt_in = 50/ft
pc_lat, pc_lon = 52.32555214, 4.918080441
NM2KM = 1.852
MpS2Kt = 1.94384
NUM_AC = 5
NUM_AC_STATE = 5
N_NEAREST = 2
D_HEADING = 45 # 45 degrees each way from desired track
D_SPEED = 5 # m/s^2, max accelleration 

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
        # route, takeoff_loc, del_loc, return_loc, reso, pad_num, reso_needed, takeoff_pads, landing_pads, fafs  = quick_OD_gen()

    def bound_angle_positive_negative_180(self, angles_deg: np.ndarray) -> np.ndarray:
        # Convert input to a numpy array if it isn't already
        angles_deg = np.asarray(angles_deg)
        
        # Apply the mapping using numpy's vectorized operations
        mapped_angles = np.where(angles_deg > 180, -(360 - angles_deg), angles_deg)
        mapped_angles = np.where(mapped_angles < -180, 360 + mapped_angles, mapped_angles)
    
        return mapped_angles
    
    """
    Merging Agent function, adjusted to the North-oriented 4-pad layout.
    First, a mask array is computed, determining whether drones are close enough to the parcel center for the merge agent to start working.
    Subsequently, all of the pad indices are looped through. For each of these, we make a mask of indices of drones bound for said pad that are also close enough.
    If the number of drones is insufficient, we pad the vector.
    The observation vector is initialised, and a loop (per drone) adds the needed observations to the empty arrays, which are then complied into a dictionary.
    After this, the observation is used by the model to generate actions, which are applied in a loop over the drones via stack command.

    """
    @timed_function(dt=5)
    def perform_merge(self):
        algorithm = SAC
        self.model = algorithm.load("//Users/sasha/Documents/Code/clean_bluesky/JP1_files/models/model_mp_SAC_5drones_j1_4.zip")
        # compute all distances and headings, for every vehicle
        qdr2pc, dist2pc = kwikqdrdist_matrix(np.full(np.shape(traf.lat), pc_lat), np.full(np.shape(traf.lat), pc_lon) ,traf.lat, traf.lon)

        if testing:
            print("qdr", qdr2pc)
            print("dist", dist2pc)
            print("phase", traf.phase)
            print("pad", traf.assigned_pad)

        if np.shape(dist2pc)[0]>0:
            # mask_landing = (np.array(traf.phase)==1) & (np.array(dist2pc)<0.99)
            mask_landing = (np.array(traf.phase)==1) & (np.array(dist2pc)<=1) # 1 nautical mile, so 1852m
            mask_landing = np.array(mask_landing).astype(int)
            ids = np.array(traf.id2idx(traf.id))
            # print(mask_landing)
            # print(np.array(traf.id)[np.array(mask_landing)==1])
            # act per pad
            padinds = [0,1,2,3]
            # padinds = [0]
            for pad_index in padinds:
                # select only indices that are in given pad
                mask_needed = ((mask_landing) & (traf.assigned_pad==pad_index)).astype(int)
                # print(f"at pad {pad_ind} needed IDs are {mask_needed}")
                pass
                if np.sum(mask_needed)>=2:
                    # see by how much fixed vector needs to be extended
                    n_extend = NUM_AC - np.sum(mask_needed)

                    # compute distances to the relevant faf
                    faf_lat, faf_lon = fafs[pad_index][0], fafs[pad_index][1]
                    qdr2pc, dist2pc = kwikqdrdist_matrix(np.full(np.shape(traf.lat), faf_lat), np.full(np.shape(traf.lat), faf_lon) ,traf.lat, traf.lon)
                    # sort indices by distance to FAF
                    # ac_idx_by_dist = ids[np.argsort(dist2pc)]
                    # print(mask_needed, "msk")
                    needed_ids = ids[mask_needed==1]
                    needed_acids = np.array(traf.id)[np.array(mask_needed)==1]
                    needed_dists = dist2pc[mask_needed==1]
                    
                    if np.sum(needed_ids)>=2:
                        ac_idx_by_dist = needed_acids[np.argsort(needed_dists)]
                        if np.shape(ac_idx_by_dist)[0]>5:
                            ac_idx_by_dist = ac_idx_by_dist[::5] # truncate at 5
                            needed_ids = needed_ids[::5]
                            needed_dists = needed_dists[::5]
                            needed_acids = needed_acids[::5]

                        # start empty dictionary - note that for current trained model, NUM_AC and NUM_AC_STATE are both 10 
                        # init empty arrays, per pad agent
                        self.cos_drift = np.array([])
                        self.sin_drift = np.array([])
                        self.airspeed = np.array([])
                        self.x_r = np.array([])
                        self.y_r = np.array([])
                        self.vx_r = np.array([])
                        self.vy_r = np.array([])
                        self.cos_track = np.array([])
                        self.sin_track = np.array([])
                        self.distances = np.array([])
                        self.intruder_dx = np.array([])
                        self.intruder_dy = np.array([])
                        self.intruder_distance = np.array([])
                        self.intruder_bearing = np.array([])
                        self.intruder_vx = np.array([])
                        self.intruder_vy = np.array([])
                        self.wpt_reach = np.array([])
                        self.waypoint_dist = np.array([])
                        
                        globind = 0
                        for i in range(NUM_AC):
                            # print(f"there are {np.shape(ac_idx_by_dist)} ac")
                            # print(f"of which {np.shape(needed_acids)} are selected")
                            # print(f"i is {i}")
                            # Check if augment needed (so n_in TMA is less than the input size)
                            aug_need = False
                            # if i>int(np.shape(needed_acids)[0])-1:
                            if i>int(np.shape(ac_idx_by_dist)[0])-1:
                                aug_need = True 

                            ac_idx = traf.id2idx(ac_idx_by_dist[i]) if not aug_need else 0 # sorted by distance to the FAF
                            hdg = bs.traf.hdg[ac_idx]
                            # Get agent aircraft airspeed, m/s
                            self.airspeed = np.append(self.airspeed, bs.traf.tas[ac_idx]) if not aug_need else  np.append(self.airspeed, 0)
                            
                            # AC relative position to FAF, m
                            if aug_need:
                                brg, dist = 0, 999
                            else:
                                brg, dist = kwikqdrdist_matrix(faf_lat, faf_lon, bs.traf.lat[ac_idx], bs.traf.lon[ac_idx])
                            self.waypoint_dist = np.append(self.waypoint_dist, dist)
                            # print("brgdist:", brg, dist)
                            self.x_r = np.append(self.x_r, (dist * NM2KM * 1000) * np.cos(np.deg2rad(brg)))
                            self.y_r = np.append(self.y_r, (dist * NM2KM * 1000) * np.sin(np.deg2rad(brg)))
                            
                            # AC relative velocity to faf (which is just velocity as it is not moving), m/s
                            vx = np.cos(np.deg2rad(hdg)) * bs.traf.tas[ac_idx] if not aug_need else 0
                            vy = np.sin(np.deg2rad(hdg)) * bs.traf.tas[ac_idx] if not aug_need else 0 
                            self.vx_r = np.append(self.vx_r, vx)
                            self.vy_r = np.append(self.vy_r, vy) 

                            # AC relative track, rad
                            track = np.arctan2(vy, vx)
                            self.cos_track = np.append(self.cos_track, np.cos(track)) if not aug_need else np.append(self.cos_track, 0)
                            self.sin_track = np.append(self.sin_track, np.sin(track)) if not aug_need else np.append(self.sin_track, 0)
                            self.distances = np.append(self.distances, dist2pc[ac_idx]) if not aug_need else np.append(self.distances, 0)
                            wpt_reached = False
                            if dist2pc[ac_idx]<0.2:
                                wpt_reached = True 
                            self.wpt_reach = np.append(self.wpt_reach, wpt_reached)
                            # now redo all for relative to other AC
                            bs.traf.lat
                            if not aug_need:
                                brg_int, dist_int = kwikqdrdist_matrix(np.array(bs.traf.lat[needed_ids]), np.array(bs.traf.lon[needed_ids]), np.array(bs.traf.lat[ac_idx]),np.array(bs.traf.lon[ac_idx]))
                            else:
                                brg_int, dist_int = 0, 999
                            # print(f"the int bearing and int dist are {brg_int}, {dist_int} for indices {needed_ids}")
                            closest_int_indices = ac_idx_by_dist[np.argsort(dist_int)] if not aug_need else [0,0,0] # closest intruders 
                            if np.shape(closest_int_indices)[0]<2:
                                closest_int_indices = [0,0,0]
                            closest_int_indices_n = closest_int_indices[1:N_NEAREST+1] if not aug_need else [0,0]
                            # print("closest", closest_int_indices_n)
                            # self.intruder_distance = np.append(self.intruder_distance,dist_int) if not aug_need else np.append(self.intruder_distance,999)
                            # self.intruder_bearing = np.append(self.intruder_bearing,brg_int) if not aug_need else np.append(self.intruder_bearing,0)
                            intindex = 0
                            # TODO: this is kinda borked, need to fix it up 
                            for intindex in range(2): # iterator for the 2 needed intruders
                                if intindex>np.shape(closest_int_indices_n)[0]-1:
                                    aug_need = True
                                # print(f"globind is {globind}")
                                globind+=1
                                # print(f"intint {intindex} at idx {int_index}")
                                # int_index = bs.traf.id2idx([int_index]) if not aug_need else 0
                                int_index = bs.traf.id2idx(closest_int_indices_n[intindex]) if not aug_need else 0
                                intruder_hdg = bs.traf.hdg[int_index] if not aug_need else 0
                                # intruder_airspeed = bs.traf.tas[int_index]
                                self.intruder_distance = np.append(self.intruder_distance,dist_int[intindex]) if not aug_need else np.append(self.intruder_bearing,999)
                                # print(self.intruder_distance, "disttttt")
                                # print(f"problem guy is {brg_int} with shape {np.shape(brg_int)}")
                                brg_int_sel = brg_int[intindex] if isinstance(brg_int, np.ndarray) else brg_int
                                self.intruder_bearing = np.append(self.intruder_bearing,brg_int_sel) if not aug_need else np.append(self.intruder_bearing,0)
                                vxi = np.cos(np.deg2rad(intruder_hdg)) * bs.traf.tas[int_index] if not aug_need else 0
                                vyi = np.sin(np.deg2rad(intruder_hdg)) * bs.traf.tas[int_index] if not aug_need else 0
                                self.intruder_vx = np.append(self.intruder_vx, vxi-vx) 
                                self.intruder_vy = np.append(self.intruder_vy, vyi-vy)

                            # wpt_qdr, wpt_dist = brg, dist
                            
                            ac_hdg = traf.hdg[i] if not aug_need else 0
                            drift_angle = ac_hdg - brg
                            # print(ac_hdg, "ach")
                            # print(wpt_qdr, "qdr")
                            drift_wpt = self.bound_angle_positive_negative_180(np.array([drift_angle]))
                            # self.drift = drifts
                            self.cos_drift = np.append(self.cos_drift, np.cos(np.deg2rad(drift_wpt)))
                            self.sin_drift = np.append(self.sin_drift, np.sin(np.deg2rad(drift_wpt)))

                            self.observation_space = spaces.Dict(
                            {
                                "cos(drift)": spaces.Box(-1, 1, shape=(NUM_AC,), dtype=np.float64),
                                "sin(drift)": spaces.Box(-1, 1, shape=(NUM_AC,), dtype=np.float64),
                                "airspeed": spaces.Box(-np.inf, np.inf, shape=(NUM_AC_STATE,), dtype=np.float64),
                                "waypoint_dist": spaces.Box(-np.inf, np.inf, shape=(NUM_AC_STATE,), dtype=np.float64),
                                "faf_reached": spaces.Box(0, 1, shape=(NUM_AC,), dtype=np.float64),
                                # "x_r": spaces.Box(-np.inf, np.inf, shape=(NUM_AC_STATE,), dtype=np.float64),
                                # "y_r": spaces.Box(-np.inf, np.inf, shape=(NUM_AC_STATE,), dtype=np.float64),
                                # "vx_r": spaces.Box(-np.inf, np.inf, shape=(NUM_AC_STATE,), dtype=np.float64),
                                # "vy_r": spaces.Box(-np.inf, np.inf, shape=(NUM_AC_STATE,), dtype=np.float64),
                                "cos(track)": spaces.Box(-np.inf, np.inf, shape=(NUM_AC_STATE,), dtype=np.float64),
                                "sin(track)": spaces.Box(-np.inf, np.inf, shape=(NUM_AC_STATE,), dtype=np.float64),
                                "distances": spaces.Box(-np.inf, np.inf, shape=(NUM_AC,), dtype=np.float64),
                                # "intruder_dx": spaces.Box(-np.inf, np.inf, shape=(NUM_AC_STATE*2,), dtype=np.float64),
                                # "intruder_dy": spaces.Box(-np.inf, np.inf, shape=(NUM_AC_STATE*2,), dtype=np.float64),
                                "intruder_distance": spaces.Box(-np.inf, np.inf, shape=(NUM_AC_STATE*N_NEAREST,), dtype=np.float64),
                                "intruder_bearing": spaces.Box(-np.inf, np.inf, shape=(NUM_AC_STATE*N_NEAREST,), dtype=np.float64),
                                "intruder_vx": spaces.Box(-np.inf, np.inf, shape=(NUM_AC_STATE*N_NEAREST,), dtype=np.float64),
                                "intruder_vy": spaces.Box(-np.inf, np.inf, shape=(NUM_AC_STATE*N_NEAREST,), dtype=np.float64),
                            }
                            )


                            # # compile observation vector per thing
                            observation = {
                                "cos(drift)": np.array(self.cos_drift),
                                "sin(drift)": np.array(self.sin_drift),
                                "airspeed": np.array((self.airspeed)/25),
                                "waypoint_dist": np.array(self.waypoint_dist/2),
                                "faf_reached": np.array(self.wpt_reach),
                                # "x_r": np.array(self.x_r/1000),
                                # "y_r": np.array(self.y_r/1000),
                                # "vx_r": np.array(self.vx_r/15),
                                # "vy_r": np.array(self.vy_r/15),
                                "cos(track)": np.array(self.cos_track),
                                "sin(track)": np.array(self.sin_track),
                                "distances": np.array(self.distances/2),
                                # "intruder_dx": np.array(self.intruder_dx/250),
                                # "intruder_dy": np.array(self.intruder_dy/250),
                                "intruder_distance": np.array(self.intruder_distance/50),
                                "intruder_bearing": np.array(self.intruder_bearing/20),
                                "intruder_vx": np.array(self.intruder_vx/20),
                                "intruder_vy": np.array(self.intruder_vy/20)
                            }

                            if testing: 
                                code.interact(local=locals())
                            print(traf.work)
                            # print(observation)
                        # # compute action
                        action, _ = self.model.predict(observation) 
                        action_index = 0
                        act_sw_ind = 0
                        for id in needed_acids:
                                    # print(f"action id ID is {id} out of {needed_acids}")
                                    # se
                                    # print(action)
                                    # stack.stack(f"{id} spd 0")
                                if self.wpt_reach[act_sw_ind]==0:
                                    dv = action[action_index+1] * D_SPEED
                                    dh = action[action_index] * D_HEADING
                                    heading_new = self.bound_angle_positive_negative_180(bs.traf.hdg[bs.traf.id2idx(f'EXP{id}')] + dh)
                                    speed_new = (bs.traf.tas[bs.traf.id2idx(f'EXP{id}')] + dv) * MpS2Kt
                                    speed_new = speed_new if speed_new>0 else 0
                                    action_index+=2
                                    stack.stack(f"{id} lnav off")
                                    stack.stack(f"{id} vnav off")
                                    # print(f"ACTION {id}!!")
                                    bs.stack.stack(f"HDG {id} {heading_new}")
                                    bs.stack.stack(f"SPD {id} {speed_new}")
                                    act_sw_ind+=1
                                else:
                                    stack.stack(f"{id} lnav on")
                                    stack.stack(f"{id} vnav on")
                                
        return 

    @classmethod
    def reset(cls,self):
        self.last_id = 0
        return
    