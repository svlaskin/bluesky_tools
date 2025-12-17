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
from bluesky.tools.aero import ft, kts
import random as random
import sys
from functools import partial
from shapely.ops import transform
# import pyproj 
import itertools
import numpy as np
import shapely
import torch

# from stable_baselines3 import SAC
# from gymnasium import spaces

from bluesky_tools.bluesky.sac_cr_att.actor import MultiHeadAdditiveActorBasic
from bluesky_tools.bluesky.sac_cr_att.critic_q import MultiHeadAdditiveCriticQv3Basic
from bluesky_tools.bluesky.sac_cr_att.SAC import SAC
from bluesky_tools.bluesky.sac_cr_att.replay_buffer import ReplayBuffer
import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from JP1_files.ODgen import kwikqdrdist_matrix2
# from JP1_files.envimport import (
#     get_city_center,
#     get_city_bounding_box,
#     get_overpass_city_name,
#     envimport,
#     write_to_csv,
# )

from JP1_files.ODgen import *



import code
testing = False
"""
Notes
- need to check how the loop goes. right now per pad seems sensible, but costs computational time.
- The general flow is: go per pad, comopute general observation WRT FAF, loop per ac and append to the needed arrays
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
# NUM_AC = 5
NUM_AC_STATE = 3
# N_NEAREST = 2
D_HEADING = 22.5 # 45 degrees each way from desired track
D_SPEED = 5*1.94384449 # m/s2 to kts # m/s^2, max accelleration 
DISTANCE_MARGIN = 0.1 # km

CENTER = ([51.990426702297746, 4.376124857109851]) # TU Delft AE Faculty coordinates

def init_plugin():
    # Addtional initilisation code
    mergeagent = MergeAgentMulti()

    # Configuration parameters
    config = {
        "plugin_name": "MERGEAGENTMULTI",
        # This is a sim plugin
        "plugin_type": "sim",
        "reset": MergeAgentMulti.reset
    }

    return config


class MergeAgentMulti(Entity):
    def __init__(self):
        super().__init__()
        self.last_id = 0 # global index
        # self.inter_flight_margin = 30
        route, takeoff_loc, del_loc, return_loc, reso, pad_num, reso_needed, takeoff_pads, self.landing_pads, self.fafs, routelen = quick_OD_gen()
        # self.rwy_lat = 
        # self.rwy_lon = 
        # n_veh = len(self.agents)
        # action_dim = 2
        
        # load model for N_veh
        # buffer = ReplayBuffer(obs_dim = 7,
        # action_dim = action_dim,
        # n_agents = n_veh,
        # size = int(4e6),
        # batch_size = 1024)
        # self.actor = MultiHeadAdditiveActorBasic(q_dim=3, kv_dim=7, out_dim=action_dim, num_heads=3)
        # critic_q = MultiHeadAdditiveCriticQv3Basic(q_dim=5, kv_dim=7, num_heads=3)
        # critic_q_target = MultiHeadAdditiveCriticQv3Basic(q_dim=5, kv_dim=7, num_heads=3)
        # # weights_folder = "/Users/sasha/Documents/Code/multiagent_merge/bluesky-gym/sac_cr_att_sas_vlim_20" # for merge
        # weights_folder = "/Users/sasha/Documents/Code/multiagent_merge/bluesky-gym/sac_cr_att_sas_vlim_20"
        # # weights_folder = "/Users/sasha/Documents/Code/multiagent_merge/bluesky-gym/sac_cr_att_per_clean" # for CR
        # self.actor.load_state_dict(torch.load(f"{weights_folder}/actor.pt"))
        # critic_q.load_state_dict(torch.load(f"{weights_folder}/qf.pt"))
        # critic_q_target.load_state_dict(torch.load(f"{weights_folder}/qf_target.pt"))
        # # print("MODELS LOADED")

        action_dim = 2
                    
        # # load model for N_veh
        self.actor = MultiHeadAdditiveActorBasic(q_dim=3, kv_dim=7, out_dim=action_dim, num_heads=3)
        weights_folder = "/Users/sasha/Documents/Code/multiagent_merge/bluesky-gym/sac_cr_att_sas_vlim_20"
        self.actor.load_state_dict(torch.load(f"{weights_folder}/actor.pt"))

    def bound_angle_positive_negative_180(self, angles_deg: np.ndarray) -> np.ndarray:
        # Convert input to a numpy array if it isn't already
        angles_deg = np.asarray(angles_deg)
        
        # Apply the mapping using numpy's vectorized operations
        mapped_angles = np.where(angles_deg > 180, -(360 - angles_deg), angles_deg)
        mapped_angles = np.where(mapped_angles < -180, 360 + mapped_angles, mapped_angles)
    
        return mapped_angles

    def latlong_to_nm(self,center, point):
        center = np.array(center, dtype=float)
        point = np.array(point, dtype=float)
        x = (point[0] - center[0]) * 60
        y = (point[1] - center[1]) * 60 * np.cos(np.radians(center[0]))
        return np.array([x, y])

    

    def euclidean_distance(self, point1: np.array, point2: np.array) -> float:
        """ Calculate the euclidean distance between two points
        Parameters
        __________
        point1: np.array
            [x, y] of the first point
        point2: np.array
            [x, y] of the second point
            
        Returns
        __________
        distance: float
            euclidean distance between the two points
        """
        return np.sqrt(np.sum((point2 - point1)**2))
    
    def _get_observation_big(self):
        obs = []
        self.waypoint_dist = {a: 0 for a in self.agents}

        for agent in self.agents:
            ac_idx = bs.traf.id2idx(agent)

            x_r = np.array([])
            y_r = np.array([])
            vx_r = np.array([])
            vy_r = np.array([])
            cos_track = np.array([])
            sin_track = np.array([])
            distances = np.array([])

            ac_hdg = bs.traf.hdg[ac_idx]
            
            # Get and decompose agent aircaft drift
            if bs.traf.fafreach[ac_idx] == 0: # pre-faf check
                wpt_qdr, wpt_dist  = bs.tools.geo.kwikqdrdist(bs.traf.lat[ac_idx], bs.traf.lon[ac_idx], self.wpt_lat, self.wpt_lon)
            else: # post-faf check
                wpt_qdr, wpt_dist  = bs.tools.geo.kwikqdrdist(bs.traf.lat[ac_idx], bs.traf.lon[ac_idx], self.rwy_lat, self.rwy_lon)

            drift = ac_hdg - wpt_qdr
            drift = self.bound_angle_positive_negative_180(drift)

            cos_drift = np.cos(np.deg2rad(drift))
            sin_drift = np.sin(np.deg2rad(drift))

            self.waypoint_dist[agent] = wpt_dist

            if wpt_dist<DISTANCE_MARGIN and bs.traf.fafreach[ac_idx]==0:
                bs.traf.fafreach[ac_idx] = 1 # condition for reaching the FAF

            # Get agent aircraft airspeed, m/s
            airspeed = bs.traf.tas[ac_idx]

            vx = np.cos(np.deg2rad(ac_hdg)) * bs.traf.gs[ac_idx]
            vy = np.sin(np.deg2rad(ac_hdg)) * bs.traf.gs[ac_idx]
            
            ac_loc = self.latlong_to_nm(CENTER, np.array([bs.traf.lat[ac_idx], bs.traf.lon[ac_idx]])) * NM2KM * 1000 # Two-step conversion lat/long -> NM -> m
            dist = [self.euclidean_distance(ac_loc, self.latlong_to_nm(CENTER, np.array([bs.traf.lat[traf.id2idx(i)], bs.traf.lon[traf.id2idx(i)]])) * NM2KM * 1000) for i in self.agents]
            ac_idx_by_dist = np.argsort(dist)
            # print(f"ac_idx_by_dist, {ac_idx_by_dist} so the acids are {self.agents[ac_idx_by_dist]} ")

            for i in range(self.num_ac): # loop through all aircraft allocated to this pad
                int_idx = ac_idx_by_dist[i]
                if int_idx == ac_idx:
                    continue
                int_hdg = bs.traf.hdg[int_idx]
                
                # Intruder AC relative position, m
                int_loc = self.latlong_to_nm(CENTER, np.array([bs.traf.lat[int_idx], bs.traf.lon[int_idx]])) * NM2KM * 1000
                x_r = np.append(x_r, int_loc[0] - ac_loc[0])
                y_r = np.append(y_r, int_loc[1] - ac_loc[1])

                # Intruder AC relative velocity, m/s
                vx_int = np.cos(np.deg2rad(int_hdg)) * bs.traf.gs[int_idx]
                vy_int = np.sin(np.deg2rad(int_hdg)) * bs.traf.gs[int_idx]
                vx_r = np.append(vx_r, vx_int - vx)
                vy_r = np.append(vy_r, vy_int - vy)

                # Intruder AC relative track, rad
                track = np.arctan2(vy_int - vy, vx_int - vx)
                cos_track = np.append(cos_track, np.cos(track))
                sin_track = np.append(sin_track, np.sin(track))

                distances = np.append(distances, dist[int_idx])

            # very crude normalization for the observation vectors
            observation = {
                "cos(drift)": np.array([cos_drift]),
                "sin(drift)": np.array([sin_drift]),
                "airspeed": np.array([(airspeed)/25]),
                "x_r": np.array(x_r[:NUM_AC_STATE]/5000),
                "y_r": np.array(y_r[:NUM_AC_STATE]/5000),
                "vx_r": np.array(vx_r[:NUM_AC_STATE]/25),
                "vy_r": np.array(vy_r[:NUM_AC_STATE]/25),
                "cos(track)": np.array(cos_track[:NUM_AC_STATE]),
                "sin(track)": np.array(sin_track[:NUM_AC_STATE]),
                "distances": np.array(distances[:NUM_AC_STATE])/250000.
            }

            obs.append(np.concatenate(list(observation.values())))
        
        observations = {
            a: o
            for a, o in zip(self.agents, obs)
        }
        # print(observations)
        
        return observations
    
    def _get_observation(self):
        obs = []
        self.waypoint_dist = {a: 0 for a in self.agents}

        for agent in self.agents:
            ac_idx = bs.traf.id2idx(agent)
            ac_hdg = bs.traf.hdg[ac_idx]
            
            # Get and decompose agent aircaft drift
            if traf.fafreach[traf.id2idx(agent)] == 0: # pre-faf check
                wpt_qdr, wpt_dist  = bs.tools.geo.kwikqdrdist(bs.traf.lat[ac_idx], bs.traf.lon[ac_idx], self.wpt_lat, self.wpt_lon)
            else: # post-faf check
                wpt_qdr, wpt_dist  = bs.tools.geo.kwikqdrdist(bs.traf.lat[ac_idx], bs.traf.lon[ac_idx], self.rwy_lat, self.rwy_lon)

            drift = ac_hdg - wpt_qdr
            drift = self.bound_angle_positive_negative_180(drift)
            cos_drift = np.cos(np.deg2rad(drift))
            sin_drift = np.sin(np.deg2rad(drift))

            self.waypoint_dist[agent] = wpt_dist
            # faf check, reimplemented here VS separate function for completeness
            if wpt_dist<DISTANCE_MARGIN and bs.traf.fafreach[ac_idx]==0:
                bs.traf.fafreach[ac_idx] = 1 # condition for reaching the FAF

            # Get agent aircraft airspeed, m/s
            airspeed = bs.traf.tas[ac_idx]

            vx = np.cos(np.deg2rad(ac_hdg)) * bs.traf.gs[ac_idx]
            vy = np.sin(np.deg2rad(ac_hdg)) * bs.traf.gs[ac_idx]

            ac_loc = self.latlong_to_nm(CENTER, np.array([bs.traf.lat[ac_idx], bs.traf.lon[ac_idx]])) * NM2KM * 1000 # Two-step conversion lat/long -> NM -> m
            x = ac_loc[0]
            y = ac_loc[1]

            observation = {
                "cos(drift)": np.array([cos_drift]),
                "sin(drift)": np.array([sin_drift]),
                "airspeed": np.array([(airspeed-150)/50]),
                "x": np.array([x/50000]),
                "y": np.array([y/50000]),
                "vx": np.array([vx/150]),
                "vy": np.array([vy/150])
            }

            obs.append(np.concatenate(list(observation.values())))

        observations = {
            a: o
            for a, o in zip(self.agents, obs)
        }
        # print(observations)
        return observations       

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
        # compute all distances and headings, for every vehicle
        qdr2pc, dist2pc = kwikqdrdist_matrix(np.full(np.shape(traf.lat), pc_lat), np.full(np.shape(traf.lat), pc_lon) ,traf.lat, traf.lon)

        # if testing:
            # print("qdr", qdr2pc)
            # print("dist", dist2pc)
            # print("phase", traf.phase)
            # print("pad", traf.assigned_pad)

        if np.shape(dist2pc)[0]>0:
            # mask_landing = (np.array(traf.phase)==1) & (np.array(dist2pc)<0.99)
            mask_landing = (np.array(traf.phase)==1) & (np.array(dist2pc)<=1) # 1 nautical mile, so 1852m - only agents coming in for landing
            # mask_landing = (np.array(traf.phase)==1)
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
                # mask_needed = (traf.assigned_pad==pad_index).astype(int)
                # print(mask_needed, len(mask_needed))
                if sum(mask_needed)>1:
                    # print("IN PAD LOOP")
                    # route, spawn_loc, del_loc, return_loc, coords, ind_pad, reso_needed, takeoff_pads, landing_pads, fafs, routelen = quick_OD_gen()
                    self.wpt_lat, self.wpt_lon = self.fafs[pad_index][0], self.fafs[pad_index][1]
                    self.rwy_lat, self.rwy_lon = self.landing_pads[pad_index][0], self.landing_pads[pad_index][1]
                    needed_ids = np.array(bs.traf.id)[mask_needed==1]
                    # print(f"Needed IDs are: {needed_ids} for pad {pad_index}")
                    self.agents = needed_ids
                    # self.wpt_dist = np.zeros_like(self.agents)
                    
                    # get observation 
                    self.num_ac = len(self.agents)
                    # self.idxs = 
                    observations = self._get_observation()
                    # import code
                    # code.interact(local=locals())
                    # print(observations)


                    # n_veh = len(self.agents)
                    action_dim = 2
                    
                    # # load model for N_veh
                    # buffer = ReplayBuffer(obs_dim = 7,
                    #   action_dim = action_dim,
                    #   n_agents = n_veh,
                    #   size = int(4e6),
                    #   batch_size = 1024)
                    # self.actor = MultiHeadAdditiveActorBasic(q_dim=3, kv_dim=7, out_dim=action_dim, num_heads=3) #FALLBACK 1
                    # critic_q = MultiHeadAdditiveCriticQv3Basic(q_dim=5, kv_dim=7, num_heads=3)
                    # critic_q_target = MultiHeadAdditiveCriticQv3Basic(q_dim=5, kv_dim=7, num_heads=3)
                    # weights_folder = "/Users/sasha/Documents/Code/multiagent_merge/bluesky-gym/sac_cr_att_sas_vlim_20" # for merge
                    # weights_folder = "/Users/sasha/Documents/Code/multiagent_merge/bluesky-gym/sac_cr_att_sas_vlim_20" # FALLBACK 2
                    # # weights_folder = "/Users/sasha/Documents/Code/multiagent_merge/bluesky-gym/sac_cr_att_per_clean" # for CR
                    # self.actor.load_state_dict(torch.load(f"{weights_folder}/actor.pt")) # FALLBACK 3
                    # critic_q.load_state_dict(torch.load(f"{weights_folder}/qf.pt"))
                    # critic_q_target.load_state_dict(torch.load(f"{weights_folder}/qf_target.pt"))
                    # # print("MODELS LOADED")

                    
                    # sac = SAC(
                    #     action_dim=action_dim,
                    #     buffer=buffer,
                    #     actor=self.actor,
                    #     critic_q=critic_q,
                    #     critic_q_target=critic_q_target
                    #     )

                    # # get actions
                    # for agent in self.agents:
                    #     observation = observations[agent]
                    obs_array = np.stack(list(observations.values()))
                    obs_tensor = torch.as_tensor(obs_array, dtype=torch.float32, device=torch.device("cpu")).unsqueeze(0) 
                    action_tensor, _ = self.actor(obs_tensor)
                    actions = action_tensor[0].detach().cpu().numpy() 
                    # print(f"{observations} are the obs")
                    # print(f"the total list is {self.agents} with len {n_veh}")
                    # action = self.get_action_mod(observation)
                    # print(actions)
                    for i in range(len(self.agents)):
                        action = actions[i]
                        dh = action[0] * D_HEADING
                        dv = action[1] * D_SPEED
                        heading_new = self.bound_angle_positive_negative_180(bs.traf.hdg[bs.traf.id2idx(self.agents[i])] + dh)
                        speed_new_c = (bs.traf.tas[bs.traf.id2idx(self.agents[i])] + dv) * MpS2Kt 
                        speed_new = speed_new_c if speed_new_c>=0 else 1 # bound above negative speed

                        #` print(speed_new)
                        if bs.traf.fafreach[traf.id2idx(self.agents[i])]==0:
                            bs.stack.stack(f"{self.agents[i]} lnav off")
                            bs.stack.stack(f"{self.agents[i]} vnav off")
                            bs.stack.stack(f"HDG {self.agents[i]} {heading_new}")
                            bs.stack.stack(f"SPD {self.agents[i]} {speed_new}")
                        # else:
                        # print("route print: ", bs.traf.ap.route[i])
                        # if len(bs.traf.ap.route[i]) != 0:
                        # else:
                        bs.stack.stack(f"{self.agents[i]} lnav on")
                        bs.stack.stack(f"{self.agents[i]} vnav on")
        return 

    @classmethod
    def reset(self):
        self.last_id = 0
        return
    