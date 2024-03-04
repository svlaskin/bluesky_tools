"""
Scenario generator for ADSL plugin testing.
Generates 
"""

import pickle as pkl
import datetime
import pandas as pd
import numpy as np
import os, sys
from scipy.stats import gamma
from bluesky.tools.geo import *
sys.path.append(os.path.join(os.path.dirname("adsl_bluesky/scenario/ADSL"), "lib"))

ft  = 0.3048                # m    of 1 foot
nm  = 1852.                 # m    of 1 nautical mile

"""
Constants
"""
cruisealt_out = 100/ft
vcruise = 40
cruisealt_in = 50/ft

"""
Settings
"""
ownship_spawnloc = [50, 4] # lat, lon [deg]
radius = 2000 # radius of conflict circle [m]
theta_res = 1 # angular resolution [deg]
# downlat, downlon, uplat, uplon = '52.2781742', '4.7287563', '52.4310638', '5.0791622'

"""
Functions for scen writing
"""
def flatten(xss):
    return [x for xs in xss for x in xs]

def read_pkl(filename):
    with open(f"{filename}", "rb") as file:
        df = pkl.load(file)
    return df

def write_pkl(filename, dataframe):
    with open(f"{filename}", "wb") as file:
        df = pkl.dump(dataframe, file)
    return

def bstime(time_start):
    return str(datetime.timedelta(seconds=float(time_start)))

def write_line(text,hdg,spd):
    name = f"testscen_{hdg}_{spd}"
    with open(f'scenario/ADSL/{name}.scn', 'a') as fd:
        fd.write(text + '\n')

def write_to_batch(text,fname):
    with open(f'{filename}', 'a') as fd:
            fd.write(text + '\n')
    return

def write_lines_to_scen(heading,speed):
    # Basic setup for sim
    write_line(text="plugin load adsl", hdg=heading, spd=speed) # load adsl plugin
    write_line(text="plugin load DetectADSL", hdg=heading, spd=speed) # load detection
    # one more for loggers needed!!

    # Spawn ownship, always same position
    text = f"{bstime(0)}>CRE own M600 {ownship_spawnloc[0]} {ownship_spawnloc[1]} 0 100 40" # ownship parameters are not varied
    write_line(text=text, hdg=heading, spd=speed) # hdg and speed only used for filename purposes here

    # Spawn Intruder: need rel. heading and speed.
    ilat, ilon = [50, 4] # determine needed lat lon -> rep with function

    intruder_spawnloc = qdrpos(ownship_spawnloc[0],ownship_spawnloc[1],heading+180,dist=radius/nm)

    text =  f"{bstime(0)}>CRE int M600 {intruder_spawnloc[0]} {intruder_spawnloc[1]} {heading} 100 {speed}"
    write_line(text=text, hdg=heading, spd=speed) # idem

    return 


"""
Scenario Generator loop
"""

rel_headings = np.arange(0,360,1) # 1 degree increments of rel. headings
v_intruder = [15, 25, 40] 

# one speed-heading combo is a unique file
for hdg in rel_headings:
    for v in v_intruder:
        name = write_lines_to_scen(hdg,v)

# remove DS_store and find all scen files to put in batch
ds_store_file_location = f"scenario/ADSL/.DS_store"
if os.path.isfile(ds_store_file_location):
    os.remove(ds_store_file_location)
namelist = os.listdir(f"scenario/ADSL")

# determine stoppage: for now 1 hour
hours_max = 1

filename = f"scenario/ADSL/ADSL_batch.scn"

for scenario_name in namelist:
    scname = scenario_name.split(".")[0]
    write_to_batch(f"00:00:00.00>SCEN {scname}",filename)
    write_to_batch(f"00:00:00.00>PCALL {scenario_name}",filename)
    write_to_batch(f"00:00:00.00> ff",filename)
    write_to_batch(f"{hours_max}:00:00.00>HOLD", filename)
