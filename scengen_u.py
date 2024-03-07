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
rpz = 50/nm # [m], converted to nm
dtlook = 15 #[s]
own_spd = 20 # [kts]
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
    write_line(text="00:00:00.00> NOISE ON", hdg=heading, spd=speed) # load adsl plugin
    write_line(text="00:00:00.00> asas on", hdg=heading, spd=speed)
    write_line(text="00:00:00.00> plugin load adsl", hdg=heading, spd=speed) # load adsl plugin
    write_line(text="00:00:00.00> IMPL ADSB ADSL", hdg=heading, spd=speed) # load adsl plugin 
    write_line(text="00:00:00.00> plugin load DetectADSL", hdg=heading, spd=speed) # load detection
    write_line(text="00:00:00.00> asas detectADSL", hdg=heading, spd=speed) # detection -> set correct
    write_line(text="00:00:00.00> reso MVP", hdg=heading, spd=speed) # MVP on
    write_line(text="00:00:00.00> plugin load traffichandler", hdg=heading, spd=speed) # handler
    write_line(text="00:00:00.00> plugin load m22logger", hdg=heading, spd=speed) # log
    write_line(text="00:00:00.00> pan 50 4", hdg=heading, spd=speed)
    write_line(text="00:00:00.00> zoom 25", hdg=heading, spd=speed)
    write_line(text=f"00:00:00.00> rpz {rpz}", hdg=heading, spd=speed) # set rpz
    write_line(text=f"00:00:00.00> dtlook {dtlook}", hdg=heading, spd=speed) # set lookahead
    write_line(text=f"00:00:00.00> startlogs", hdg=heading, spd=speed) # start logs



    ownship_id = 'D1'
    intruder_id = 'D2'

    # Spawn ownship, always same position. ID D1 for ownship.
    text = f"{bstime(0)}>CRE {ownship_id} M600 {ownship_spawnloc[0]} {ownship_spawnloc[1]} 0 100 {own_spd}" # ownship parameters are not varied
    write_line(text=text, hdg=heading, spd=speed) # hdg and speed only used for filename purposes here

    # Spawn Intruder: need rel. heading and speed. ID is D2. Use CRECONFS function
    dcpa = 25/nm
    dH = 0
    tlosv = 0

    text =  f"{bstime(0)}>CRECONFS {intruder_id} M600 {ownship_id} {heading} {dcpa} {dtlook*1.1} {dH} {tlosv} {speed}"
    write_line(text=text, hdg=heading, spd=speed) # idem

    return 


"""
Scenario Generator loop
"""

rel_headings = np.arange(6,8,1) # 1 degree increments of rel. headings
v_intruder =  [5, 15, 25, 35] # [kts]

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
hours_max = 0

filename = f"scenario/ADSL/ADSL_batch.scn"

for scenario_name in namelist:
    scname = scenario_name.split(".")[0]
    write_to_batch(f"00:00:00.00>SCEN {scname}",filename)
    write_to_batch(f"00:00:00.00>PCALL {scenario_name}",filename)
    write_to_batch(f"00:00:00.00> ff",filename)
    write_to_batch(f"{hours_max}:03:00.00>HOLD", filename)
