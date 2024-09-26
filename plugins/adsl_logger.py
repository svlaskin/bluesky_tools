import bluesky as bs
from bluesky.core import Entity, timed_function
from bluesky.stack import command
from bluesky import stack
from bluesky.tools import datalog
from bluesky.tools.aero import ft

import pandas as pd
import datetime

def init_plugin():
    # Configuration parameters
    config = {
        'plugin_name': 'adsl_logger',
        'plugin_type': 'sim',
        'reset': reset
    }
    
    bs.traf.AdslLogger = AdslLogger()
    return config

def reset():
    bs.traf.AdslLogger.reset()

class AdslLogger(Entity):
    def __init__(self):
        super().__init__()
        # Create the loggers
        self.conflict_list = []
        self.los_list = []
        self.log_dir = "output"
        
    def reset(self):
        # Reset the loggers and all the vars
        self.conflict_list = []
        self.los_list = []

    @timed_function(name="print_param_adsl", dt=0.5)
    def print_param_adsl(self):
        # stack.stack('ECHO hpos_noise: {}, update_prob: {}'.format(self.hpos_noise_m, self.update_prob))
        # stack.stack('ECHO traf_lat: {}, adsb_lat: {}'.format(bs.traf.lat[0], bs.traf.adsb.lat[0]))
        confpairs = bs.traf.cd.confpairs_unique_real
        lospairs = bs.traf.cd.lospairs_unique_real

        # if(len(confpairs) > 0):
        #     stack.stack('ECHO In conflict: {}'.format(confpairs[0]))

        # stack.stack('ECHO In LoS: {}'.format(lospairs))

    @stack.command(name="LogCPA")
    def log_cpa(self, scenario_name, rpz: float):
        cpa_los_sev = []

        df_1 = pd.DataFrame(bs.traf.cd.dist_closest, index=bs.traf.id, columns=bs.traf.id)

        current_datetime = datetime.datetime.now()
        formatted_datetime = current_datetime.strftime("%Y_%m_%d_%H_%M_%S")

        dictionary_cpa = {(index, column): (rpz-value)/rpz*100 for index, row in df_1.iterrows() for column, value in row.items() if value < 100}
        values = list(dictionary_cpa.values())
        keys = list(dictionary_cpa.keys())

        df = pd.DataFrame({'los_sev_pair': keys, 'los_sev_val': values})

        df.to_csv(f'{self.log_dir}/dist_{scenario_name}_{formatted_datetime}.log')
        
        return
        