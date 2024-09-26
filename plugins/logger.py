import bluesky as bs
from bluesky import stack
from bluesky.core import Entity
from bluesky.tools.geo import kwikdist
from bluesky.tools.misc import degto180
from bluesky.tools.aero import kts, ft, fpm, nm
from bluesky.core.simtime import timed_function
import numpy as np

@core.timed_function(name="print_param_adsl", dt=0.5)
    def print_param_adsl(self):
        # stack.stack('ECHO hpos_noise: {}, update_prob: {}'.format(self.hpos_noise_m, self.update_prob))
        stack.stack('ECHO traf_lat: {}, adsb_lat: {}'.format(bs.traf.lat, self.lat))
        return

def init_plugin():
    # Configuration parameters
    config = {
        'plugin_name': 'TRAFFICHANDLER',
        'plugin_type': 'sim',
        'reset': reset
    }
    # Put TrafficSpawner in bs.traf
    bs.traf.TrafficHandler = TrafficHandler()
    return config

class DetectADSL(ConflictDetection):
    ''' Base class for Conflict Detection implementations. '''
    def __init__(self):
        super().__init__()