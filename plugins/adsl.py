""" BlueSky plugin template. The text you put here will be visible
    in BlueSky as the description of your plugin. """
import random
import numpy as np
from math import degrees, radians, sin, cos

# Import the global bluesky objects. Uncomment the ones you need
from bluesky import core, stack, traf, sim  #, settings, navdb, sim, scr, tools
from bluesky.tools.aero import ft

from bluesky.traffic import ADSB
from scipy.stats import halfnorm

### Initialization function of your plugin. Do not change the name of this
### function, as it is the way BlueSky recognises this file as a plugin.
def init_plugin():
    ''' Plugin initialisation function. '''
    adsl = ADSL()

    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'ADSL',

        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim',
        }

    # init_plugin() should always return a configuration dict.
    return config

class ADSL(ADSB):
    ''' Example new entity object for BlueSky. '''
    def __init__(self):
        super().__init__()
        
        self.GT_RES = 360/512
        self.R = 6373.0

        with self.settrafarrays():
            # Most recent broadcast data
            self.lastupdate = np.array([])
            self.lat        = np.array([])
            self.lon        = np.array([])
            self.alt        = np.array([])
            self.trk        = np.array([])
            self.tas        = np.array([])
            self.gs         = np.array([])
            self.vs         = np.array([])

            self.pos_noise_lat = np.array([])
            self.pos_noise_lon = np.array([])

            self.ac_cat = np.array([])
            self.ac_stat = np.array([])

        self.set_nav_noise(True)
        self.set_comm_parameter(False, False)
        self.update_prob = 1.0
        
        self.comm_noise = True
        self.comm_std_dev = 5

        self.time_elapsed_total = []

    def setnoise(self, n):
        self.set_nav_noise(n)
        self.set_comm_parameter(n, n)

    def set_nav_noise(self, cond):
        self.nav_noise = cond
        self.hpos_noise_m = 1.5 # in meter, one standard deviation
        self.gs_noise_ms = 0.0 # in m/s, one standard deviation

    def set_comm_parameter(self, cond_trunc, cond_reso):
        self.comm_trunc = cond_trunc
        self.comm_reso = cond_reso
        self.trunctime = 0

    def create(self, n=1):
        super().create(n)
        
        # self.lastupdate[-n:] = -self.trunctime * np.random.rand(n)
        self.lastupdate[-n:] = np.random.normal(0, self.comm_std_dev) # add sim.simt here because ac can be spawned at any sim.simt time
        self.lat[-n:] = traf.lat[-n:]
        self.lon[-n:] = traf.lon[-n:]
        self.alt[-n:] = traf.alt[-n:]
        self.trk[-n:] = traf.trk[-n:]
        self.tas[-n:] = traf.tas[-n:]
        self.gs[-n:]  = traf.gs[-n:]

        self.ac_cat[-n:] = self.is_drone(traf.type[-n])
        self.ac_stat[-n:] = random.choices([0, 1], weights=(90, 10), k=1)

    def update(self):

        # up = np.where(self.lastupdate + self.trunctime < sim.simt)

        if(self.comm_noise):
            time_elapsed = sim.simt - self.lastupdate
            update_prob = self.update_prob
            up = np.where(np.random.random(size = traf.ntraf) < update_prob)
        else:
            up = np.array([True] * traf.ntraf)

        nup = len(up)
        
        if self.nav_noise:
            delta_lat, delta_lon = self.get_lat_lon_noise(self.hpos_noise_m, self.lat, self.lon, up)
            self.lat[up] += delta_lat
            self.lon[up] += delta_lon

            self.gs[up] = traf.gs[up] + np.random.normal(0, self.gs_noise_ms, nup)
        else:
            self.lat[up] = traf.lat[up]
            self.lon[up] = traf.lon[up]
            self.alt[up] = traf.alt[up]

        if(self.comm_reso):
            self.gs = self.encode_decode_gs(self.gs[up])
            self.trk = self.encode_decode_trk(traf.trk[up])

        self.tas[up] = traf.tas[up]
        self.vs[up]  = traf.vs[up]

        self.lastupdate[up] = sim.simt
        self.time_elapsed_total.extend(time_elapsed[up])

    def reset(self):
        super().reset()
        self.time_elapsed_total.clear()

    def is_drone(self, ac_type):
        drone_list = ["M600", "Amzn", "Mnet", "Phan4", "M100", "M200", "Mavic", "Horsefly"]
        drone_list = [type.upper() for type in drone_list]

        return ac_type in drone_list
    
    ## ADS-L.4.SRD860.G.1.7
    def decode_alt(self, encoded_value):
        base_bit = 12
        exp_bit = 2

        bin_base = 2**base_bit - 1
        bin_exp = 2**exp_bit - 1

        exponent = (encoded_value >> base_bit) & bin_exp
        base = encoded_value & bin_base
        
        value = (2**exponent * (2**base_bit + base) - 2**base_bit) - 320
        
        return value

    def encode_alt(self, decoded_value):
        normalized_value = decoded_value + 320
        base_bit = 12
        exp_list = [0, 1, 2, 3]
        
        test_val = [2**exponent * (2**base_bit) - 2**base_bit for exponent in exp_list]
        e_bool = [normalized_value > val for val in test_val]
        e_star = [exp_list[i] for i in range(len(exp_list)) if e_bool[i]][-1]
        
        base = (normalized_value + 2**base_bit)/(2**e_star) - 2**base_bit
        
        if(base == 2**base_bit):
            e_star -= -1
            base = 0
            
        return e_star, round(base)
    
    def encode_decode_alt(self, alt):
        base_bit = 12
        exponent, base = self.encode_alt(alt)
        return (2**exponent * (2**base_bit + base) - 2**base_bit) - 320


    def decode_gs(encoded_value):
        exp_bit = 2
        base_bit = 6
        
        bin_base = 2**base_bit - 1 #bin_base = 0b111111 = 63
        bin_exp = 2**exp_bit - 1 #bin_base = 0b11 = 3

        exponent = (encoded_value >> base_bit) & bin_exp
        base = encoded_value & bin_base
        
        value = (2**exponent * (2**base_bit + base) - 2**base_bit)*0.25
        
        return value, exponent, base

    def encode_gs(self, decoded_value):
        normalized_value = 1 if decoded_value < 0 else decoded_value/0.25 

        base_bit = 6
        exp_list = [0, 1, 2, 3]
        
        test_val = [2**exponent * (2**base_bit) - 2**base_bit for exponent in exp_list]
        e_bool = [normalized_value > val for val in test_val]

        try:
            e_star = [exp_list[i] for i in range(len(exp_list)) if e_bool[i]][-1]
        except Exception as error:
            # handle the exception
            print("An exception occurred:", error) # An exception occurred: division by zero
            print(normalized_value)
        
        base = (normalized_value + 2**base_bit)/(2**e_star) - 2**base_bit
        
        if(base == 2**base_bit):
            e_star -= -1
            base = 0
            
        return e_star, round(base)
    
    def encode_decode_gs_single(self, gs):
        base_bit = 6
        exponent, base = self.encode_gs(gs)
        return (2**exponent * (2**base_bit + base) - 2**base_bit)*0.25
    
    def encode_decode_gs(self, vector_gs):
        func = np.vectorize(self.encode_decode_gs_single)
        return func(vector_gs)

    def encode_ground_track(self, ground_track):
        if ground_track is None:
            return None
        return int(round(ground_track / self.GT_RES)) % 512

    def decode_ground_track(self, encoded_ground_track):
        if encoded_ground_track is None:
            return None
        return encoded_ground_track * self.GT_RES
    
    def encode_decode_trk_single(self, trk):
        trk_adsl_encoded = self.encode_ground_track(trk)
        return self.decode_ground_track(trk_adsl_encoded)
    
    def encode_decode_trk(self, vec_trk):
        func = np.vectorize(self.encode_decode_trk_single)
        return func(vec_trk)

    def decode_vs(encoded_value):
        sign_bit = 1
        exp_bit = 2
        base_bit = 6
        
        bin_sign = 2**sign_bit - 1
        bin_base = 2**base_bit - 1
        bin_exp = 2**exp_bit - 1

        sign = (encoded_value >> (base_bit + exp_bit)) & bin_sign
        exponent = (encoded_value >> base_bit) & bin_exp
        base = encoded_value & bin_base
        
        value = (2**exponent * (2**base_bit + base) - 2**base_bit)*0.125

        if(sign):
            value *= -1
        
        return value, exponent, base

    def encode_vs(self, decoded_value):
        sign = 1 if decoded_value < 0 else 0
        decoded_value = abs(decoded_value)
        
        normalized_value = decoded_value/0.125
        base_bit = 6
        exp_list = [0, 1, 2, 3]
        
        test_val = [2**exponent * (2**base_bit) - 2**base_bit for exponent in exp_list]
        e_bool = [normalized_value > val for val in test_val]
        e_star = [exp_list[i] for i in range(len(exp_list)) if e_bool[i]][-1]
        
        base = (normalized_value + 2**base_bit)/(2**e_star) - 2**base_bit
        
        if(base == 2**base_bit):
            e_star -= -1
            base = 0
            
        return sign, e_star, round(base)

    @stack.command(name='POS_ADSL')
    def get_full_info(self, acid: 'acid'):
        s1 = f'Info on {traf.id[acid]} {traf.type[acid]}'
        
        if(self.is_drone(acid)):
            s2_1 = f'Cat: Drone'
        else:
            s2_1 = f'Cat: GA'

        if(self.stat[acid]):
            s2_2 = f'Stat: Emergency'
        else:
            s2_2 = f'Stat: Not Emergency'

        s2 = f'{s2_1} {s2_2}'

        s3 = f'Pos: {traf.lat[acid]} {traf.lon[acid]}'
        s4 = f'HDG: {traf.trk[acid]} HDG_ADSL: {self.encode_decode_trk(traf.trk[acid])}'
        s5 = f'ALT: {traf.alt[acid]} ALT_ADSL: {self.encode_decode_alt(traf.alt[acid])}'
        s6 = f'GS: {traf.gs[acid]} GS_ADSL: {self.encode_decode_gs(traf.gs[acid])}'

        sfinal = f'{s1}\n{s2}\n{s3}\n{s4}\n{s5}\n{s6}\n{s7}'

        return True, sfinal

    def get_lat_lon_noise(self, stdev, lat_ref, lon_ref, up):
        nb = len(up[0])

        angles_rad = np.random.uniform(0, 2*np.pi, size = nb)
        distance = np.random.normal(0, stdev, size = nb) / 1000 #km
        
        delta_lat = distance * np.sin(angles_rad) / 110.574
        delta_lon = distance * np.cos(angles_rad) / (111.320*np.cos(np.radians(lat_ref[up])))
        
        return delta_lat, delta_lon


    @core.timed_function(name="print_param_adsl", dt=0.5)
    def print_param_adsl(self):
        stack.stack('ECHO hpos_noise: {}, update_prob: {}'.format(self.hpos_noise_m, self.update_prob))
        return

    @stack.command(name='ADSL_HPOS_NOISE')
    def set_adsl_hpos_noise(self, hpos_noise: float = 1.5):
        self.hpos_noise_m = hpos_noise #m
        stack.stack(f'ECHO ADSL_HPOS_NOISE {self.hpos_noise_m}')

        return 

    @stack.command(name='ADSL_DELAY_STDEV')
    def set_adsl_delay_stdev(self, delay: float = 5):
        self.comm_std_dev = delay
        stack.stack(f'ECHO ADSL_DELAY_STDEV {self.comm_std_dev}')

        return 
    
    @stack.command(name='ADSL_GS_NOISE')
    def get_full_info(self, gs_noise_ms: float = 1.5):
        self.gs_noise_ms = gs_noise_ms #m

        return 