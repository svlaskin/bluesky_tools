''' State-based conflict detection. '''
import numpy as np
from bluesky import stack, settings, traf
from bluesky.tools import geo
from bluesky.tools.aero import nm, ft
from bluesky.traffic.asas import ConflictDetection
import bluesky as bs

from math import radians, degrees, cos, sin, sqrt

def init_plugin():
    ''' Plugin initialisation function. '''

    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'DetectADSL',

        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim',
        }

    # init_plugin() should always return a configuration dict.
    return config

class DetectADSL(ConflictDetection):
    ''' Base class for Conflict Detection implementations. '''
    def __init__(self):
        super().__init__()
        ## Default values
        # [m] Horizontal separation minimum for detection
        self.rpz_def = settings.asas_pzr * nm
        self.global_rpz = True
        # [m] Vertical separation minimum for detection
        self.hpz_def = settings.asas_pzh * ft
        self.global_hpz = True
        # [s] lookahead time
        self.dtlookahead_def = settings.asas_dtlookahead
        self.global_dtlook = True
        self.dtnolook_def = 0.0
        self.global_dtnolook = True

        #-------Variables without ADSL effect
        # Conflicts and LoS detected in the current timestep (used for resolving)
        self.confpairs = list()
        self.confpairs_groundtruth = list()
        self.lospairs = list()
        self.lospairs_groundtruth = list()
        self.lospairs_real = list()
        self.qdr = np.array([])
        self.dist = np.array([])
        self.dcpa = np.array([])
        self.tcpa = np.array([])
        self.tLOS = np.array([])
        self.cpa_closest = np.array([])
        self.dist_closest = np.array([])

        # Unique conflicts and LoS in the current timestep (a, b) = (b, a)
        self.confpairs_unique = set()
        self.confpairs_unique_real = set()
        self.lospairs_unique = set()
        self.lospairs_unique_real = set()

        # All conflicts and LoS since simt=0
        self.confpairs_all = list()
        self.lospairs_all = list()
        self.confpairs_all_real = list()
        self.lospairs_all_real = list()
        
        # Per-aircraft conflict data
        with self.settrafarrays():
            self.inconf = np.array([], dtype=bool)  # In-conflict flag
            self.tcpamax = np.array([]) # Maximum time to CPA for aircraft in conflict
            # [m] Horizontal separation minimum for detection
            self.rpz = np.array([])
            # [m] Vertical separation minimum for detection
            self.hpz = np.array([])
            # [s] lookahead time
            self.dtlookahead = np.array([])
            self.dtnolook = np.array([])

        self.nb_true_positive = 0
        self.nb_false_positive = 0
        self.nb_true_negative = 0
        self.nb_false_negative = 0

        self.use_adsl = True

    def create(self, n):
        super().create(n)
        # Initialise values of own states
        # self.rpz[-n:] = 50 if traf.adsb.ac_cat[-n:] else self.rpz_def
        self.rpz[-n:] = self.rpz_def
        self.hpz[-n:] = self.hpz_def
        # self.dtlookahead[-n:] = 50 if traf.adsb.ac_cat[-n:] else self.dtlookahead_def
        self.dtlookahead[-n:] = self.dtlookahead_def
        self.dtnolook[-n:] = self.dtnolook_def
        
        self.cpa_closest = np.full((traf.ntraf, traf.ntraf), 999)
        self.dist_closest = np.full((traf.ntraf, traf.ntraf), 999)

    def detect(self, ownship, intruder, rpz, hpz, dtlookahead):
        confpairs, lospairs, inconf, tcpamax, qdr, \
            dist, dcpa, tcpa, tLOS, swlos, cpa_all, dist_all = \
                self.detect_ideal(ownship, intruder, rpz, hpz, dtlookahead)
        
        confpairs_adsl, lospairs_adsl, inconf_adsl, tcpamax_adsl, qdr_adsl, \
            dist_adsl, dcpa_adsl, tcpa_adsl, tLOS_adsl, swlos_adsl, dist_adsl_all = \
                self.detect_adsl(ownship, intruder, rpz, hpz, dtlookahead)
        
        # self.cpa2_closest = 
        cpa_update_indices = np.logical_and(swlos, cpa_all < self.cpa_closest)
        dist_update_indices = np.logical_and(swlos, dist_all < self.dist_closest)
        self.cpa_closest[cpa_update_indices] = cpa_all[cpa_update_indices]
        self.dist_closest[dist_update_indices] = dist_all[dist_update_indices]
        
        #check false positive
        for pair_adsl in confpairs_adsl:
            if(pair_adsl not in confpairs):
                self.nb_false_positive += 1
            else:
                self.nb_true_positive += 1

        #check false negative
        for pair in confpairs:
            if(pair not in confpairs_adsl):
                self.nb_false_negative += 1
            # else:
            #     self.nb_true_negative += 1

        if(self.use_adsl):
            return confpairs_adsl, lospairs_adsl, inconf_adsl, tcpamax_adsl, \
            qdr_adsl, dist_adsl, dcpa_adsl, tcpa_adsl, tLOS_adsl
        else:
            return confpairs, lospairs, inconf, tcpamax, \
                qdr, dist, dcpa, tcpa, tLOS

        

    def update(self, ownship, intruder):
        ''' Perform an update step of the Conflict Detection implementation. '''        
        self.confpairs, self.lospairs, self.inconf, self.tcpamax, self.qdr, \
            self.dist, self.dcpa, self.tcpa, self.tLOS = \
                self.detect(ownship, intruder, self.rpz, self.hpz, self.dtlookahead)

        # confpairs has conflicts observed from both sides (a, b) and (b, a)
        # confpairs_unique keeps only one of these
        confpairs_unique = {frozenset(pair) for pair in self.confpairs}
        lospairs_unique = {frozenset(pair) for pair in self.lospairs}

        self.confpairs_all.extend(confpairs_unique - self.confpairs_unique)
        self.lospairs_all.extend(lospairs_unique - self.lospairs_unique)

        self.confpairs_all = list(set(self.confpairs_all))
        self.lospairs_all = list(set(self.lospairs_all))

        # Update confpairs_unique and lospairs_unique
        self.confpairs_unique = confpairs_unique
        self.lospairs_unique = lospairs_unique

        # get the real metrics, instead of the measured one
        self.confpairs_groundtruth, self.lospairs_groundtruth, inconf, tcpamax, qdr, \
            dist, dcpa, tcpa, tLOS, swlos, cpa_all, dist_all = \
                self.detect_ideal(ownship, intruder, self.rpz, self.hpz, self.dtlookahead)
        
        # confpairs has conflicts observed from both sides (a, b) and (b, a)
        # confpairs_unique keeps only one of these
        confpairs_unique_real = {frozenset(pair) for pair in self.confpairs_groundtruth}
        lospairs_unique_real = {frozenset(pair) for pair in self.lospairs_groundtruth}

        self.confpairs_all_real.extend(confpairs_unique_real - self.confpairs_unique_real)
        self.lospairs_all_real.extend(lospairs_unique_real - self.lospairs_unique_real)

        self.confpairs_all_real = list(set(self.confpairs_all_real))
        self.lospairs_all_real = list(set(self.lospairs_all_real))

        # Update confpairs_unique and lospairs_unique
        self.confpairs_unique_real = confpairs_unique_real
        self.lospairs_unique_real = lospairs_unique_real

    def reset(self):
        super().reset()
        self.clearconfdb()
        self.confpairs_all.clear()
        self.lospairs_all.clear()
        self.confpairs_all_real.clear()
        self.lospairs_all_real.clear()
        self.rpz_def = bs.settings.asas_pzr * nm
        self.hpz_def = bs.settings.asas_pzh * ft
        self.dtlookahead_def = bs.settings.asas_dtlookahead
        self.dtnolook_def = 0.0
        self.global_rpz = self.global_hpz = True
        self.global_dtlook = self.global_dtnolook = True

    def clearconfdb(self):
        ''' Clear conflict database. '''
        self.confpairs_unique.clear()
        self.lospairs_unique.clear()
        self.confpairs_unique_real.clear()
        self.lospairs_unique_real.clear()
        self.confpairs.clear()
        self.lospairs.clear()
        self.qdr = np.array([])
        self.dist = np.array([])
        self.dcpa = np.array([])
        self.tcpa = np.array([])
        self.tLOS = np.array([])
        self.inconf = np.zeros(bs.traf.ntraf)
        self.tcpamax = np.zeros(bs.traf.ntraf)

    def detect_ideal(self, ownship, intruder, rpz, hpz, dtlookahead):
        ''' Conflict detection between ownship (traf) and intruder (traf/adsb).'''
        # Identity matrix of order ntraf: avoid ownship-ownship detected conflicts

        I = np.eye(ownship.ntraf)

        # Horizontal conflict ------------------------------------------------------
        # qdrlst is for [i,j] qdr from i to j, from perception of ADSB and own coordinates
        qdr, dist = geo.kwikqdrdist_matrix(np.asmatrix(ownship.lat), np.asmatrix(ownship.lon),
                                    np.asmatrix(intruder.lat), np.asmatrix(intruder.lon))

        # Convert back to array to allow element-wise array multiplications later on
        # Convert to meters and add large value to own/own pairs
        qdr = np.asarray(qdr)
        dist = np.asarray(dist) * nm + 1e9 * I

        # Calculate horizontal closest point of approach (CPA)
        qdrrad = np.radians(qdr)
        dx = dist * np.sin(qdrrad)  # is pos j rel to i
        dy = dist * np.cos(qdrrad)  # is pos j rel to i

        # Ownship track angle and speed
        owntrkrad = np.radians(ownship.trk)
        ownu = ownship.gs * np.sin(owntrkrad).reshape((1, ownship.ntraf))  # m/s
        ownv = ownship.gs * np.cos(owntrkrad).reshape((1, ownship.ntraf))  # m/s

        # Intruder track angle and speed
        inttrkrad = np.radians(intruder.trk)
        intu = intruder.gs * np.sin(inttrkrad).reshape((1, ownship.ntraf))  # m/s
        intv = intruder.gs * np.cos(inttrkrad).reshape((1, ownship.ntraf))  # m/s

        du = ownu - intu.T  # Speed du[i,j] is perceived eastern speed of i to j
        dv = ownv - intv.T  # Speed dv[i,j] is perceived northern speed of i to j

        dv2 = du * du + dv * dv
        dv2 = np.where(np.abs(dv2) < 1e-6, 1e-6, dv2)  # limit lower absolute value
        vrel = np.sqrt(dv2)

        tcpa = -(du * dx + dv * dy) / dv2 + 1e9 * I

        # Calculate distance^2 at CPA (minimum distance^2)
        dcpa2 = np.abs(dist * dist - tcpa * tcpa * dv2)

        # Check for horizontal conflict
        # RPZ can differ per aircraft, get the largest value per aircraft pair
        rpz = np.asarray(np.maximum(np.asmatrix(rpz), np.asmatrix(rpz).transpose()))
        R2 = rpz * rpz
        swhorconf = dcpa2 < R2  # conflict or not

        # Calculate times of entering and leaving horizontal conflict
        dxinhor = np.sqrt(np.maximum(0., R2 - dcpa2))  # half the distance travelled inzide zone
        dtinhor = dxinhor / vrel

        tinhor = np.where(swhorconf, tcpa - dtinhor, 1e8)  # Set very large if no conf
        touthor = np.where(swhorconf, tcpa + dtinhor, -1e8)  # set very large if no conf

        # Vertical conflict --------------------------------------------------------

        # Vertical crossing of disk (-dh,+dh)
        dalt = ownship.alt.reshape((1, ownship.ntraf)) - \
            intruder.alt.reshape((1, ownship.ntraf)).T  + 1e9 * I

        dvs = ownship.vs.reshape(1, ownship.ntraf) - \
            intruder.vs.reshape(1, ownship.ntraf).T
        dvs = np.where(np.abs(dvs) < 1e-6, 1e-6, dvs)  # prevent division by zero

        # Check for passing through each others zone
        # hPZ can differ per aircraft, get the largest value per aircraft pair
        hpz = np.asarray(np.maximum(np.asmatrix(hpz), np.asmatrix(hpz).transpose()))
        tcrosshi = (dalt + hpz) / -dvs
        tcrosslo = (dalt - hpz) / -dvs
        tinver = np.minimum(tcrosshi, tcrosslo)
        toutver = np.maximum(tcrosshi, tcrosslo)

        # Combine vertical and horizontal conflict----------------------------------
        tinconf = np.maximum(tinver, tinhor)
        toutconf = np.minimum(toutver, touthor)

        swconfl = np.array(swhorconf * (tinconf <= toutconf) * (toutconf > 0.0) *
                           np.asarray(tinconf < np.asmatrix(dtlookahead).T) * (1.0 - I), dtype=bool)

        # --------------------------------------------------------------------------
        # Update conflict lists
        # --------------------------------------------------------------------------
        # Ownship conflict flag and max tCPA
        inconf = np.any(swconfl, 1)
        tcpamax = np.max(tcpa * swconfl, 1)

        # Select conflicting pairs: each a/c gets their own record
        confpairs = [(ownship.id[i], ownship.id[j]) for i, j in zip(*np.where(swconfl))]
        swlos = (dist < rpz) * (np.abs(dalt) < hpz)
        lospairs = [(ownship.id[i], ownship.id[j]) for i, j in zip(*np.where(swlos))]

        #update value if in los
        # update_indices = np.logical_and(swlos, dcpa2 < closest_ever)
        # closest_ever[update_indices] = dcpa2[update_indices]

        # # print(swconfl, dist, swlos)
        # print(dcpa2, closest_ever)

        return confpairs, lospairs, inconf, tcpamax, \
            qdr[swconfl], dist[swconfl], np.sqrt(dcpa2[swconfl]), \
                tcpa[swconfl], tinconf[swconfl], swlos, np.sqrt(dcpa2), dist
    
    def detect_adsl(self, ownship, intruder, rpz, hpz, dtlookahead):
        ''' Conflict detection between ownship (traf) and intruder (traf/adsb).'''
        # Identity matrix of order ntraf: avoid ownship-ownship detected conflicts

        I = np.eye(ownship.ntraf)

        # Horizontal conflict ------------------------------------------------------
        # qdrlst is for [i,j] qdr from i to j, from perception of ADSB and own coordinates

        own_lat_measured = ownship.lat + ownship.adsb.delta_lat
        own_lon_measured = ownship.lon + ownship.adsb.delta_lon
        own_gs_measured = ownship.gs + ownship.adsb.delta_gs

        qdr, dist = geo.kwikqdrdist_matrix(np.asmatrix(own_lat_measured), np.asmatrix(own_lon_measured),
                                    np.asmatrix(intruder.adsb.lat), np.asmatrix(intruder.adsb.lon))

        # Convert back to array to allow element-wise array multiplications later on
        # Convert to meters and add large value to own/own pairs
        qdr = np.asarray(qdr)
        dist = np.asarray(dist) * nm + 1e9 * I

        # Calculate horizontal closest point of approach (CPA)
        qdrrad = np.radians(qdr)
        dx = dist * np.sin(qdrrad)  # is pos j rel to i
        dy = dist * np.cos(qdrrad)  # is pos j rel to i

        ## add noise for the ownship
        

        # Ownship track angle and speed
        owntrkrad = np.radians(ownship.trk)
        ownu = own_gs_measured * np.sin(owntrkrad).reshape((1, ownship.ntraf))  # m/s
        ownv = own_gs_measured * np.cos(owntrkrad).reshape((1, ownship.ntraf))  # m/s

        # Intruder track angle and speed
        inttrkrad = np.radians(intruder.adsb.trk)
        intu = intruder.adsb.gs * np.sin(inttrkrad).reshape((1, ownship.ntraf))  # m/s
        intv = intruder.adsb.gs * np.cos(inttrkrad).reshape((1, ownship.ntraf))  # m/s

        du = ownu - intu.T  # Speed du[i,j] is perceived eastern speed of i to j
        dv = ownv - intv.T  # Speed dv[i,j] is perceived northern speed of i to j

        dv2 = du * du + dv * dv
        dv2 = np.where(np.abs(dv2) < 1e-6, 1e-6, dv2)  # limit lower absolute value
        vrel = np.sqrt(dv2)

        tcpa = -(du * dx + dv * dy) / dv2 + 1e9 * I

        # Calculate distance^2 at CPA (minimum distance^2)
        dcpa2 = np.abs(dist * dist - tcpa * tcpa * dv2)

        # Check for horizontal conflict
        # RPZ can differ per aircraft, get the largest value per aircraft pair
        rpz = np.asarray(np.maximum(np.asmatrix(rpz), np.asmatrix(rpz).transpose()))
        R2 = rpz * rpz
        swhorconf = dcpa2 < R2  # conflict or not

        # Calculate times of entering and leaving horizontal conflict
        dxinhor = np.sqrt(np.maximum(0., R2 - dcpa2))  # half the distance travelled inzide zone
        dtinhor = dxinhor / vrel

        tinhor = np.where(swhorconf, tcpa - dtinhor, 1e8)  # Set very large if no conf
        touthor = np.where(swhorconf, tcpa + dtinhor, -1e8)  # set very large if no conf

        # Vertical conflict --------------------------------------------------------

        # Vertical crossing of disk (-dh,+dh)
        dalt = ownship.alt.reshape((1, ownship.ntraf)) - \
            intruder.alt.reshape((1, ownship.ntraf)).T  + 1e9 * I

        dvs = ownship.vs.reshape(1, ownship.ntraf) - \
            intruder.vs.reshape(1, ownship.ntraf).T
        dvs = np.where(np.abs(dvs) < 1e-6, 1e-6, dvs)  # prevent division by zero

        # Check for passing through each others zone
        # hPZ can differ per aircraft, get the largest value per aircraft pair
        hpz = np.asarray(np.maximum(np.asmatrix(hpz), np.asmatrix(hpz).transpose()))
        tcrosshi = (dalt + hpz) / -dvs
        tcrosslo = (dalt - hpz) / -dvs
        tinver = np.minimum(tcrosshi, tcrosslo)
        toutver = np.maximum(tcrosshi, tcrosslo)

        # Combine vertical and horizontal conflict----------------------------------
        tinconf = np.maximum(tinver, tinhor)
        toutconf = np.minimum(toutver, touthor)

        swconfl = np.array(swhorconf * (tinconf <= toutconf) * (toutconf > 0.0) *
                           np.asarray(tinconf < np.asmatrix(dtlookahead).T) * (1.0 - I), dtype=bool)

        # --------------------------------------------------------------------------
        # Update conflict lists
        # --------------------------------------------------------------------------
        # Ownship conflict flag and max tCPA
        inconf = np.any(swconfl, 1)
        tcpamax = np.max(tcpa * swconfl, 1)

        # Select conflicting pairs: each a/c gets their own record
        confpairs = [(ownship.id[i], ownship.id[j]) for i, j in zip(*np.where(swconfl))]
        swlos = (dist < rpz) * (np.abs(dalt) < hpz)
        lospairs = [(ownship.id[i], ownship.id[j]) for i, j in zip(*np.where(swlos))]

        return confpairs, lospairs, inconf, tcpamax, \
            qdr[swconfl], dist[swconfl], np.sqrt(dcpa2[swconfl]), \
                tcpa[swconfl], tinconf[swconfl], swlos, dist
    
    @stack.command(name='DETECT_USING_ADSL')
    def set_use_adsl(self, cond: int):
        self.use_adsl = bool(cond)
        stack.stack(f'ECHO Using_ADSL set to {self.use_adsl}')

        return
    
    @stack.command(name='ECHO_ADSL')
    def echo_adsl(self):
        stack.stack(f'ECHO {traf.gs} {traf.adsb.gs}')
        stack.stack(f'ECHO {traf.lat} {traf.adsb.lat}')
        stack.stack(f'ECHO {traf.lon} {traf.adsb.lon}')

    @stack.command(name='GET_RPZ')
    def get_rpz(self):
        stack.stack(f'ECHO {self.rpz}')
    
    @stack.command(name='SET_RPZ')
    def set_rpz(self):
        self.rpz = np.array([30, 30])