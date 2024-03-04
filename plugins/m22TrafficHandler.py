import bluesky as bs
from bluesky import stack
from bluesky.core import Entity
from bluesky.tools.geo import kwikdist
from bluesky.tools.misc import degto180
from bluesky.tools.aero import kts, ft, fpm, nm
from bluesky.core.simtime import timed_function
import numpy as np


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

def reset():
    bs.traf.TrafficHandler.reset()
    
class TrafficHandler(Entity):
    def __init__(self):
        super().__init__()
        self.cruise_spd = 30 * kts
        self.cruiselayerdiff = 30 * ft
        
        # Logging related stuff
        self.prevconfpairs = set()
        self.prevlospairs = set()
        self.confinside_all = 0
        self.deleted_aircraft = 0
        self.losmindist = dict()
        
        with self.settrafarrays():
            self.allocated_alt = []
            self.street_numbers = []
            self.distance2D = np.array([])
            self.distance3D = np.array([])
            self.distancealt = np.array([])
            
    def create(self, n=1):
        super().create(n)
        # Save the starting altitude
        self.allocated_alt[-n:] = bs.traf.alt[-n:]
        self.street_numbers[-n:] = [None] * n
        self.distance2D[-n:] = [0]*n
        self.distance3D[-n:] = [0]*n
        self.distancealt[-n:] = [0]*n
        
    def reset(self):
        # Logging related stuff
        self.prevconfpairs = set()
        self.prevlospairs = set()
        self.confinside_all = 0
        self.deleted_aircraft = 0
        self.losmindist = dict()
        
        with self.settrafarrays():
            self.allocated_alt = []
            self.street_numbers = []
            self.distance2D = np.array([])
            self.distance3D = np.array([])
            self.distancealt = np.array([])

    @timed_function(dt = 0.5)
    def delete_aircraft(self):
        self.update_logging()
        # Delete aircraft that have LNAV off and have gone past the last waypoint.
        lnav_on = bs.traf.swlnav
        still_going_to_dest = np.logical_and(abs(degto180(bs.traf.trk - bs.traf.ap.qdr2wp)) < 10.0, 
                                       bs.traf.ap.dist2wp > 5)
        delete_array = np.logical_and.reduce((np.logical_not(lnav_on), 
                                         bs.traf.actwp.swlastwp,
                                         np.logical_not(still_going_to_dest)))
        
        if np.any(delete_array):
            # Get the ACIDs of the aircraft to delete
            acids_to_delete = np.array(bs.traf.id)[delete_array]
            for acid in acids_to_delete:
                # Log the stuff for this aircraft in the flstlog
                idx = bs.traf.id.index(acid)
                bs.traf.M22Logger.flst.log(
                    acid,
                    bs.traf.m22delay.create_time[idx],
                    bs.sim.simt - bs.traf.m22delay.create_time[idx],
                    (self.distance2D[idx]),
                    (self.distance3D[idx]),
                    (self.distancealt[idx]),
                    bs.traf.lat[idx],
                    bs.traf.lon[idx],
                    bs.traf.alt[idx]/ft,
                    bs.traf.tas[idx]/kts,
                    bs.traf.vs[idx]/fpm,
                    bs.traf.hdg[idx],
                    bs.traf.cr.active[idx],
                    bs.traf.aporasas.alt[idx]/ft,
                    bs.traf.aporasas.tas[idx]/kts,
                    bs.traf.aporasas.vs[idx]/fpm,
                    bs.traf.aporasas.hdg[idx])
                stack.stack(f'DEL {acid}')
                
            
    def update_logging(self):        
        # Increment the distance metrics
        resultantspd = np.sqrt(bs.traf.gs * bs.traf.gs + bs.traf.vs * bs.traf.vs)
        self.distance2D += bs.sim.simdt * abs(bs.traf.gs)
        self.distance3D += bs.sim.simdt * resultantspd
        self.distancealt += bs.sim.simdt * abs(bs.traf.vs)
        
        # Now let's do the CONF and LOS logs
        confpairs_new = list(set(bs.traf.cd.confpairs) - self.prevconfpairs)
        if confpairs_new:
            done_pairs = []
            for pair in set(confpairs_new):
                # Check if the aircraft still exist
                if (pair[0] in bs.traf.id) and (pair[1] in bs.traf.id):
                    # Get the two aircraft
                    idx1 = bs.traf.id.index(pair[0])
                    idx2 = bs.traf.id.index(pair[1])
                    done_pairs.append((idx1,idx2))
                    if (idx2,idx1) in done_pairs:
                        continue
                        
                    bs.traf.M22Logger.conflog.log(pair[0], pair[1],
                                    bs.traf.lat[idx1], bs.traf.lon[idx1],bs.traf.alt[idx1],
                                    bs.traf.lat[idx2], bs.traf.lon[idx2],bs.traf.alt[idx2])
                
        self.prevconfpairs = set(bs.traf.cd.confpairs)
        
        # Losses of separation as well
        # We want to track the LOS, and log the minimum distance and altitude between these two aircraft.
        # This gives us the lospairs that were here previously but aren't anymore
        lospairs_out = list(self.prevlospairs - set(bs.traf.cd.lospairs))
        
        # Attempt to calculate current distance for all current lospairs, and store it in the dictionary
        # if entry doesn't exist yet or if calculated distance is smaller.
        for pair in bs.traf.cd.lospairs:
            # Check if the aircraft still exist
            if (pair[0] in bs.traf.id) and (pair[1] in bs.traf.id):
                idx1 = bs.traf.id.index(pair[0])
                idx2 = bs.traf.id.index(pair[1])
                # Calculate current distance between them [m]
                losdistance = kwikdist(bs.traf.lat[idx1], bs.traf.lon[idx1], bs.traf.lat[idx2], bs.traf.lon[idx2])*nm
                # To avoid repeats, the dictionary entry is DxDy, where x<y. So D32 and D564 would be D32D564
                dictkey = pair[0]+pair[1] if int(pair[0][3:]) < int(pair[1][3:]) else pair[1]+pair[0]
                if dictkey not in self.losmindist:
                    # Set the entry
                    self.losmindist[dictkey] = [losdistance, 
                                                bs.traf.lat[idx1], bs.traf.lon[idx1], bs.traf.alt[idx1], 
                                                bs.traf.lat[idx2], bs.traf.lon[idx2], bs.traf.alt[idx2],
                                                bs.sim.simt, bs.sim.simt]
                    # This guy here                             ^ is the LOS start time
                else:
                    # Entry exists, check if calculated is smaller
                    if self.losmindist[dictkey][0] > losdistance:
                        # It's smaller. Make sure to keep the LOS start time
                        self.losmindist[dictkey] = [losdistance, 
                                                bs.traf.lat[idx1], bs.traf.lon[idx1], bs.traf.alt[idx1], 
                                                bs.traf.lat[idx2], bs.traf.lon[idx2], bs.traf.alt[idx2],
                                                bs.sim.simt, self.losmindist[dictkey][8]]
        
        # Log data if there are aircraft that are no longer in LOS
        if lospairs_out:
            done_pairs = []
            for pair in set(lospairs_out):
                # Get their dictkey
                dictkey = pair[0]+pair[1] if int(pair[0][3:]) < int(pair[1][3:]) else pair[1]+pair[0]
                # Is this pair in the dictionary?
                if dictkey not in self.losmindist:
                    # Pair was already logged, continue
                    continue
                losdata = self.losmindist[dictkey]
                # Remove this aircraft pair from losmindist
                self.losmindist.pop(dictkey)
                #Log the LOS
                bs.traf.M22Logger.loslog.log(losdata[8], losdata[7], pair[0], pair[1],
                                losdata[1], losdata[2],losdata[3],
                                losdata[4], losdata[5],losdata[6],
                                losdata[0])
                
        
        self.prevlospairs = set(bs.traf.cd.lospairs)
            
    @stack.command
    def deleteall(self):
        '''Delete all aircraft.'''
        while self.ntraf>0:
            self.delete(0)
        return
    
    #@timed_function(name='cruisespd', dt = 0.5)
    def speed_control(self):
        '''Set the cruise speed of all aircraft.'''
        # First, some checks
        in_turn = np.logical_or(bs.traf.ap.inturn, bs.traf.ap.dist2turn < 50)  # Are aircraft in a turn?
        cr_active = bs.traf.cd.inconf # Are aircraft doing CR?
        in_vert_man = np.abs(bs.traf.vs) > 0 # Are aircraft performing a vertical maneuver?
        speed_zero = np.array(bs.traf.selspd) == 0 # The selected speed is 0, so we're at our destination and landing
        lnav_on = bs.traf.swlnav
        
        # Set the speed of all aircraft that meed the conditions to 30
        set_cruise_speed = np.logical_and.reduce((lnav_on,
                                                  np.logical_not(in_turn),
                                                  np.logical_not(cr_active),
                                                  np.logical_not(in_vert_man),
                                                  np.logical_not(speed_zero)))
        
        bs.traf.selspd = np.where(set_cruise_speed, bs.traf.actwp.cruisespd, bs.traf.selspd)