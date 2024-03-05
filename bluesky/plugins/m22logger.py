import bluesky as bs
from bluesky.core import Entity, timed_function
from bluesky.stack import command
from bluesky import stack
from bluesky.tools import datalog
from bluesky.tools.aero import ft

def init_plugin():
    # Configuration parameters
    config = {
        'plugin_name': 'M22LOGGER',
        'plugin_type': 'sim',
        'reset': reset
    }
    bs.traf.M22Logger = M22Logger()
    return config

def reset():
    bs.traf.M22Logger.reset()


class M22Logger(Entity):
    def __init__(self):
        super().__init__()
        # Create the loggers
        self.flst = datalog.crelog('FLSTLOG', None, flstheader)
        self.conflog = datalog.crelog('CONFLOG', None, confheader)
        self.reglog = datalog.crelog('REGLOG', None, regheader)
        self.loslog = datalog.crelog('LOSLOG', None, losheader)
        
    def reset(self):
        # Reset the loggers and all the vars
        self.flst = datalog.crelog('FLSTLOG', None, flstheader)
        self.conflog = datalog.crelog('CONFLOG', None, confheader)
        self.reglog = datalog.crelog('REGLOG', None, regheader)
        self.loslog = datalog.crelog('LOSLOG', None, losheader)
        
    @timed_function(name='reglog', dt=5)
    def thereglog(self):
        bs.traf.M22Logger.reglog.log(*bs.traf.id)
        bs.traf.M22Logger.reglog.log(*bs.traf.alt/ft)
        bs.traf.M22Logger.reglog.log(*bs.traf.lat)
        bs.traf.M22Logger.reglog.log(*bs.traf.lon)
        return
    
    @command
    def startlogs(self):
        bs.traf.M22Logger.flst.start()
        bs.traf.M22Logger.conflog.start()
        bs.traf.M22Logger.reglog.start()
        bs.traf.M22Logger.loslog.start()
        return

flstheader = \
    '#######################################################\n' + \
    'FLST LOG\n' + \
    'Flight Statistics\n' + \
    '#######################################################\n\n' + \
    'Parameters [Units]:\n' + \
    'Deletion Time [s], ' + \
    'Call sign [-], ' + \
    'Spawn Time [s], ' + \
    'Flight time [s], ' + \
    'Distance 2D [m], ' + \
    'Distance 3D [m], ' + \
    'Distance ALT [m],' + \
    'Latitude [deg], ' + \
    'Longitude [deg], ' + \
    'Altitude [ft], ' + \
    'TAS [kts], ' + \
    'Vertical Speed [fpm], ' + \
    'Heading [deg], ' + \
    'ASAS Active [bool], ' + \
    'Pilot ALT [ft], ' + \
    'Pilot SPD (TAS) [kts], ' + \
    'Pilot HDG [deg], ' + \
    'Pilot VS [fpm]\n'

confheader = \
    '#######################################################\n' + \
    'CONF LOG\n' + \
    'Conflict Statistics\n' + \
    '#######################################################\n\n' + \
    'Parameters [Units]:\n' + \
    'Simulation time [s], ' + \
    'ACID1 [-],' + \
    'ACID2 [-],' + \
    'LAT1 [deg],' + \
    'LON1 [deg],' + \
    'ALT1 [ft],' + \
    'LAT2 [deg],' + \
    'LON2 [deg],' + \
    'ALT2 [ft],' + \
    'CPALAT [lat],' + \
    'CPALON [lon]\n'
    
losheader = \
    '#######################################################\n' + \
    'LOS LOG\n' + \
    'LOS Statistics\n' + \
    '#######################################################\n\n' + \
    'Parameters [Units]:\n' + \
    'LOS exit time [s], ' + \
    'LOS start time [s],' + \
    'Time of min distance [s],' + \
    'ACID1 [-],' + \
    'ACID2 [-],' + \
    'LAT1 [deg],' + \
    'LON1 [deg],' + \
    'ALT1 [ft],' + \
    'LAT2 [deg],' + \
    'LON2 [deg],' + \
    'ALT2 [ft],' + \
    'DIST [m]\n'

regheader = \
    '#######################################################\n' + \
    'REGULAR LOG\n' + \
    'Statistics recorded regularly at a certain simtime interval.\n' + \
    '#######################################################\n\n' + \
    'Parameters [Units]:\n' + \
    'Simulation time [s], ' + \
    'ACIDs [-], ALTs [ft], LATs [deg], LONs [deg]\n'