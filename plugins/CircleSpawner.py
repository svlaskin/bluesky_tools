""" A plugin to spawn traffic through a circular area.
The inputs are the desired traffic numbers within the circle, the circle position (center, in lat/lon) and the circle radius (in meters), as well as the aircraft type.
The destination waypoints can be set, but those (for good practice) should be far outside the experiment area for more 
The headings are randomized, as are the spawn locations along the borders of the spawn zone.

 """
import numpy as np

# Import the global bluesky objects. Uncomment the ones you need
from bluesky import traf, sim  # , settings, navdb, traf, sim, scr, tools
from bluesky.tools import datalog, areafilter
from bluesky.core import Entity, timed_function
from bluesky.tools.aero import ft, kts, nm, fpm
from bluesky.tools.geo import *
from bluesky import stack
from bluesky import core
from bluesky import traffic
from bluesky import simulation
import time
import datetime
from bluesky.tools.misc import degto180

def unix_to_datetime(unix_timestamp):
    return datetime.datetime.fromtimestamp(int(unix_timestamp))

def init_plugin():

    # Addtional initilisation code
    cityroute = CircleSpawner()

    # Configuration parameters
    config = {
        "plugin_name": "CIRCLESPAWNER",
        # This is a sim plugin
        "plugin_type": "sim",
    }

    return config


class CircleSpawner(Entity):
    def __init__(self):
        self.spawn_points2 = []
        height = 1000
        speed = 105
        heading = 100
        self.radius = 0
        self.latc = 0
        self.lonc = 0
        self.density = 0
        self.mdens = 0  # measured density for center measurement area circle
        self.lat_cc, self.lon_cc = 0, 0  # empty latlon for drones in central area
        self.standardtime = time.time()
        # headerr = (
        #     "############################################################\n"
        #     + "Inst. Measured Density\n"
        #     + "Drone lat\n"
        #     + "Drone lon\n"
        #     + "ACID"
        # )
        headerr = "density,latitude,longitude,id,time"
        self.loggedstuff = datalog.crelog("CircleSpawnLog", None, headerr)
        super().__init__()

    @stack.command
    # Initialisation function - define circle position and area, as well as the desired density.
    def spawncircle(
        self,
        radius: float,
        circle_center_lat: float,
        circle_center_lon: float,
        density: float,
        heading_min: float,
        heading_max: float,
    ):
        acid_gen = "EXP" + str(
            int(np.round(np.random.rand(1) * 10000))
        )  # random acid generate
        self.spawn_points2 = np.array(
            [
                qdrpos(
                    float(circle_center_lat),
                    float(circle_center_lon),
                    float(theta),
                    float(radius / nm),
                )
                for theta in np.arange(heading_min, heading_max, 1)
            ]
        )
        spawn_point = self.spawn_points2[
            np.random.randint(0, np.size(self.spawn_points2) / 2)
        ]
        height = 1000
        speed = 105
        heading = 100
        self.radius = radius
        self.latc = circle_center_lat
        self.lonc = circle_center_lon
        self.density = density
        self.mdens = 0  # measured density for center measurement area circle
        self.lat_cc, self.lon_cc = 0, 0  # empty latlon for drones in central area
        # Logging start
        self.loggedstuff.start()
        # use area plugin to automate deletion after pass of large spawn/delete boundary
        stack.stack(
            f"CIRCLE a1 {self.latc} {self.lonc} {self.radius/nm}"
        )  # Large external area where aircraft spawn and are deleted
        stack.stack(
            f"CIRCLE a2 {self.latc} {self.lonc} {0.45*self.radius/nm}"
        )  # smaller internal measurement area
        stack.stack(f"PAN {self.latc} {self.lonc}")
        stack.stack(f"ZOOM {np.sqrt(radius/100)}")
        stack.stack(f"ASAS ON")
        stack.stack("AREA a2")
        stack.stack("op")
        return

    @stack.command
    # Function to spawn drone/AC periodically - feeds into the timed function that follows and requires no inputs.
    def spawnsimple(self):
        randint = np.random.randint(0, np.size(self.spawn_points2) / 2)
        spawn_point = self.spawn_points2[randint]
        acid_gen = "D" + str(
            int(np.round(np.random.rand(1) * 100000))
        )  # random acid generate
        # heading_randcomp = 0 # to be edited!
        heading = (
            np.arange(0, 360, 3)[randint] - 180.0
        )  # set heading into the center of the circle, and add distribution component!
        height = 1000 * np.random.rand(1)[0]
        speed = (
            150.0 * np.random.rand(1)[0]
        )  # same distribution considerations needed for the speed and height! [TODO!!]
        stack.stack(
            f"CRE {acid_gen}, M600, {spawn_point[0]} {spawn_point[1]} {heading} {height} {speed}"
        )
        dest_center = kwikpos(
            self.latc, self.lonc, heading, 2.5 * self.radius / nm
        )  # basis point for random destination select
        lat_dest, lon_dest = kwikpos(
            dest_center[0],
            dest_center[1],
            heading + 90 * (np.random.rand(1) * 2 - 1),
            0.2 * self.radius * np.random.rand(1) / nm,
        )
        stack.stack(f"{acid_gen} dest {float(lat_dest)},{float(lon_dest)}")
        return

    @core.timed_function(name="cdenskeep", dt=0.5)
    # spawn aircraft continuously to keep density at prescribed value
    def genac(self):
        if np.size(traf.id) < self.density:
            stack.stack(f"SPAWNSIMPLE")
        return

    @timed_function(dt = 0.5)
    def delete_aircraft(self):
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
                stack.stack(f'DEL {acid}')

    @core.timed_function(name="statcomp", dt=1.0)
    def statcomp(self):
        dist2center = kwikdist_matrix(
            traf.lat,
            traf.lon,
            np.full(np.shape(traf.lat), self.latc),
            np.full(np.shape(traf.lon), self.lonc),
        )  # compute distance of all aircraft to the circle center and check whether within radius
        ind2keep = (
            dist2center <= self.radius*0.45/nm
        )  # index array for drones in center circle, which is 0.45 in radius compred to spawn zone.
        self.mdens = np.shape(dist2center[ind2keep])[
            0
        ]  # just length of center dist vec where drones are in measurement circle
        self.lat_cc, self.lon_cc = (
            traf.lat[ind2keep],
            traf.lon[ind2keep],
        )  # same for lat/lon
        acid_in_cc = np.array(traf.id)[
            ind2keep
        ]  # again, same logic, just keep the indices within radius of innter circle
        self.loggedstuff.log(
            self.mdens,
            self.lat_cc,
            self.lon_cc,
            acid_in_cc,
            unix_to_datetime(sim.simt+self.standardtime),
        )

        return
