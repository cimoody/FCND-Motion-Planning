import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import visdom
import utm

# from planning_utils_start import a_star, heuristic, create_grid
from planning_utils import a_star, heuristic, create_grid
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local
import pandas as pd


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        # Setting up drone and things that may change
        self.filename = 'colliders.csv'
        self.TARGET_ALTITUDE = 5
        self.SAFETY_DISTANCE = 5
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.local_home = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}
        self.deadband = 5.0 # m around each waypoint

        # Code for live plotting
        # from
        # https://udacity.github.io/udacidrone/docs/visdom-tutorial.html
        # default opens up to http://localhost:8097
        # Must have python window running >python -m visdom.server
        self.v = visdom.Visdom()
        assert self.v.check_connection()

        # Plot NE
        ne = np.array([self.local_position[0], self.local_position[1]]).reshape(1, -1)
        self.ne_plot = self.v.scatter(ne, opts=dict(
            title="Local position (north, east)", 
            xlabel='North', 
            ylabel='East'
        ))

        # Plot D
        d = np.array([self.local_position[2]])
        self.t = 0
        self.d_plot = self.v.line(d, X=np.array([self.t]), opts=dict(
            title="Altitude (meters)", 
            xlabel='Timestep', 
            ylabel='Down'
        ))

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def update_ne_plot(self):
        """ 
        Code for plotting position from
        https://udacity.github.io/udacidrone/docs/visdom-tutorial.html
        """
        ne = np.array([self.local_position[0], self.local_position[1]]).reshape(1, -1)
        self.v.scatter(ne, win=self.ne_plot, update='append')

    def update_d_plot(self):
        """
        Code for plotting altitude from
        https://udacity.github.io/udacidrone/docs/visdom-tutorial.html
        """
        d = np.array([self.local_position[2]])
        # update timestep
        self.t += 1
        self.v.line(d, X=np.array([self.t]), win=self.d_plot, update='append')


    def local_position_callback(self):
        """
        Has drone pass through phases based on position
        """
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        """
        Has drone land at last phases.
        """
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        """
        Transitions between the different states.
        """
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        """
        Initiates the arming transition.
        """
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        """
        Initiates the takeoff transition.
        """
        print("takeoff transition")
        self.target_position[2] = self.TARGET_ALTITUDE
        print(" target: ", self.target_position)
        self.takeoff(self.target_position[2])
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """
        Takes the top waypoint and sets it as the target position.
        Then commands the drone to fly to the target position.
        """
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        """
        Initiates the landing transition.
        """
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        """
        Initiates the disarming transition and releases control.
        """
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        """
        Stops the drone.
        """
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        """
        Sends waypoints from self.waypoints to file, 
         - read through
        https://github.com/msgpack/msgpack-python README
        to understand msgpack
        """
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    # def global_to_local(self):
    #     (east_home, north_home, _, _) = utm.from_latlon(self.global_home[1], self.global_home[0])
    #     (east, north, _, _) = utm.from_latlon(self.global_position[1], self.global_position[0])
    #     local_position = np.array([north - north_home, east - east_home, -(self.global_position[2] - self.global_home[2])])
    #     return local_position

    # def local_to_global(self, east, north):
    #     (east_home, north_home, zone_number, zone_letter) = utm.from_latlon(
    #                                                     self.global_home[1], self.global_home[0])
    #     (lat, lon) = utm.to_latlon(east + east_home, north + north_home, zone_number, zone_letter)
    #     global_position = np.array([lon, lat, -(self.local_home[2] - self.global_home[2])])
    #     return global_position

    # def point(self, p):
    #     return np.array([p[0], p[1], 1.]).reshape(1, -1)
    #
    # def collinearity_check(self, p1, p2, p3, epsilon=1e-2):
    #     collinear = False
    #     pp1 = np.array([p1[0], p1[1], self.TARGET_ALTITUDE]).reshape(1, -1)
    #     pp2 = np.array([p2[0], p2[1], self.TARGET_ALTITUDE]).reshape(1, -1)
    #     pp3 = np.array([p3[0], p3[1], self.TARGET_ALTITUDE]).reshape(1, -1)
    #     mat = np.vstack((pp1, pp2, pp3 ))
    #     # mat = np.vstack((self.point(p1), self.point(p2), self.point(p3)))
    #     det = np.linalg.det(mat)
    #     if np.abs(det) < epsilon:
    #         collinear = True
    #     return collinear

    def prune_path(self, path, err=.1):
        pruned_path = [p for p in path] 
        i = 0 
        while i < (len(pruned_path) - 2):
            p1 = np.array([pruned_path[i  ][0], pruned_path[i  ][1], self.TARGET_ALTITUDE]).reshape(1, -1)
            p2 = np.array([pruned_path[i+1][0], pruned_path[i+1][1], self.TARGET_ALTITUDE]).reshape(1, -1)
            p3 = np.array([pruned_path[i+2][0], pruned_path[i+2][1], self.TARGET_ALTITUDE]).reshape(1, -1)
            collinear = False
            mat = np.vstack((p1, p2, p3))
            det = np.linalg.det(mat)
            if np.abs(det) < err:
                collinear = True
            if collinear:
                pruned_path.remove(pruned_path[i+1])
            else: 
                i += 1
        return pruned_path


    def plan_path(self):
        """
        Planning the path for the drone to take.
        """
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = self.TARGET_ALTITUDE
        SAFETY_DISTANCE = self.SAFETY_DISTANCE

        self.target_position[2] = TARGET_ALTITUDE

        # # TODO: read lat0, lon0 from colliders into floating point values
        # q = []
        f = open(self.filename, 'r')
        q = f.readline()
        f.close()
        q = q.split(' ')
        lat0 = np.float(q[1][:-1])
        lon0 = np.float(q[3])

        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)

        # TODO: retrieve current global position
        current_global_position = self.global_position

        # TODO: convert to current local position using global_to_local()
        current_local_position = global_to_local(self.global_position, self.global_home)

        print('global home {0}, position {1}, local position {2}'.format(
                self.global_home,
                self.global_position,
                self.local_position))
        print('global home {0}, position {1}, local position {2}'.format(
            self.global_home,
            current_global_position,
            current_local_position))

        # Read in obstacle map
        data = np.loadtxt(self.filename, delimiter=',', dtype='Float64', skiprows=2)

        # ## Starter code
        # # Define a grid for a particular altitude and safety margin around obstacles
        # grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        # print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # # Define starting point on the grid (this is just grid center)
        # grid_start = (-north_offset, -east_offset)
        # # TODO: convert start position to current position rather than map center
        #
        # # Set goal as some arbitrary position on the grid
        # grid_goal = (-north_offset + 10, -east_offset + 10)
        # # TODO: adapt to set goal as latitude / longitude position and convert
        #
        # # Run A* to find a path from start to goal
        # # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # # or move to a different search space such as a graph (not done here)
        # print('Local Start and Goal: ', grid_start, grid_goal)
        # path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        # print("Path: ", path)
        # # TODO: prune path to minimize number of waypoints
        # # TODO (if you're feeling ambitious): Try a different approach altogether!
        #
        # # Convert path to waypoints
        # waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        # # Set self.waypoints
        # self.waypoints = waypoints
        # # TODO: send waypoints to sim (this is just for visualization of waypoints)
        # self.send_waypoints()
        # ## Starter code end


        ## My code
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        # grid_start = (-north_offset, -east_offset)

        # TODO: convert start position to current position rather than map center
        drone_start_global =  (self._longitude, self._latitude, 0.0)
        self.set_home_position(drone_start_global[0], drone_start_global[1], drone_start_global[2])
        drone_start_local = (global_to_local(drone_start_global, self.global_home))
        drone_start_local = (int(drone_start_local[0]) - north_offset, int(drone_start_local[1]) - east_offset)
        grid_start = drone_start_local
        print("Drone Start", drone_start_global,"\n", drone_start_local)
        print("drone_start_global", drone_start_global)
        print("drone_start_local", drone_start_local)

        # Set goal as some arbitrary position on the grid
        Lat = 37.79096 # picked locatiion by flying around #
        Lon = -122.40012 # picked location by flying around #
        Alt = -self._altitude
        print("Alt: ", Alt, " , Lat: ", Lat, " , Lon: ", Lon)
        drone_end_global = (Lon, Lat, 0.)
        drone_end_local = global_to_local(drone_end_global, self.global_home)
        grid_goal = (-north_offset + int(drone_end_local[0]), -east_offset + int(drone_end_local[1]))

        print("drone_end_local: ", drone_end_local, grid_goal)

        # TODO: adapt to set goal as latitude / longitude position and convert

        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        print("Path: ", path)
        # TODO: prune path to minimize number of waypoints
        # TODO (if you're feeling ambitious): Try a different approach altogether!
        pruned_path = self.prune_path(path, 100.)
        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in pruned_path]
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()
        ## My code ends


    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
