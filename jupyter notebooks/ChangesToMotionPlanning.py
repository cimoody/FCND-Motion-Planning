    def global_to_local(self):
        (east_home, north_home, _, _) = utm.from_latlon(self.global_home[1], self.global_home[0])
        (east, north, _, _) = utm.from_latlon(self.global_position[1], self.global_position[0])
        local_position = numpy.array([north - north_home, east - east_home, -(self.global_position[2] - self.global_home[2])])
        return local_position

    def local_to_global(self):
        (east_home, north_home, zone_number, zone_letter) = utm.from_latlon(
                                                        self.global_home[1], self.global_home[0])
        (lat, lon) = utm.to_latlon(east_home + self.local_position[1],
                               north_home + self.local_position[0], zone_number,
                               zone_letter)
        global_position = numpy.array([lon, lat, -(self.local_position[2]-self.global_home[2])])
        return global_position

    def point(self, p):
        return np.array([p[0], p[1], 1.]).reshape(1, -1)

    def collinearity_check(self,p1, p2, p3, epsilon=1e-6):   
        m = np.concatenate((p1, p2, p3), 0)
        det = np.linalg.det(m)
        return abs(det) < epsilon

    def prune_path(self, path):
        pruned_path = [p for p in path]
        # TODO: prune the path!
        i = 0
        while i < (len(pruned_path) - 2):
            p1 = self.point(pruned_path[i])
            p2 = self.point(pruned_path[i+1])
            p3 = self.point(pruned_path[i+2])
            if self.collinearity_check(p1, p2, p3):
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
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        data = np.loadtxt(self.filename, delimiter=',', dtype='Float64', skiprows=2)
        lat0 = 0. 
        lon0 = 0.
        lat0, lon0, _, _, _, _ = data[0, :] # north, east, alt, d_north, d_east, d_alt

        # TODO: set home position to (lon0, lat0, 0)
        self.home = np.array([lon0, lat0, 0])

        # TODO: retrieve current global position
        current_global_postion = self.global_position
 
        # TODO: convert to current local position using global_to_local()
        current_local_position = self.global_to_local
        
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt(self.filename, delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        # grid_start = (-north_offset, -east_offset)
        grid_start = (-north_offset, -east_offset)
        # TODO: convert start position to current position rather than map center
        self.start = current_global_postion
        
        # Set goal as some arbitrary position on the grid
        grid_goal = (-north_offset + 10, -east_offset + 10)
        # TODO: adapt to set goal as latitude / longitude position and convert

        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        # TODO: prune path to minimize number of waypoints
        path = self.prune_path(path)