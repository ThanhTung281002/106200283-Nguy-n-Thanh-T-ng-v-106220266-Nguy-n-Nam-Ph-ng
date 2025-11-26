# drone/planner/search_pattern.py

class SearchPattern:
    """
    Generate search patterns for autonomous rescue drones.
    Currently supports:
    - lawnmower (zig-zag search)
    """

    def lawnmower(self, 
                  x_min: float, x_max: float,
                  y_min: float, y_max: float,
                  strip_width: float = 5.0,
                  z: float = -3.0):
        """
        Create a list of NED waypoints to cover a rectangular area.
        
        x_min, x_max: north axis bounds  (NED north; usually negative is forward)
        y_min, y_max: east axis bounds
        strip_width: distance between each zig-zag lane
        z: altitude (NED: negative value means height above ground)

        Returns:
            list of waypoints (tuples): [(x, y, z), (x, y, z), ...]
        """
        waypoints = []

        # Start from x_min and sweep toward x_max
        x = x_min
        flip = False  # controls up-to-down or down-to-up movement

        while x <= x_max:
            if not flip:
                # go bottom → top
                waypoints.append((x, y_min, z))
                waypoints.append((x, y_max, z))
            else:
                # go top → bottom
                waypoints.append((x, y_max, z))
                waypoints.append((x, y_min, z))

            flip = not flip
            x += strip_width

        return waypoints
