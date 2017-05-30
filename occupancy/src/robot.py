import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid
from utils import *

import settings


class PARTICLE:
    """
    Docstring
    """
    def __init__(self):
        self.x = np.random.random() * settings.MAP_WIDTH
        self.y = np.random.random() * settings.MAP_HEIGHT
        self.theta = np.random.random() * 2.0 * np.pi
        self.weight = 1.0
        self.pose = self.x, self.y, self.theta
        # map needs size information to be used correctly
        self.map = MAP()

    def move(self, con):

        #self.set_pose(x, y, theta)  # adjust pose

        return self  # new object with updated values

    def measurement_prob(self, scan):
        """
        Take laser scan data and compare it to particles position by use of
            raycaster. Compute weight by summing up the quality of readings from each scan
            
        :param scan:    laser scan msg
        :return:        weight of particle based on correct pose
        """

        # take laser scan and compare it to raycaster for particles position
        # number of laser scans
        # scan_count = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
        # beams = np.arange(scan.angle_min, scan.angle_max + 0.01, scan.angle_increment)  # needed .01 to give first beam

        # from eric m
        # precomputes the coefficient and exponent base for faster normal calculations
        STD_DEV_HIT = settings.STD_DEV_HIT
        p_hit_coeff = 1./(np.sqrt(2*np.pi*STD_DEV_HIT*STD_DEV_HIT))
        p_hit_exp   = np.exp(-1./(2.*STD_DEV_HIT*STD_DEV_HIT))

        def p_hit(sensed_distance, raytraced_distance):
            if sensed_distance > MAX_DIST:
                return 0.0
            return p_hit_coeff*(p_hit_exp**((sensed_distance - raytraced_distance)**2))

        def p_max(sensed_distance, raytraced_distance):
            if sensed_distance == MAX_DIST:
                return 1.0
            return 0.0

        def p_rand(sensed_distance, raytraced_distance):
            if sensed_distance < MAX_DIST:
                return 0.0
            return 1./MAX_DIST

        scan_min = scan.angle_min
        scan_inc = scan.angle_increment

        prob = 1.0
        for l in xrange(len(scan.ranges)):
            sensed = scan.ranges[l]
            traced = mcl_tools.map_range(self, scan_min + (l * scan_inc))
            prob *= settings.Z_HIT * p_hit(sensed, traced) \
                    + settings.Z_RAND * p_rand(sensed, traced) \
                    + settings.Z_MAX * p_max(sensed, traced)

        self.set_weight(prob)
        #print "particle prob ", prob

        return prob


    def __repr__(self):
        return '[x=%.6s y=%.6s theta=%.6s]' % (str(self.x), str(self.y), str(self.theta))


class MAP:
    """
    The Map class stores an occupancy grid as a two dimensional
    numpy array.

    """
    # need to change defaults if not using hallway map from stage 53.900 10.600
    def __init__(self, origin_x=-26.95, origin_y=-5.0, resolution=0.1, width=53.3, height=10.9):
        """
        :param origin_x:    Position of the grid cell (0,0)

        :param origin_y:    in the map coordinate system.
        :param resolution:  Width of each grid square in meters.
        :param width:       meter
        :param height:
        :return:
        """
        self.origin_x = origin_x  # global cord
        self.origin_y = origin_y
        self.resolution = resolution
        self.width = width/self.resolution  # convert to # of cells
        self.height = height/self.resolution  # convert to # of cells
        self.grid = np.ones((self.height, self.width)) * logit(.5)  # np.zeros((self.height, self.width))

    def to_message(self):
        """ Return a nav_msgs/OccupancyGrid representation of this map. """
        grid_msg = OccupancyGrid()

        # Set up the header.
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "/map"

        # .info is a nav_msgs/MapMetaData message.
        # TODO WHY DO I HAVE TO CAST AS INT MOFO
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height

        # Rotated maps are not supported... quaternion represents no
        # rotation.
        grid_msg.info.origin = Pose(Point(self.origin_x, self.origin_y, 0),
                               Quaternion(0, 0, 0, 1))

        # need to go from lod_odds to 0-100
        #for (h, w), value in np.ndenumerate(self.grid):

            #print value, unlogit(value)
            #self.grid[h][w] = int(100*unlogit(value))
        # Flatten the numpy array
        flat_grid = self.grid.reshape((self.grid.size,))
        #[(i+1)*50 for i in flat_grid]
        flat_grid = [unlogit(i)*100 for i in flat_grid]

        grid_msg.data = list(np.round(flat_grid))  # converts to int
        return grid_msg

