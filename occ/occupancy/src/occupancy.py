#!/usr/bin/env python
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped, Pose
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from utils import *  # includes rospy and numpy
from robot import PARTICLE

# parameters
MAP_WIDTH = 53.9
MAP_HEIGHT = 10.6
STD_DEV_HIT = 1.0
LAMBDA_SHORT = 1.0
Z_HIT = 0.5
Z_RAND = 0.2
Z_MAX = 0.1
Z_SHORT = 0.1
MAX_DIST = 5.0
l_occ = .8
l_free = .2
alpha = 1
PARTICLE_COUNT = 1

# start of globals
last_time = None
actual_pose = None


class MAPPER:
    def __init__(self):
        global MAP_WIDTH, MAP_HEIGHT, STD_DEV_HIT, LAMBDA_SHORT, \
                Z_HIT, Z_RAND, Z_MAX, Z_SHORT, PARTICLE_COUNT, MAX_DIST, \
                l_occ, l_free, alpha, PARTICLE_COUNT
        rp.init_node('mapper')

        rp.get_param('~MAP_WIDTH', MAP_WIDTH)

        self.last_time = rp.get_rostime()
        self.part = PARTICLE()
        #self.parset = [PARTICLE() for _ in xrange(PARTICLE_COUNT)]
        self.pose = PoseStamped()
        rp.Subscriber('robot/base_scan', LaserScan, self.scan_callback, queue_size=1)

        MAP_WIDTH      = rp.get_param('~map_width',      53.9)
        MAP_HEIGHT     = rp.get_param('~map_height',     10.6)
        STD_DEV_HIT    = rp.get_param('~std_dev_hit',     1.0)
        LAMBDA_SHORT   = rp.get_param('~lambda_short',    1.0)
        Z_HIT          = rp.get_param('~z_hit',           0.5)
        Z_RAND         = rp.get_param('~z_rand',          0.2)
        Z_MAX          = rp.get_param('~z_max',           0.1)
        Z_SHORT        = rp.get_param('~z_short',         0.1)
        MAX_DIST       = rp.get_param('~max_dist',        5.0)
        l_occ          = rp.get_param('~l_occ',           .8)
        l_free         = rp.get_param('~l_free',          .2)
        alpha          = rp.get_param('~alpha',           1)
        PARTICLE_COUNT = rp.get_param('~particle_count',  1)

        # TODO should be odometry we subscribe to
        rp.Subscriber('/stage/odom', Odometry, self.odom_callback, queue_size=1)

        self.map_pub = rp.Publisher('/map', OccupancyGrid, latch=True, queue_size=1)
        self.map_data_pub = rp.Publisher('/map_metadata', MapMetaData, latch=True, queue_size=1)
        self.cmd_vel = rp.Publisher('/robot/cmd_vel', Twist, queue_size=1)

        # pass current pose from stage to rviz
        self.pose_pub = rp.Publisher('/robot/pose', PoseStamped, latch=True, queue_size=1)
        self.control = (1.0, 0.0)

        # dt = (rp.get_rostime() - last_time).to_sec()

    def publish_map(self):
        """ Publish the map. """
        #grid_msg = self.parset[0].map.to_message()
        grid_msg = self.part.map.to_message()
        self.map_data_pub.publish(grid_msg.info)
        self.map_pub.publish(grid_msg)

    def scan_callback(self, scan):
        """
        :param scan:
        :return: nothing
        """
        self.part.map = occupancy_grid_mapping(self.part.map, self.pose.pose, scan)
        # Now that the map is updated, publish it!
        rp.loginfo("Scan is processed, publishing updated map.")
        self.publish_map()

    def odom_callback(self, msg):
        """ update actual pose from msg from stage
        :param msg: frame odom, twist and pose
        :return: nothing
        """

        self.pose.header.frame_id = "/base_link"
        self.pose.header.stamp = rp.Time.now()
        #print "callback ", msg

        self.pose = msg
        #print "to send out", self._pose

if __name__ == '__main__':
    try:
        m = MAPPER()
        #print"particle set contains", m.get_parset()
        rp.spin()

    except rp.ROSInterruptException:
        pass
