#!/usr/bin/env python
import mcl_tools
import numpy as np
import rospy as rp
# parameters
MAP_WIDTH      = rp.get_param('/map_width')
MAP_HEIGHT     = rp.get_param('/map_height')
STD_DEV_HIT    = rp.get_param('/std_dev_hit')
LAMBDA_SHORT   = rp.get_param('/lambda_short')
Z_HIT          = rp.get_param('/z_hit')
Z_RAND         = rp.get_param('/z_rand')
Z_MAX          = rp.get_param('/z_max')
Z_SHORT        = rp.get_param('/z_short')
MAX_DIST       = rp.get_param('/max_dist')
PARTICLE_COUNT = rp.get_param('/particle_count')


class ROBOT:
    """
    Docstring
    """
    global MAP_WIDTH, MAP_HEIGHT, STD_DEV_HIT, LAMBDA_SHORT, Z_HIT, Z_RAND, Z_MAX, Z_SHORT, MAX_DIST,PARTICLE_COUNT
    # parameters


    def __init__(self):
        self.x = np.random.random() * MAP_WIDTH
        self.y = np.random.random() * MAP_HEIGHT
        self.theta = np.random.random() * 2.0 * np.pi
        self.weight = 0.1
        self.forward_noise = 0.01
        self.turn_noise = 0.01

    def reset(self):  # would like a better way to do
        x = np.random.random() * MAP_WIDTH
        y = np.random.random() * MAP_HEIGHT

        while mcl_tools.map_hit(x, y):  # outside of map?
            x = np.random.random() * MAP_WIDTH
            y = np.random.random() * MAP_HEIGHT

        self.x = x
        self.y = y

        self.theta = np.random.random() * 2.0 * np.pi
        self.weight = 0.1
        self.forward_noise = 0.01
        self.turn_noise = 0.01

    def set_pose(self, new_x, new_y, new_orientation):
        if new_x < 0 or new_x >= MAP_WIDTH:
            raise ValueError, 'X coordinate out of bound'
        if new_y < 0 or new_y >= MAP_HEIGHT:
            raise ValueError, 'Y coordinate out of bound'
        if new_orientation < 0 or new_orientation >= 2 * np.pi:
            raise ValueError, 'Orientation must be in [0..Tau]'
        self.x = float(new_x)
        self.y = float(new_y)
        self.theta = float(new_orientation)

    def set_x(self, new_x):
        if new_x < 0 or new_x >= MAP_WIDTH:
            raise ValueError('X coordinate out of bound')
        self.x = float(new_x)

    def set_y(self, new_y):
        if new_y < 0 or new_y >= MAP_HEIGHT:
            raise ValueError('Y coordinate out of bound')
        self.y = float(new_y)

    def set_theta(self, new_orientation):
        if new_orientation < 0 or new_orientation >= 2 * np.pi:
            raise ValueError, 'Orientation must be in [0..Tau]'
        self.theta = float(new_orientation)

    def set_weight(self, new_weight):
        self.weight = float(new_weight)

    def move(self, control):
        # print con
        # turn, and add randomness to the turning command
        forward, turn = control  # unpack linear/angular velocities
        theta = self.theta + float(turn) + np.random.normal(0.0, self.turn_noise)
        theta %= 2 * np.pi  # make sure between 0 and Tau

        dist = float(forward) + np.random.normal(0.0, self.forward_noise)
        x = self.x + (np.cos(theta) * dist)
        y = self.y + (np.sin(theta) * dist)
        x %= MAP_WIDTH  # cyclic truncate
        y %= MAP_HEIGHT  # make sure still inside world range

        self.set_pose(x, y, theta)  # adjust pose
        while mcl_tools.map_hit(self.x, self.y):  # movement now outside of map?
            #print "kill bad particle"
            self.reset()

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
        scan_count = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
        beams = np.arange(scan.angle_min, scan.angle_max + 0.01, scan.angle_increment)  # needed .01 to give first beam

        w_sum = 0
        for l in xrange(scan_count):
            #print np.degrees(beams[l]), ": stage ", scan.ranges[l], mcl_tools.map_range(self, beams[l])
            # had to add small value since map_range could return 0 and sigma cant be 0
            w_sum += mcl_tools.gauss_prob(scan.ranges[l], (mcl_tools.map_range(self, beams[l]))+.001)

        self.set_weight(w_sum)
        #print "particle internal weight sum ", w_sum

        return w_sum


    def measurement_prob1(self, scan):
        """
        Take laser scan data and compare it to particles position by use of
            raycaster. Compute weight by summing up the quality of readings from each scan
            
        :param scan:    laser scan msg
        :return:        weight ofsettings. particle based on correct pose
        """

        # take laser scan and compare it to raycaster for particles position
        # number of laser scans
        # scan_count = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
        # beams = np.arange(scan.angle_min, scan.angle_max + 0.01, scan.angle_increment)  # needed .01 to give first beam

        # from eric m
        # precomputes the coefficient and exponent base for faster normal calculations
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
            traced = mcl_tools.map_range(self, scan_min+(l*scan_inc))
            prob *= Z_HIT*p_hit(sensed, traced)\
                + Z_RAND*p_rand(sensed, traced)\
                + Z_MAX*p_max(sensed, traced)
            print "probability", prob
        self.set_weight(prob)
        # print "particle prob ", prob

        return prob

    def get_pose(self):
        """:returns x (float), y (float), orientation (radians) of particle."""
        return self.x, self.y, self.theta

    def get_x(self):
        """:returns x position of robot (float)."""
        return self.x

    def get_y(self):
        """:returns y position of robot (float)."""
        return self.y

    def get_theta(self):
        """:returns theta of robot (radians)."""
        return self.theta

    def get_weight(self):
        """:returns probability of correctness"""
        return self.weight

    def __repr__(self):
        return '[x=%.6s y=%.6s w=%.6s]' % (str(self.x), str(self.y), str(self.theta))
