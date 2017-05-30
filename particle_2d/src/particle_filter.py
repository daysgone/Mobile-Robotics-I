#!/usr/bin/env python
#import roslib;roslib.load_manifest('with_weights')
import rospy as rp
import numpy as np
import mcl_tools
from robot import ROBOT

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

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

rp.init_node('with_weights')

CONTROL = np.zeros(2, dtype=np.float)  # [0.0, 0.0]  # linear velocity, angular velocity
CMD_VEL = None  # publisher
COUNT = 0
W_SLOW = 0
W_FAST = 0

PRIOR_TIME = None
BEAMS = None
P = []


def actual_pose(msg):
    """ DEBUGGING ONLY
    ONly used to show actual pose in map not for particle filter
    :param msg:
    """
    #print msg.pose.pose
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    t = msg.pose.pose.orientation.z
    pose = (x+MAP_WIDTH/2, y+MAP_HEIGHT/2, t, 0)

    mcl_tools.show_real_pose(pose)


def particle_filter(ps, control, scan):
    """
    :param ps:      particle list
    :param control: tuple of linear and angular velocity
    :param scan:    laser scan msg
    :return:        updated list of particles
    """
    global W_SLOW, W_FAST, COUNT, PRIOR_TIME

    # need this to set initial time
    if PRIOR_TIME is None:
        PRIOR_TIME = rp.get_rostime()

    # <editor-fold desc="motion update">
    # adjust speed based on simulation
    time_dif = (rp.get_rostime() - PRIOR_TIME).to_sec()
    con = [time_dif * i for i in control]
    #con = np.multiply(time_dif, control) #numpy should be used for large data sets
    rp.loginfo("control" + str(con))

    #moving each particle based on adjusted velocity
    for p in xrange(PARTICLE_COUNT):
        ps[p].move(con)
    # </editor-fold>

    # <editor-fold desc="Weight update">
    weight_p = []
    w_avg = 0.0
    w_sum = 0.0
    max_w = (0, 0.0)  # id, weight

    for p in xrange(PARTICLE_COUNT):
        w_sum += ps[p].measurement_prob(scan)
        weight = ps[p].get_weight()  # individual weight
        weight_p.append(weight)
        w_avg += 1.0/PARTICLE_COUNT * weight

        if weight > max_w[1]:
            max_w = p, weight
    #print weight_p
    mcl_tools.show_best_pose(ps[max_w[0]].get_pose())
    # </editor-fold>

    # <editor-fold desc="random sampling with probability">

    sample_p = []

    # do actual resampling

    # 0 <= alpha_slow << alpha_fast
    alpha_slow = 0.001   # decay rate for long-term avg
    alpha_fast = 0.08    # decay rate for short-term avg

    W_SLOW += alpha_slow * (w_avg - W_SLOW)
    W_FAST += alpha_fast * (w_avg - W_FAST)
    #print 'long term avg: ', W_SLOW
    #print 'short term avg: ', W_FAST

    # if short-term likelihood is worse long-term, samples added in proportion to quotient
    # sudden decrease in likelihood increases number of random samples
    random_prob = max(0.0, 1.0 - W_FAST/W_SLOW)

    # create an array of random indexes (1 to particle count-1)
    rand_array = np.random.randint(low=1, high=PARTICLE_COUNT-1, size=int(PARTICLE_COUNT * random_prob))  # array of indexes to randomly move
    index = int(np.random.random() * PARTICLE_COUNT-len(rand_array))
    beta = 0.0
    for k in xrange(PARTICLE_COUNT-len(rand_array)):
        #print "gets here: ", k
        # from thrun's udacity class
        beta += np.random.random() * 2.0 * (max_w[1]/w_sum)
        #print beta, ps[index].get_weight()
        count = 0
        while beta > ps[index].get_weight():
            count += 1
            #print "stuck here?", count
            beta -= ps[index].get_weight()
            index = (index + 1) % PARTICLE_COUNT
        sample_p.append(ps[index])

    #print "sample_p len ", len(sample_p) #weight
    # create random particles

    while len(sample_p) < PARTICLE_COUNT:
        r = ROBOT()
        r.set_weight(w_avg)  # give it a weight so it shows up
        COUNT += 1
        sample_p.append(r)

    #print '\ncreated ', COUNT, ' new particles'
    #print 'total num particles:', len(sample_p)

    ps = sample_p

    # </editor-fold>
    weight_p = np.multiply(weight_p, 1.0/np.sum(weight_p))  # normalize the weights

    # reset data
    COUNT = 0
    PRIOR_TIME = rp.get_rostime()

    return mcl_tools.random_sample(ps, PARTICLE_COUNT, weight_p)


def got_vel(msg):

    global CONTROL
    #print msg
    CONTROL = msg.linear.x, msg.angular.z


def got_scan(msg):
    """
    This function updates on each message received from laser scan
    Command robot in simulation space and pass information to particle filter
    :param msg:     contains message of laser scan data
    """
    global P, CMD_VEL, CONTROL, PRIOR_TIME, BEAMS

    # control simulated robot movement
    if BEAMS is None:  # statically use it after initialization (message shouldn't change length while it's running)
        BEAMS = np.multiply(range(len(msg.ranges)), msg.angle_increment) + msg.angle_min
    #print rp.get_rostime()
    P = particle_filter(P, CONTROL, msg)
    #print P
    mcl_tools.show_particles(P)


def main():
    global P
    global CMD_VEL
    # Uncomment for debugging if nesssary, recomment before turning in.
    # rp.Subscriber('/stage/base_pose_ground_truth', Odometry, mcl_debug.got_odom)
    rp.Subscriber('/robot/base_scan', LaserScan, got_scan)  # movement/sensor particle update
    rp.Subscriber('/robot/cmd_vel', Twist, got_vel)
    #actual robot pose only used to show it on map not for partcile filter
    rp.Subscriber('/stage/base_pose_ground_truth', Odometry, actual_pose)  #actual robot pose

    #removed due to wander node
    #CMD_VEL = rp.Publisher('/robot/cmd_vel', Twist, queue_size=1)

    # create initial particles
    for p in xrange(PARTICLE_COUNT):
        r = ROBOT()
        r.set_pose(5, 5.6, 0)  # map space
        P.append(r)

    mcl_tools.mcl_init('particle_2d')
    mcl_tools.mcl_run_viz()


if __name__ == '__main__':
    try:
        main()
        print "error"

    except rp.ROSInterruptException:
        pass
