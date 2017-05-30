import numpy as np
import rospy as rp
from numpy import inf
from pprint import pprint
import tf

l_occ = .8
l_free = .2
angles = None
last_time = None


def fullprint(*args, **kwargs):
    opt = np.get_printoptions()
    np.set_printoptions(threshold=inf)
    pprint(*args, **kwargs)
    np.set_printoptions(**opt)


def logit(p):
    return np.log(p/(1-p))


def unlogit(l):
    return 1-(1/(np.exp(l)+1))


def dist2pnt(pnt1, pnt2):
    """ Take 2 points are return the distance between them
    :param pnt1: point to measure (x,y,theta)
    :param pnt2: point to measure to (x,y,theta)
    :return: distance
    """

    return np.sqrt((pnt1[0] - pnt2[0])**2 + (pnt1[1] - pnt2[1])**2)


def in_perceptual_field(center, point, scan):
    """
    :param center:  point of reference (x,y,theta)
    :param point:   point to check if in FOV(x,y,theta)
    :param scan:    laserscan msg
    :return: (bool) if point in the perceptual field of the robot
    """
    r = scan.range_max
    angle_min = scan.angle_min
    angle_max = scan.angle_max

    #print np.rad2deg(angle_max)

    #TODO not sure if this is still needed
    if angle_max > np.pi:
        new_max = angle_max % np.pi
        angle_max = -np.pi + new_max
    #print "laser starts at: ", np.rad2deg(angle_min), "laser ends at:", np.rad2deg(angle_max)

    d = dist2pnt(center, point)

    if d < r:  # within circle of radius range_max of scan
        #print "within circle"
        theta = np.arctan2(point[1] - center[1], point[0] - center[0]) - center[2]
        #fix if theta goes past -180 degrees  since ROS goes CCW 0 -> 179.9999 -> -179.999 -> 0
        if theta <= -np.pi:
            theta += 2*np.pi
        #print np.rad2deg(theta)
        #print angle_min, theta, angle_max
        if angle_min < theta < angle_max:
            #print "within fov"
            #print "grid space:", point[0], point[1], " is ", d, " from ", center[0], center[1]
            return True
    return False


def inverse_range_sensor_model(cell, pose, scan):
    """
    brief description

    :param cell:        (x,y,value) of cell to update
    :param pose:        current pose (x,y,t)
    :param scan:        sensor scan
    ..  :var angles:    global list of angles of sensor sweep
    ..  :var alpha:     thickness of obstacles
    ..  :var beta:      width of sensor beam
    :return:            value to change occupancy value by

    ..  note::
    """
    global angles

    # <editor-fold desc="book way of doing it">
    alpha = .2  # TODO find out what value
    laser_sweep = np.abs(scan.angle_max - scan.angle_min)  # sensor sweep
    theta = pose[2]  # robots orientation
    #l_0 = cell[2]  # prior log odds value

    #print "robot pose", pose
    #print "checking grid cell ", cell

    # angle between vector(1,0) from robots position and point position
    phi = np.arctan2(cell[1] - pose[1], cell[0] - pose[0]) - theta
    #print "angle between points", np.rad2deg(phi)
    r = dist2pnt(cell, pose)
    #print np.rad2deg(angles)
    # calculate the difference of the angle to each beam
    angle_dif = np.abs(np.subtract(angles, phi))
    #print np.rad2deg(angle_dif)
    k = np.argmin(angle_dif)  # find index of closest beam to point
    #print "index to closest beam", k
    #print scan.ranges[k]
    #print min(scan.range_max, scan.ranges[k] + alpha/2)

    # return the cell's value based on the scan data
    if r > min(scan.range_max, scan.ranges[k] + alpha/2) or angle_dif[k] > laser_sweep/2:
        return cell[2]  # return previous value l_0
    if scan.ranges[k] < scan.range_max and np.abs(r - scan.ranges[k]) < alpha/2:
        #print l_occ
        return logit(l_occ)  # occupied area
    if r <= scan.ranges[k]:
        return logit(l_free)
    print("should not get here")
    return -1
    # </editor-fold>


def occupancy_grid_mapping(map_prior, pose, scan):
    """ mapping with known poses

    assumptions:
        cells are independent of each other
        world is static
        no controls
        only observations
        binary values

    loop through each cell in map and update value data based on inverse sensor model
    :param map_prior: 2D array of prior occupancy values
                location within map corresponds to x,y location in real world
    :param pose:  current known pose, ex. position.x
    :param scan:  current sensor reading
    :return:    updated occupancy map
    """
    global angles
    #print "ros pose\n", pose

    # pose comes in as a quaternion need to change to XYZ
    quat = (
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quat)
    actual_pose = pose.pose.position.x+abs(map_prior.origin_x), pose.pose.position.y+abs(map_prior.origin_y), euler[2]  # yaw

    # dont want to recalculate this for every cell
    if angles is None:
        angles = np.multiply(range(len(scan.ranges)), scan.angle_increment) \
            + scan.angle_min

    # for all cells in map
    #print map_prior.grid
    count = 0

    for (h, w), value in np.ndenumerate(map_prior.grid):
        # need to scale h,w grid cells back to real world measurements
        #print "height:", h*map_prior.resolution, "width:",  w*map_prior.resolution
        if in_perceptual_field(actual_pose, (w*map_prior.resolution, h*map_prior.resolution, 0), scan):


            l_0 = value
            #print l_0

            '''
            if map_prior.grid[h][w] <= 0:
                l_0 = 0.1
            else:
                #print "prior value", map_prior.grid[h][w]
                l_0 = logit(map_prior.grid[h][w])
                print map_prior.grid[h][w], ":", l_0
            '''
            #print "prior", value
            value += inverse_range_sensor_model((w*map_prior.resolution,
                                                h*map_prior.resolution,
                                               value), actual_pose, scan) - l_0
            #print "post update", value
            '''
            #if map_prior.grid[h][w] >= 100:
                #print "value too large"
                #map_prior.grid[h][w] = 100

            if map_prior.grid[h][w] < -1:
                print "value too small", value, map_prior.grid[h][w]
                map_prior.grid[h][w] = -1
            '''
            map_prior.grid[h][w] = value
            #if map_prior.grid[h][w] > 100 or map_prior.grid[h][w] < 0:
                #print h, w, value, map_prior.grid[h][w]
    # try and find center of occupancy
    #center_h = map_prior.height/2
    #center_w = map_prior.width/2
    #map_prior.grid[center_h, center_w] = 100
    print"cells not correct", count
    map_update = map_prior
    #fullprint(map_prior.grid)  # if you want to see the raw matrix values
    return map_update



def sample_motion_model(odo, prior_pose):
    global last_time
    '''if last_time is None:
        last_time = rp.get_rostime()

    # probabilistically move all the particles
    rp.loginfo((rp.get_rostime() - last_time).to_sec())
    new_pos = partial(integrate_control_to_distance, control, (rp.get_rostime() - last_time).to_sec())
    last_time = rp.get_rostime()
    cur_pose = new_pos(part)
    '''
    return 0, 0, 1


def measurement_model_map(map_prior, cur_pose, scan):
    weight = 1.0
    return weight


def scan_match(pose, map, scan):
    """
    #maximize the likelihood of the current poase and map relative to the previous pose and map
    #xtstar = argmax{p(zt|xt,mt-1)p(xt\ut-1,xtstar-1)}
    xtstar-1 initial guess
    #zt current measurement
    #mt-1 map constructed so far
    #ut-1 robot motion odom
    #xt-1 previous pose
    """
    #scan match in blocks blocks of 100?
    #particle filter to estimate between chunks
    proposed_pose = prev_pose

    xtstar = 0.0
    return xtstar


