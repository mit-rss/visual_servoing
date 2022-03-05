#! /usr/bin/env python

"""
@author: jared

Generic pure pursuit control law for bicycle dynamics model

## Note - does radius of curvature calculation still work for q =/ 0?
"""
import numpy as np

def purepursuit(ld, L, vel_des, x, y, q, waypoints):
    """
    Given lookahead distance, robot length, desired velocity, current pose, and path waypoints (2d np array), 
    returns an instantaneous x linear velocity and steering angle eta to follow trajectory
    """
    goal = look_ahead(ld, x, y, waypoints) # Instantaneous goal - lookahead point
    R = (ld*ld)/(2*goal[1]) # Radius of curvature connecting these points

    # Control law !!
    eta = np.arctan(L / R)
    lin_vel = vel_des

    return eta, lin_vel

def look_ahead(ld, x, y, waypoints):
    """
    Given position, lookahead distance, and path,
    returns the point on path that is instantaneously one look ahead distance away from rover
    If multiple intersections, selects "farthest along" point
    https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
    """
    for i in range(len(waypoints) - 1):
        # Geometry of segment vector and vector from robot to start point
        seg_vec = waypoints[i+1] - waypoints[i]
        rover_vec = waypoints[i] - np.array([x, y])
        a = np.dot(seg_vec, seg_vec)
        b = 2 * np.dot(rover_vec, seg_vec)
        c = np.dot(rover_vec, rover_vec) - ld * ld
        discriminant = b*b-4*a*c
        
        if discriminant >= 0:
            # p1 and p2 are integer multiples of the segment vector corresponding to 
            # where the intersection lies
            p1 = (-b + np.sqrt(discriminant))/(2*a)
            p2 = (-b - np.sqrt(discriminant))/(2*a)
            
            # p1 will always be farther than p2
            if p1 >= 0 and p1 <= 1:
                # Successful intersection
                return p1 * seg_vec + waypoints[i]
                
            if p2 >= 0 and p2 <= 1:
                # Successful intersection
                return p2 * seg_vec + waypoints[i]
    
    # If nothing found
    return waypoints[0,:]