import math
from geometry_msgs.msg import Point
from time import time

import math

def quaternion_to_euler(q):
    q0, q1, q2, q3 = q

    sinr_cosp = 2 * (q0 * q1 + q2 * q3)
    cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (q0 * q2 - q3 * q1)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (q0 * q3 + q1 * q2)
    cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def calculate_heading_error(goal_heading, current_heading):
    '''Takes in a goal heading and current heading, returns requried angle change where positive indicates a clockwise angle change required to make the goal and current heading be the same'''
    
    signed_angle_change = goal_heading - current_heading
    
    if signed_angle_change > 180:
        signed_angle_change -= 360
    elif signed_angle_change < -180:
        signed_angle_change += 360
    
    return signed_angle_change

def calculate_goal_heading(lat1, lon1, lat2, lon2)-> float:
    '''returns a goal heading angle where north is 0 degrees, east is 90, south is 180, west is 270'''
    d_lon = math.radians(lon2 - lon1)
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    
    x = math.sin(d_lon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(d_lon))
    
    initial_bearing = math.atan2(x, y)
    initial_bearing = math.degrees(initial_bearing)
    return (initial_bearing + 360) % 360

def calculate_distance(lat1, lon1, lat2, lon2)-> float:
    '''returns a goal heading angle where (in degrees) north is 0, east is 90, south is 180, west is 270'''
    earth_radius_meters = 6_378_137 
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    
    a = math.sin(delta_phi / 2.0) ** 2 + \
        math.cos(phi1) * math.cos(phi2) * \
        math.sin(delta_lambda / 2.0) ** 2
    
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return earth_radius_meters * c

def euuclidean_distance(pt1: Point, pt2: Point) -> float:
    '''calculates the euclidean distance between two points in the units the points were given'''
    return math.sqrt((pt1.x - pt2.x)**2 + (pt1.y - pt2.y)**2)