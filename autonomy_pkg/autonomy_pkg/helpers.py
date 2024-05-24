import math

class LatLong():

    def __init__(self, lat, long):
        self.lat: float = lat
        self.long: float = long

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