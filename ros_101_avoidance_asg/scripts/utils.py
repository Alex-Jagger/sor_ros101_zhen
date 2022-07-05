"""
Utility library
"""
import math

def xy_to_range_angle(x, y):
    rng = math.sqrt(x*x + y*y)
    angle = math.atan2(y, x)
    return (rng, angle)

def error_angle(target, source):
    ang     = target - source
    if math.fabs(ang) > math.pi:
        if ang > 0: 
            ang -= 2.0*math.pi
        else: 
            ang += 2.0*math.pi
        
    return ang
