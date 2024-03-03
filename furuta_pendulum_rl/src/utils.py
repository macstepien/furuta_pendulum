import math

def limit_minus_pi_pi(angle):
    angle = math.fmod(angle, 2 * math.pi)
    if angle > math.pi:
        angle = 2 * math.pi - angle
    elif angle < -math.pi:
        angle = 2 * math.pi + angle
    return angle
