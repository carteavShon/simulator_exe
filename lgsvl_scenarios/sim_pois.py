
import simulator_types
import lgsvl
from lgsvl.geometry import Spawn, Transform, Vector
import math

def WG_Bells_Path(sim: lgsvl.Simulator):
    return [
        simulator_types.lat_lon_2_map_ground(sim, 31.964906, 34.824580),
        simulator_types.lat_lon_2_map_ground(sim, 31.964906, 34.824580),
        simulator_types.lat_lon_2_map_ground(sim, 31.964894, 34.824614),
        simulator_types.lat_lon_2_map_ground(sim, 31.964877, 34.824661),
        simulator_types.lat_lon_2_map_ground(sim, 31.964858, 34.824706),
        simulator_types.lat_lon_2_map_ground(sim, 31.964838, 34.824749),
        simulator_types.lat_lon_2_map_ground(sim, 31.964817, 34.824794),
        simulator_types.lat_lon_2_map_ground(sim, 31.964799, 34.824837),
        simulator_types.lat_lon_2_map_ground(sim, 31.964780, 34.824886),
        simulator_types.lat_lon_2_map_ground(sim, 31.964761, 34.824932),
        simulator_types.lat_lon_2_map_ground(sim, 31.964742, 34.824978),
        simulator_types.lat_lon_2_map_ground(sim, 31.964723, 34.825024),
        simulator_types.lat_lon_2_map_ground(sim, 31.964701, 34.825075),
        simulator_types.lat_lon_2_map_ground(sim, 31.964680, 34.825127),
        simulator_types.lat_lon_2_map_ground(sim, 31.964654, 34.825187),
        simulator_types.lat_lon_2_map_ground(sim, 31.964630, 34.825244),
        simulator_types.lat_lon_2_map_ground(sim, 31.964604, 34.825302),
        simulator_types.lat_lon_2_map_ground(sim, 31.964578, 34.825356),
        simulator_types.lat_lon_2_map_ground(sim, 31.964544, 34.825409),
        simulator_types.lat_lon_2_map_ground(sim, 31.964507, 34.825452),
        simulator_types.lat_lon_2_map_ground(sim, 31.964468, 34.825491),
        simulator_types.lat_lon_2_map_ground(sim, 31.964432, 34.825531),
        simulator_types.lat_lon_2_map_ground(sim, 31.964403, 34.825569),
        simulator_types.lat_lon_2_map_ground(sim, 31.964372, 34.825613),
        simulator_types.lat_lon_2_map_ground(sim, 31.964343, 34.825654),
        simulator_types.lat_lon_2_map_ground(sim, 31.964310, 34.825698),
        simulator_types.lat_lon_2_map_ground(sim, 31.964279, 34.825743),
        simulator_types.lat_lon_2_map_ground(sim, 31.964248, 34.825790),
        simulator_types.lat_lon_2_map_ground(sim, 31.964219, 34.825833),
        simulator_types.lat_lon_2_map_ground(sim, 31.964193, 34.825870),
        simulator_types.lat_lon_2_map_ground(sim, 31.964166, 34.825911),
        simulator_types.lat_lon_2_map_ground(sim, 31.964136, 34.825959),
        simulator_types.lat_lon_2_map_ground(sim, 31.964105, 34.826011),
        simulator_types.lat_lon_2_map_ground(sim, 31.964071, 34.826068),
        simulator_types.lat_lon_2_map_ground(sim, 31.964033, 34.826123),
        simulator_types.lat_lon_2_map_ground(sim, 31.963998, 34.826168),
        simulator_types.lat_lon_2_map_ground(sim, 31.963973, 34.826198),
        simulator_types.lat_lon_2_map_ground(sim, 31.963944, 34.826223),
        simulator_types.lat_lon_2_map_ground(sim, 31.963903, 34.826246),
        simulator_types.lat_lon_2_map_ground(sim, 31.963862, 34.826259),
        simulator_types.lat_lon_2_map_ground(sim, 31.963807, 34.826268),
        simulator_types.lat_lon_2_map_ground(sim, 31.963761, 34.826274),
        simulator_types.lat_lon_2_map_ground(sim, 31.963720, 34.826274),
        simulator_types.lat_lon_2_map_ground(sim, 31.963687, 34.826266),
        simulator_types.lat_lon_2_map_ground(sim, 31.963654, 34.826249),
        simulator_types.lat_lon_2_map_ground(sim, 31.963626, 34.826224),
        simulator_types.lat_lon_2_map_ground(sim, 31.963603, 34.826203),
        simulator_types.lat_lon_2_map_ground(sim, 31.963573, 34.826182),
        simulator_types.lat_lon_2_map_ground(sim, 31.963543, 34.826168),
        simulator_types.lat_lon_2_map_ground(sim, 31.963503, 34.826152),
        simulator_types.lat_lon_2_map_ground(sim, 31.963463, 34.826135),
        simulator_types.lat_lon_2_map_ground(sim, 31.963424, 34.826116),
        simulator_types.lat_lon_2_map_ground(sim, 31.963383, 34.826096),
        simulator_types.lat_lon_2_map_ground(sim, 31.963340, 34.826076),
        simulator_types.lat_lon_2_map_ground(sim, 31.963297, 34.826055),
        simulator_types.lat_lon_2_map_ground(sim, 31.963255, 34.826029),
        simulator_types.lat_lon_2_map_ground(sim, 31.963224, 34.825999),
        simulator_types.lat_lon_2_map_ground(sim, 31.963196, 34.825960),
        simulator_types.lat_lon_2_map_ground(sim, 31.963173, 34.825925),
        simulator_types.lat_lon_2_map_ground(sim, 31.963156, 34.825900),
        simulator_types.lat_lon_2_map_ground(sim, 31.963133, 34.825874),
        simulator_types.lat_lon_2_map_ground(sim, 31.963106, 34.825853),
        simulator_types.lat_lon_2_map_ground(sim, 31.963078, 34.825840),
        simulator_types.lat_lon_2_map_ground(sim, 31.963046, 34.825833),
        simulator_types.lat_lon_2_map_ground(sim, 31.963008, 34.825825),
        simulator_types.lat_lon_2_map_ground(sim, 31.962982, 34.825819),
        simulator_types.lat_lon_2_map_ground(sim, 31.962969, 34.825816),
        simulator_types.lat_lon_2_map_ground(sim, 31.962961, 34.825813),
        simulator_types.lat_lon_2_map_ground(sim, 31.962953, 34.825812),
        simulator_types.lat_lon_2_map_ground(sim, 31.962952, 34.825811),
        simulator_types.lat_lon_2_map_ground(sim, 31.962900, 34.825770)]

def angle_btw_2_points_deg(pointA, pointB):
  dx = pointB.x - pointA.x
  dy = pointB.z - pointA.z
  return math.degrees(math.atan2(dx,dy)) #remove degrees if you want your answer in radians

def get_yaw(points: list, at_index, default_yaw):
    if len(points) <= 1:
        return default_yaw

    if at_index >= len(points) - 1: # if last point
        return get_yaw(points, at_index-1, default_yaw) # yaw at previous index
    
    return angle_btw_2_points_deg(points[at_index], points[at_index+1])

def interpolate_yaw(points: list):
    for i in range(0, len(points)-1):
        points[i].angle.y = (points[i].angle.y + points[i+1].angle.y)/2
    return points

def get_drive_points(points: list, speed: float, default_yaw) -> list:
    drive_points = [lgsvl.DriveWaypoint(
        position=points[i], 
        speed=speed, 
        angle=Vector(0.0, get_yaw(points, i, default_yaw), 0.0)) for i in range(0, len(points))] 

    # drive_points = interpolate_yaw(drive_points)
    
    return drive_points 