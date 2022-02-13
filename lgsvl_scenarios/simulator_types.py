from enum import Enum
import lgsvl
from lgsvl.geometry import Transform, Vector

# ego_name = "341f52f0-a51d-489b-b17c-de30f79f1d99" # from ROS2 sensor configuration

vehicle_types = [
        # "BoxTruck",
        "Sedan",
        "SUV",
        "Jeep",
        #"SchoolBus",
        "Hatchback"]

pedestrian_types = [
        "Bill",
        "Bob",
        "EntrepreneurFemale",
        "Howard",
        "Johny",
        "Pamela",
        "Presley",
        "Robin",
        "Stephen",
        "Zoe"]

class ego_types(Enum):
        Jaguar = "d460c3b4-4486-484b-b5f7-bc44accc2b8a"
        PilotcarPC4 = "4a4f4856-446c-416d-96b2-d4aa53dc9c42"
        Jaguar_AllSensors = "a7514079-b491-41fa-a581-24ee2494f4e5"
        PilotcarPC4_AllSensors = "c5fa7bef-505b-410d-9a3d-575e51876dbf"
        PilotcarPC4_With2D = "bed6a2bd-d10b-4f29-80f5-5e51bf3b58a8"
        DefaultType = PilotcarPC4_AllSensors

class map_types(Enum):
        AutonomouStuff = "AutonomouStuff"
        SanFrancisco = "SanFrancisco"
        CubeTown = "CubeTown"
        SingleLaneRoad = "SingleLaneRoad"
        EmptyMap = "df8f4b9b-16e6-47ed-810a-fac594ea4cc1"#"EmptyMap"
        GanBIvrit = "4b6de6d5-4bd8-4905-a352-afb0f010d34e" #"GanBIvrit"
        CarteavVillage = "16a8f796-8b70-40e3-b77e-ff815bf7e891" #"CarteavVillage"
        # AutonomouStuff = "2aae5d39-a11c-4516-87c4-cdc9ca784551"
        # SanFrancisco = "5d272540-f689-4355-83c7-03bf11b6865f"
        # CubeTown = "06773677-1ce3-492f-9fe2-b3147e126e27"
        # SingleLaneRoad = "a6e2d149-6a18-4b83-9029-4411d7b2e69a"  
             
class ganBivrit_spawn_cordinates(Enum):
        parking1 ={"lon":34.8265818,"lat":31.9651178}
        parking2 ={"lon":34.8260782,"lat":31.9656669}
        lettersPuzzel = {"lon":34.8263164,"lat":31.9649525}
        waterCircle = {"lon":34.8255831,"lon":31.9657457}
        westGate = {"lon": 34.82465315,"lat":31.96559413}
        westGate2 = {"lon": 34.82446041,"lat":31.96554187}
        backRoad = {"lon": 34.82617902,"lat":31.96468383}
        maze = {"lon": 34.8259694,"lat":31.9636530}
        backEntrance = {"lon": 34.8258718,"lat":31.9634080}
        playYard = {"lon":34.8267078,"lat":31.9636308}
        playYardBack = {"lon":34.8268809,"lat":31.9633875}
        GanSipurCoffee ={"lon":34.8267438,"lat":31.9640471}

# class sensors_sets(Enum):
#         AllSensors = "909be5bb-cfa4-43af-b1f6-5e8780d63382"
#         AllSensorsNoMap = "29821194-2596-4248-aa60-353bffcd6ef4"

def add_random_npc(sim, state):
        import random
        npc_type = random.choice(vehicle_types)     
        npc = None   
        try:
                npc = sim.add_agent(npc_type, lgsvl.AgentType.NPC, state)
        except Exception as e:
                print('add_random_npc failed: ' + npc_type + ': ' + str(e))
        
        return npc

class LatLonData():
        def __init__(self,latitude = 0.0, longitude = 0.0, altitude = 0.0, orientation = 0.0):
                self.latitude = latitude
                self.longitude = longitude
                self.altitude = altitude
                self.orientation = orientation

class positions():
        start_lat_lon_unity = LatLonData(latitude=37.3807272, longitude=-121.9093698, altitude = 15.0)
        dest_lat_lon_unity = LatLonData(latitude=37.3817492, longitude=-121.9089572, altitude = 15.0)

        start_lat_lon_carteav = LatLonData(latitude=31.9717199, longitude=34.7755087, altitude = 90.0)     
  
        start_lat_lon_ganheb = LatLonData(latitude=31.96495480, longitude=34.826310, altitude = 90.0) # LettersPazzle
        dest_lat_lon_ganheb = LatLonData(latitude=31.96544351, longitude=34.82458788, altitude = 90.0) 
               
        start_rotation_unity = lgsvl.Vector(0, -106.5, 0)
        ped_rotation_90_unity = lgsvl.Vector(0, -196.5, 0)
        ped_rotation_45_unity = lgsvl.Vector(0, 118.5, 0)   

        start_rotation_ganheb = lgsvl.Vector(0, 120, 0)
        start_rotation_ganheb_back = lgsvl.Vector(0, -60, 0)
        ped_rotation_90_ganheb = lgsvl.Vector(0, 30, 0)
        ped_rotation_45_ganheb = lgsvl.Vector(0, -15, 0)

def pose_2_ground(sim, pose):
        layer_mask = 0
        layer_mask |= 1 << 0  # 0 is the layer for the road (default)

        hit = sim.raycast(
                pose,
                lgsvl.Vector(0, -1, 0),
                layer_mask,
        )
        if (hit == None):
                print("Failed to transform point to the ground: " + str(pose))

        return hit.point if hit != None else None

def lat_lon_pose_2_map_ground(sim, lat_lon_pose):
        pose = sim.map_from_gps(
                latitude=lat_lon_pose.latitude,
                longitude=lat_lon_pose.longitude,
                altitude=lat_lon_pose.altitude,
                orientation=lat_lon_pose.orientation).position

        # print("Transform from lat/lon: {}".format(zero_pose))

        pose_ground = pose_2_ground(sim, pose)
        if pose_ground == None:
                pose_ground = pose
        return pose_ground

def lat_lon_2_map_ground(sim, lat, lon, alt: float = 100.0):
        pose = sim.map_from_gps(
                latitude=lat,
                longitude=lon,
                altitude=alt,
                orientation=0).position

        # print("Transform from lat/lon: {}".format(zero_pose))

        pose_ground = pose_2_ground(sim, pose)

        if not pose_ground:
                print("pose_2_ground failed: " + str(lat) + ", " + str(lon))
        return pose_ground        


