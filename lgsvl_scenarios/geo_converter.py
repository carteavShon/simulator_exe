import numpy as np
import navpy

geo_converter_instance =None # geo_converter.geoConverter(31.97171990, 34.77550870, 0)

class geoConverter():

    def __init__ (self, base_lat, base_lon, base_alt):
        self.base_lla = [base_lat, base_lon, base_alt]
        self.base_ecef = np.array(navpy.lla2ecef(self.base_lla[0],self.base_lla[1],self.base_lla[2])).reshape(1,3)

        lat_deg_to_meter = (self.convertFromGeo(self.base_lla) - self.convertFromGeo(self.base_lla - np.array([1e-3, 0, 0])))[0] * 1e3
        lon_deg_to_meter = (self.convertFromGeo(self.base_lla) - self.convertFromGeo(self.base_lla + np.array([0, 1e-3, 0])))[1] * 1e3

    def convertToGeo(self,point):
        p = np.array(point)
        if p.shape[0] == 2:
            p = np.append(p,0)
        ecef_total =  self.base_ecef + navpy.ned2ecef(p*[1 ,-1, -1],  self.base_lla[0],  self.base_lla[1],  self.base_lla[2])
        return navpy.ecef2lla(ecef_total)

    def convertToGeo1(self,point):
        p = np.array(point)
        if len(p.shape) == 1 and p.shape[0] == 2:
            p = np.append(p,0)
        elif len(p.shape)==2 and p.shape[1] == 2:
            p = np.concatenate((p, np.zeros([p.shape[0], 1])), 1)

        ecef_total = self.base_ecef + navpy.ned2ecef(p * [1 ,-1, -1], self.base_lla[0], self.base_lla[1], self.base_lla[2])
        return np.stack(navpy.ecef2lla(ecef_total),1)
        

    def convertFromGeo(self, geo):
        geo, N = navpy.utils.input_check_Nx3(geo)
        if N==1:
            ecef = navpy.lla2ecef(geo[0],geo[1],geo[2])
        else:
            ecef = navpy.lla2ecef(geo[:,0],geo[:,1],geo[:,2])
        ned_total = navpy.ecef2ned(ecef - self.base_ecef, self.base_lla[0], self.base_lla[1], self.base_lla[2])
        return ned_total * [1, -1, -1] # to nwu





#ant_offset_in_body = np.array([0, -0.6, 1.74]) # to the left


    # def test_geo_convert(self):
    #     from geometry_msgs.msg import Point
    #     coffee = Point(x=34.7752572, y=31.9716892, z=0.0)
    #     office = Point(x=34.7754292, y=31.9716770, z=0.0)
    #     conf_room = Point(x=34.7752253, y=31.9716106, z=0.0)

    #     p1 = mapping_data_parser.point_geo2local(coffee)
    #     p2 = mapping_data_parser.point_geo2local(office)
    #     p3 = mapping_data_parser.point_geo2local(conf_room)

    #     coffee_after = mapping_data_parser.point_local2geo(p1)
    #     office_after = mapping_data_parser.point_local2geo(p2)
    #     conf_room_after = mapping_data_parser.point_local2geo(p3)

    #     print("test convert done")