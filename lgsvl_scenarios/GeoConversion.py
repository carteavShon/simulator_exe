import numpy as np
import navpy

class GeoConversion:
    def __init__(self,base_lla):
        self.base_lla = base_lla
        self.base_ecef = np.array(navpy.lla2ecef(base_lla[0], base_lla[1], base_lla[2])).reshape(1, 3)
        self.lat_deg_to_meter = (self.convertFromGeo(base_lla) - self.convertFromGeo(base_lla - np.array([1e-3, 0, 0])))[0] * 1e3
        self.lon_deg_to_meter = (self.convertFromGeo(base_lla) - self.convertFromGeo(base_lla + np.array([0, 1e-3, 0])))[1] * 1e3

    def convertToGeo(self,point):
        p = np.array(point)
        if len(p.shape) == 1 and p.shape[0] == 2:
            p = np.append(p,0)
        elif len(p.shape)==2 and p.shape[1] == 2:
            p = np.concatenate((p, np.zeros([p.shape[0], 1])), 1)

        p, N = navpy.utils.input_check_Nx3(p)

        ecef_total = self.base_ecef + navpy.ned2ecef(p * [1 ,-1, -1], self.base_lla[0], self.base_lla[1], self.base_lla[2])
        toRet = navpy.ecef2lla(ecef_total)
        if N==1:
            return toRet
        else:
            return np.stack(toRet,1)

    def convertFromGeo(self,geo):
        geo, N = navpy.utils.input_check_Nx3(geo)
        if N==1:
            ecef = navpy.lla2ecef(geo[0],geo[1],geo[2])
        else:
            ecef = navpy.lla2ecef(geo[:,0],geo[:,1],geo[:,2])
        ned_total = navpy.ecef2ned(ecef - self.base_ecef, self.base_lla[0], self.base_lla[1], self.base_lla[2])
        return ned_total * [1, -1, -1] # to nwu


