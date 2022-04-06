import bag2reader as br
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
import navpy
import plotly.express as px
import yaml
import plotly.graph_objects as go
from pathlib import Path
import os.path
from shutil import copyfile


from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import pandas as pd

os.path.join('/home/yehuda/git_repos/carteav_sw/carteav_dev/install/carteav_interfaces/lib/python3.8/site-packages/carteav_interfaces/msg')
from carteav_interfaces.msg import CartLocation


def convertToGeo(point):
    p = np.array(point)
    if len(p.shape) == 1 and p.shape[0] == 2:
        p = np.append(p,0)
    elif len(p.shape)==2 and p.shape[1] == 2:
        p = np.concatenate((p, np.zeros([p.shape[0], 1])), 1)

    ecef_total = base_ecef + navpy.ned2ecef(p * [1 ,-1, -1], base_lla[0], base_lla[1], base_lla[2])
    return np.stack(navpy.ecef2lla(ecef_total),1)

def convertFromGeo(geo):
    geo, N = navpy.utils.input_check_Nx3(geo)
    if N==1:
        ecef = navpy.lla2ecef(geo[0],geo[1],geo[2])
    else:
        ecef = navpy.lla2ecef(geo[:,0],geo[:,1],geo[:,2])
    ned_total = navpy.ecef2ned(ecef - base_ecef, base_lla[0], base_lla[1], base_lla[2])
    return ned_total * [1, -1, -1] # to nwu



# carteav:
base_lla = [31.97171990, 34.77550870, 65]
# imecar:
#base_lla = [36.84086960, 30.61009380, 39.325]
base_ecef = np.array(navpy.lla2ecef(base_lla[0],base_lla[1],base_lla[2])).reshape(1,3)

lat_deg_to_meter = (convertFromGeo(base_lla) - convertFromGeo(base_lla - np.array([1e-3, 0, 0])))[0] * 1e3
lon_deg_to_meter = (convertFromGeo(base_lla) - convertFromGeo(base_lla + np.array([0, 1e-3, 0])))[1] * 1e3




def save_roads(folder_path, output_path, is_one_way):
    print('save_roads')

    yaml_file_path = output_path + '/roads.yaml'
    sql_file_path = output_path + '/roads.sql'

    if(os.path.isfile(yaml_file_path)):
        copyfile(yaml_file_path, yaml_file_path + ".bak")

    if(os.path.isfile(sql_file_path)):
        copyfile(sql_file_path, sql_file_path + ".bak")

    print('Read bag files to Yaml: ' + yaml_file_path)
    offset = np.array([0, 0, 0])
    ret_val = createYamlFromBag(folder_path, yaml_file_path, offset) 
    
    if ret_val == False:
        return

    print('load Yaml: ' + yaml_file_path)
    map = yaml.load(open(yaml_file_path),Loader=yaml.FullLoader)

    print('Cereate Sql file: ' + sql_file_path)

    sql_full = "INSERT INTO tbl_roads(geometry, one_way) VALUES "

    index = 0
    for line in map:
        index = index + 1
        print('parse road no.: ', index)

        
        oneline_geom = "ST_GeomFromText('LINESTRINGZ("
        for point in line:
            point_str = "{0} {1} {2},".format(point[1], point[0], point[2])
            oneline_geom += point_str

            print('Carete Sql file - add point: ' + point_str)

        oneline_geom = oneline_geom[:-(1)]
        oneline_geom = oneline_geom + ")')"

        ST_Simplfy = "ST_simplify({0}, 0.000001) ".format(oneline_geom)

        line_val = "({0}, {1}),".format(ST_Simplfy, is_one_way)
        sql_full = "{0}{1}".format(sql_full, line_val)

    sql_full = sql_full[:-1]
    sql_full = sql_full + ";"

    f = open(sql_file_path, "w")
    f.write(sql_full)
    f.close()

    print('Carete Sql roads Finished')


def save_polygon(folder_path, output_path):

    print('save_polygon')
    yaml_file_path = output_path + '/polygon.yaml'
    sql_file_path = output_path + '/polygon.sql'

    if(os.path.isfile(yaml_file_path)):
        copyfile(yaml_file_path, yaml_file_path + ".bak")

    if(os.path.isfile(sql_file_path)):
        copyfile(sql_file_path, sql_file_path + ".bak")
    
    print('Read bag files to Yaml: ' + yaml_file_path)
    
    offset_drive_to_the_left = np.array([0, 0.6, 0])
    offset_drive_to_the_right = np.array([0, -0.6, 0])
    ret_val = createYamlFromBag(folder_path, yaml_file_path, offset_drive_to_the_left)
    
    if ret_val == False:
        return

    print('load Yaml: ' + yaml_file_path)
    map = yaml.load(open(yaml_file_path),Loader=yaml.FullLoader)

    print('Carete Sql file: ' + sql_file_path)

    sql_full = "INSERT INTO tbl_site_boundries (geometry) VALUES "

    index = 0
    for line in map:
        
        index = index + 1
        print('parse polygon no.: ', index)

        geomStr = ""
        
        first_point = line[0]
        for point in line:
            point_str = "{0} {1} {2}, ".format(point[1],point[0],point[2])
            geomStr += point_str
            
            print('Carete Sql file - add point: ' + point_str)

        # add first point:
        point_str = "{0} {1} {2}".format(first_point[1], first_point[0] ,first_point[2])# + " " + point[2]
        geomStr += point_str
        geom_text_str = "ST_GeomFromText('PolygonZ(({0}))')".format(geomStr)
  
        ST_Simplfy = "ST_simplify({0}, 0.000001)".format(geom_text_str)
        
        sql_full = "{0} ({1}),".format(sql_full, ST_Simplfy)

    sql_full =  "{0};".format(sql_full[:-(1)])
    
    f = open(sql_file_path, "w")
    f.write(sql_full)
    f.close()
    print('Carete Sql polygons Finished')


def createYamlFromBag(folder_path, yaml_name, ant_offset_in_body = None):
    
    map = []
    
    all_dir = os.listdir(folder_path)
    all_dir.sort()

    if len(all_dir) == 0:
        print('Empty folder:  ' + folder_path)
        return False

    for file in all_dir:
        if(file.startswith("_")):
            continue;
        print('adding file ' + file)


        loc_msgs = []

        with Reader(folder_path + file) as r:
            for idx, (topic, msg_type, timestamp, rawdata) in enumerate(r.messages(['/cart_location'])):
                    msg_object = get_message(msg_type)
                    dmsg = deserialize_message(rawdata, msg_object)
                    loc_msgs.append(dmsg)

            if(len(loc_msgs)>0):
                arr = np.array([[e.pose.position.x, e.pose.position.y , e.pose.position.z - ant_offset_in_body[2]] for e in loc_msgs])
                q_vec = np.array([[e.pose.orientation.x, e.pose.orientation.y , e.pose.orientation.z] for e in loc_msgs])
                q_0 = np.array([[e.pose.orientation.w] for e in loc_msgs])
                [rot1,rot2,rot3] = navpy.quat2angle(-q_0,q_vec)
                dcm = navpy.angle2dcm(rot1,rot2,rot3)
                offset_in_nwu = np.einsum("nij,j -> ni", dcm, ant_offset_in_body)
                offset_arr = arr+offset_in_nwu
                geo_arr = convertToGeo(offset_arr)

                # plt.plot(-arr[:, 1], arr[:, 0], '-o')
                # plt.plot(-offset_arr[:, 1], offset_arr[:, 0], '-o')
                # plt.axis('equal')
                # plt.show()

            points = [[float(e[0]) ,float(e[1]) ,float(e[2])] for e in geo_arr]
            map.append(points)
    
    if not os.path.exists(os.path.dirname(yaml_name)):
        os.makedirs(os.path.dirname(yaml_name))

    with open(yaml_name, "w") as f:
        yaml.dump(map, f)

    return True


def createYamlFromBag_old(ant_offset_in_body, folder_path, yaml_name):
    
    map = []
    
    all_dir = os.listdir(folder_path)
    all_dir.sort()

    if len(all_dir) == 0:
        print('Empty folder:  ' + folder_path)
        return False

    for file in all_dir:
        if(file.startswith(".")):
            continue;
        print('adding file ' + file)

        topics = br.bag_to_dataframes(folder_path + file,verbose=False)

        pvt_df = topics['ublox_msgs/msg/NavPVT']
        att_df = topics['ublox_msgs/msg/NavATT']

        tow_pvt = [e.i_tow for e in pvt_df['msgs'].to_numpy()]
        tow_att = [e.i_tow for e in att_df['msgs'].to_numpy()]
        if len(tow_pvt) != len(tow_att):
            dif_count = abs(len(tow_pvt) - len(tow_att))
            if len(tow_pvt) > len(tow_att):
                if tow_pvt[0] != tow_att[0]:
                    pvt_df = pvt_df[dif_count:]
                else:
                    pvt_df = pvt_df[:dif_count]

            if len(tow_att) > len(tow_pvt):
                if tow_att[0] != tow_pvt[0]:
                    att_df = att_df[dif_count:]
                else:
                    att_df = att_df[:dif_count]


        att_deg = np.array([[e.roll * 1e-5, e.pitch * 1e-5, e.heading * 1e-5] for e in att_df['msgs'].to_numpy()])
        dcm = navpy.angle2dcm(att_deg[:,2], att_deg[:,1], att_deg[:,0], input_unit='deg')

        offset_in_nwu = np.einsum("nij,j -> ni", dcm, ant_offset_in_body)
        geo = np.array([[e.lat * 1e-7,e.lon * 1e-7,e.height * 1e-3] for e in pvt_df['msgs'].to_numpy()])
        sideGeo = convertToGeo(convertFromGeo(geo) - offset_in_nwu)
        
        points = [[float(e[0]) ,float(e[1]) ,float(e[2])] for e in sideGeo]
        map.append(points)
    
    if not os.path.exists(os.path.dirname(yaml_name)):
        os.makedirs(os.path.dirname(yaml_name))

    with open(yaml_name, "w") as f:
        yaml.dump(map, f)

    return True

def main(args=None):
# save_polygon('/mnt/DATA/rosbag_records/carteav_bounds_temp/', '/mnt/DATA/rosbag_records/carteav_bounds_temp/output')
#    save_polygon("Mapping/Converter/polygon_data/", "Mapping/Converter/output")
    convert_type = input("To convert roads enter 1 \nTo convert polygons(boundries) enter 2:")
    bags_path = input("Enter path for bags files. (The 'output' folder will be saved in this deirectory): ")
    if not bags_path.endswith('/'):
        bags_path = bags_path + '/'

    if convert_type == '1':
        one_way_input = input("Is roads one way? (y/n): ")
        one_way = 0
        if one_way_input == 'y':
           one_way = 1 

        save_roads(bags_path, bags_path + "_output", one_way)

    if convert_type == '2':
        save_polygon(bags_path, bags_path + "_output")


    
   
if __name__ == '__main__':
    main()