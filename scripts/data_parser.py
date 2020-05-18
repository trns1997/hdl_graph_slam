#!/usr/bin/env python
import rospy
import json
import tf
import os
import glob
import pathlib
import numpy as np
from std_msgs.msg import String
from geographic_msgs.msg import GeoPointStamped
from sensor_msgs.msg import Imu, NavSatFix
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import MarkerArray, Marker


def main():
    # Load File
    folder_path = '/home/dank-engine/3d_ws/json2_merger/'
    path = list(pathlib.Path(folder_path).glob('*.json'))
    start_idx = 700

    # for i in range(start_idx, start_idx+len(path)-1):
    for i in range(200, 552):
        filename = folder_path + \
            '{i:0{width}}'.format(i=i+1, width=4) + '.json'
        with open(filename, 'r') as f:
            print(filename)
            data = json.load(f)

            # GPS Topic
            gps_coor = GeoPointStamped()
            gps_map = NavSatFix()
            gps_pub = rospy.Publisher('/gps', GeoPointStamped, queue_size=10)
            gps_map_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)

            # PointCloud Topic
            cloud = PointCloud2()
            pc_data = rospy.Publisher(
                '/velodyne_points', PointCloud2, queue_size=32)

            rospy.init_node('converter')

            now = rospy.get_rostime()

            # Publish GPS Data in ROS
            gps_coor.header.stamp = now
            gps_coor.header.frame_id = "gps"
            gps_coor.position.latitude = data["INS_SWIFT"]["Latitude"]
            gps_coor.position.longitude = data["INS_SWIFT"]["Longitude"]
            gps_coor.position.altitude = float('nan')

            gps_map.header.stamp = now
            gps_map.header.frame_id = "base_link"
            gps_map.latitude = data["INS_SWIFT"]["Latitude"]
            gps_map.longitude = data["INS_SWIFT"]["Longitude"]
            gps_map.altitude = data["INS_SWIFT"]["Altitude"]

            gps_map_pub.publish(gps_map)
            gps_pub.publish(gps_coor)

            # Publish Point Cloud Data
            point = []
            scan = data["PointCloud"]
            for i in range(len(scan)):
                if(scan[i]["x"] != None):
                    xyzi = (scan[i]["x"], scan[i]["y"], scan[i]
                            ["z"], scan[i]["intensity"])
                    point.append(xyzi)

                else:
                    xyzi = (np.nan, np.nan, np.nan, 0)
                    point.append(xyzi)

            cloud = xyzi_array_to_pointcloud2(
                point, now, "velodyne")
            pc_data.publish(cloud)

            r = rospy.Rate(1)  # 10hz
            r.sleep()


def xyzi_array_to_pointcloud2(points, stamp=None, frame_id=None, seq=None):
    '''
    Create a sensor_msgs.PointCloud2 from an array
    of points.
    '''
    msg = PointCloud2()

    if stamp:
        msg.header.stamp = stamp
    if frame_id:
        msg.header.frame_id = frame_id
    if seq:
        msg.header.seq = seq
    else:
        N = len(points)
        xyzi = np.array(np.hstack([points]), dtype=np.float32)
        msg.height = 1
        msg.width = N

    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1)
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = msg.point_step * N
    msg.is_dense = True
    msg.data = xyzi.tostring()

    return msg


if __name__ == '__main__':
    main()
