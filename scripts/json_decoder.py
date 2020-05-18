#!/usr/bin/env python
import rospy
import json
import tf
import os,glob
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

    id = 0
    k = 0
    num = 20
    arr_rx = np.zeros(num)
    arr_ry = np.zeros(num)
    arr_lx = np.zeros(num)
    arr_ly = np.zeros(num)
    arr_cx = np.zeros(num)
    arr_cy = np.zeros(num)

    for i in range(100, 552):
        filename = folder_path + '{i:0{width}}'.format(i=i+1, width=4) + '.json'
        with open(filename, 'r') as f: 
            print(filename)           
            data = json.load(f)

            # GPS Topic
            gps_coor = GeoPointStamped()
            gps_map = NavSatFix()
            gps_pub = rospy.Publisher('/gps', GeoPointStamped, queue_size=10)
            gps_map_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)
            
            #IMU Topic
            imu_coor = Imu()
            imu_pub = rospy.Publisher('/imu', Imu, queue_size=10)

            #PointCloud Topic
            cloud = PointCloud2()
            pc_data = rospy.Publisher('/velodyne_points', PointCloud2, queue_size=32)


            rospy.init_node('converter')


            # Publish GPS Data in ROS
            gps_coor.header.frame_id = "gps"
            gps_coor.position.latitude = data["INS_SWIFT"]["Latitude"]
            gps_coor.position.longitude = data["INS_SWIFT"]["Longitude"]
            gps_coor.position.altitude =  float('nan')

            gps_map.header.frame_id = "base_link"
            gps_map.latitude = data["INS_SWIFT"]["Latitude"]
            gps_map.longitude = data["INS_SWIFT"]["Longitude"]
            gps_map.altitude = data["INS_SWIFT"]["Altitude"]

            gps_pub.publish(gps_coor)
            gps_map_pub.publish(gps_map)

            # Publish IMU Data in ROS
            roll = data["INS_SWIFT"]["Roll"]
            pitch = data["INS_SWIFT"]["Pitch"]
            yaw = data["INS_SWIFT"]["Yaw"]
            quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            imu_coor.header.frame_id = "imu"
            imu_coor.header.stamp = rospy.get_rostime()
            imu_coor.orientation.x = quaternion[0]
            imu_coor.orientation.y = quaternion[1]
            imu_coor.orientation.z = quaternion[2]
            imu_coor.orientation.w = quaternion[3]
            imu_coor.orientation_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            imu_pub.publish(imu_coor)

            # Publish Point Cloud Data
            point = []
            scan = data["PointCloud"]
            for i in range(len(scan)):
                if(scan[i]["x"] != None):
                    xyzi = (scan[i]["x"], scan[i]["y"], scan[i]["z"], scan[i]["intensity"])
                    point.append(xyzi)
                
                else:
                    xyzi = (np.nan, np.nan, np.nan, 0)
                    point.append(xyzi)

            cloud = xyzi_array_to_pointcloud2(point, rospy.get_rostime(), "velodyne" )
            pc_data.publish(cloud)

            lane_mark = rospy.Publisher('/marker_lane', Marker, queue_size=100)
            marker_l = Marker()
            marker_l.header.frame_id = "base_link"
            marker_l.type = marker_l.TEXT_VIEW_FACING
            marker_l.action = marker_l.ADD
            marker_l.scale.x = 5
            marker_l.scale.y = 5
            marker_l.scale.z = 5
            marker_l.color.a = 1.0
            marker_l.color.r = 1.0
            marker_l.color.g = 1.0
            marker_l.color.b = 0.0
            marker_l.pose.position.x = 0
            marker_l.pose.position.y = 30
            marker_l.pose.position.z = 1
            marker_l.pose.orientation.w = 1.0
            marker_l.text = "Lanes: " + str(data["Lanes"]) + '\n' + "Width: " + str(data["Width"]) + '\n' + "Slope_Hoizontal: " + str(data["Slope"]["Slope_Hoizontal"])
            marker_l.id = 0
            lane_mark.publish(marker_l)

            try:
                r = rospy.Rate(1) # 10hz
                lane_r = data["Right_Marker"]
                lane_l = data["Left_Marker"]
                lane_c = data["Center_Marker"]

                publisher = rospy.Publisher('/drive_region', MarkerArray, queue_size=100)

                arr_rx[k%num] = lane_r["x"]
                arr_ry[k%num] = lane_r["y"]
                arr_lx[k%num] = lane_l["x"]
                arr_ly[k%num] = lane_l["y"]
                arr_cx[k%num] = lane_c["x"]
                arr_cy[k%num] = lane_c["y"]

                # if k == 10:
                markerArray = MarkerArray()
                
                marker = Marker()
                marker.header.frame_id = "base_link"
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                marker.scale.x = 2
                marker.scale.y = 2
                marker.scale.z = 2
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.pose.orientation.w = 1.0
                marker.pose.position.x = np.average(arr_rx)
                marker.pose.position.y = np.average(arr_ry)
                marker.pose.position.z = 0

                marker1 = Marker()
                marker1.header.frame_id = "base_link"
                marker1.type = marker.SPHERE
                marker1.action = marker.ADD
                marker1.scale.x = 2
                marker1.scale.y = 2
                marker1.scale.z = 2
                marker1.color.a = 1.0
                marker1.color.r = 1.0
                marker1.color.g = 1.0
                marker1.color.b = 0.0
                marker1.pose.orientation.w = 1.0
                marker1.pose.position.x = np.average(arr_lx)
                marker1.pose.position.y = np.average(arr_ly)
                marker1.pose.position.z = 0

                marker2 = Marker()
                marker2.header.frame_id = "base_link"
                marker2.type = marker.SPHERE
                marker2.action = marker.ADD
                marker2.scale.x = 2
                marker2.scale.y = 2
                marker2.scale.z = 2
                marker2.color.a = 1.0
                marker2.color.r = 0.0
                marker2.color.g = 0.0
                marker2.color.b = 1.0
                marker2.pose.orientation.w = 1.0
                marker2.pose.position.x = np.average(arr_cx)
                marker2.pose.position.y = np.average(arr_cy)
                marker2.pose.position.z = 0

                markerArray.markers.append(marker)
                markerArray.markers.append(marker1)
                markerArray.markers.append(marker2)


                # Renumber the marker IDs
                for m in markerArray.markers:
                    m.id = id
                    id += 1

                # Publish the MarkerArray
                publisher.publish(markerArray)
                # k = 0

                k = k + 1
                r.sleep()
            
            except KeyError:
                r = rospy.Rate(1) # 10hz
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
    msg.is_dense = True;
    msg.data = xyzi.tostring()

    return msg 
 
if __name__ == '__main__':
    main()