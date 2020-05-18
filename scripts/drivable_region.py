#!/usr/bin/env python
import rospy
import json
import tf
import os,glob
import pathlib
import numpy as np
import matplotlib.pyplot as plt
import itertools 
import pandas as pd
import time
import math
from std_msgs.msg import String
from geographic_msgs.msg import GeoPointStamped
from sensor_msgs.msg import Imu, NavSatFix
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from scipy.spatial.distance import pdist, squareform
from scipy.ndimage import gaussian_filter
from visualization_msgs.msg import MarkerArray, Marker

x_r = 0
x_l = 0
y_r = 0
y_l = 0  
def callBack_1(msg):
    global x_r, x_l, y_r, y_l
    raw = list(point_cloud2.read_points(msg, skip_nans=True, field_names = ("x", "y")))
    df = pd.DataFrame(raw, columns=['x', 'y'])
    # plt.scatter(df['x'], df['y'])
    indexNames = df[(df['x'] <= 0) | (df['x'] >= 9) | (df['y'] <= -6) | (df['y'] >= 6)].index
    df.drop(indexNames , inplace=True)
    PairwiseDist = squareform(pdist(df, 'euclidean'))
    RelativeDist = np.diag(PairwiseDist, k=-1)
    lineInd = np.where(RelativeDist>5)
    # plt.scatter(df['x'], df['y'])
    # plt.show()

    a = 0
    l = {}
    length = 0
    for i in range(len(lineInd[0])-1):
        if (lineInd[0][i+1] - lineInd[0][i]) > 500:
            l[a] = df[lineInd[0][i]+10:lineInd[0][i+1]]
            # plt.scatter(l[a]['x'],l[a]['y'])
            if (len(l[a]) > length):
                length = len(l[a])
            a += 1
    # plt.show()

    g = {}
    for j in range(len(l)):
        ln = abs(length - len(l[j]))
        if ln % 2 == 0:
            a = np.zeros((ln/2, 2))
            b = a

        else:
            a = np.zeros((int((ln/2.0)+0.5), 2))
            b = np.zeros((int((ln/2.0)-0.5), 2))
        g[j] = np.concatenate((a, l[j], b))
    #     plt.scatter(g[j][:,0],g[j][:,1])
    # plt.show()

    
    diff_1 = 0
    for k in range(len(g)-1):
        d = abs(g[k+1][:,0] - g[k][:,0])
        d = np.subtract(d, np.mean(d))
        # plt.scatter(range(len(d)), d)
        diff = abs(np.subtract(d,gaussian_filter(d, sigma=25)))
        diff_1 += diff

    # plt.show()

    f = abs(np.subtract(diff_1,gaussian_filter(diff_1, sigma=15)))
    h = np.where(f > 0.75*np.mean(f))

    # plt.scatter(range(len(f)), f)
    # plt.scatter(range(len(f)), 0.75*np.mean(f)*np.ones(len(f)))
    # plt.show()

    q = 100
    t = 0
    for u in range(len(h[0])-1):
        e = h[0][u+1] - h[0][u]
        if e > 100:
            if np.mean(f[h[0][u]:h[0][u+1]]) < q:
                q = np.mean(f[h[0][u]:h[0][u+1]])
                t = u

    # x_1 = []
    # x_2 = []
    # y_1 = []
    # y_2 = [] 
   
    # for m in range(len(g)):
    #     plt.scatter(g[m][:,0], g[m][:,1], c = 'b')
    #     plt.scatter(g[m][h[0][t]:h[0][t+1],0], g[m][h[0][t]:h[0][t+1],1], c = 'r')

    # plt.show()

    # plt.show(block=False)
    # plt.pause(0.1)
    # plt.close() 

    # x_r = np.mean(np.asarray(x_1))
    # x_l = np.mean(np.asarray(x_2))
    # y_r = np.mean(np.asarray(y_1))
    # y_l = np.mean(np.asarray(y_2))

    x_r = g[len(g)-1][h[0][t],1]
    x_l = g[len(g)-1][h[0][t+1],1]
    y_r = g[len(g)-1][h[0][t],0]
    y_l = g[len(g)-1][h[0][t+1],0]  
    # print(x_r, y_r, x_l, y_l)
  
count = 0
id = 0
MARKERS_MAX = 100
def callBack_2(msg):
    global count, MARKERS_MAX, id
    # distance = [msg.markers[0].points[-1].x - msg.markers[0].points[-2].x, msg.markers[0].points[-1].y - msg.markers[0].points[-2].y, msg.markers[0].points[-1].z - msg.markers[0].points[-2].z]
    # norm = math.sqrt(distance[0] ** 2 + distance[1] ** 2 + distance[2] ** 2)
    # direction = [distance[0] / norm, distance[1] / norm, distance[2] / norm]   

    publisher = rospy.Publisher('/drive_region', MarkerArray, queue_size=100)
    
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
    marker.pose.position.x = y_r
    marker.pose.position.y = x_r
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
    marker1.pose.position.x = y_l
    marker1.pose.position.y = x_l
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
    marker2.pose.position.x = (y_l+y_r)/2.0
    marker2.pose.position.y = (x_l+x_r)/2.0
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

    count += 1


def main():
    rospy.init_node('driveable_region')
    rospy.Subscriber("/pc2", PointCloud2, callBack_1)
    rospy.Subscriber("/hdl_graph_slam/markers", MarkerArray, callBack_2)                                                  
    rospy.spin()

if __name__ == '__main__':
    main()