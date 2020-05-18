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
import json
from std_msgs.msg import String
from geographic_msgs.msg import GeoPointStamped
from sensor_msgs.msg import Imu, NavSatFix
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from scipy.spatial.distance import pdist, squareform
from scipy.ndimage import gaussian_filter
from visualization_msgs.msg import MarkerArray, Marker
from collections import OrderedDict


folder_path = '/home/dank-engine/3d_ws/pkloud/'
folder_path_new = '/home/dank-engine/3d_ws/pkloud_merge/'
o = 150
def callBack_1(msg):
    global o
    raw = list(point_cloud2.read_points(msg, skip_nans=True, field_names = ("x", "y")))
    df = pd.DataFrame(raw, columns=['x', 'y'])
    indexNames = df[(df['x'] <= 0) | (df['x'] >= 9) | (df['y'] <= -6) | (df['y'] >= 6)].index
    df.drop(indexNames , inplace=True)
    PairwiseDist = squareform(pdist(df, 'euclidean'))
    RelativeDist = np.diag(PairwiseDist, k=-1)
    lineInd = np.where(RelativeDist>5)

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
        diff = abs(np.subtract(d,gaussian_filter(d, sigma=25)))
        diff_1 += diff

    f = abs(np.subtract(diff_1,gaussian_filter(diff_1, sigma=15)))
    h = np.where(f > 0.75*np.mean(f))

    # plt.scatter(range(len(f)), f)
    # plt.show()

    q = 100
    t = 0
    for u in range(len(h[0])-1):
        e = h[0][u+1] - h[0][u]
        if e > 100:
            if np.mean(f[h[0][u]:h[0][u+1]]) < q:
                q = np.mean(f[h[0][u]:h[0][u+1]])
                t = u

    y_r = g[len(g)-1][h[0][t],1]
    y_l = g[len(g)-1][h[0][t+1],1]
    x_r = g[len(g)-1][h[0][t],0]
    x_l = g[len(g)-1][h[0][t+1],0]  
    # print(x_r, y_r, x_l, y_l)

    filename = folder_path + '{o:0{width}}'.format(o=o+1, width=5) + '.json'
    with open(filename, 'r') as f: 
        print(filename)           
        data = json.load(f, object_pairs_hook=OrderedDict)

    data['Right_Marker'] = {
        'y': y_r,
        'x': x_r
    }
    data['Left_Marker'] = {
        'y': y_l,
        'x': x_l
    }
    data['Center_Marker'] = {
        'y': (y_l+y_r)/2.0,
        'x': (x_l+x_r)/2.0
    }
    
    filename = folder_path_new + '{o:0{width}}'.format(o=o+1, width=4) + '.json'
    with open(filename, 'w') as outfile:
        json.dump(data, outfile, indent=4)

    o = o+1


def main():
    rospy.init_node('marker_merger')
    rospy.Subscriber("/pc2", PointCloud2, callBack_1)                                            
    rospy.spin()

if __name__ == '__main__':
    main()