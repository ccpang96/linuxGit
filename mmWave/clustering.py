#!/usr/bin/env python
#source /opt/ros/kinetic/setup.bash
import  rospy
from sensor_msgs.msg import PointCloud2 
from sensor_msgs import point_cloud2 as pc2 
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
from sklearn.cluster import DBSCAN

x = []                           #list of all x co-ordinates of detected points
y = []                           #list of all y co-ordinates of detected points
c = []
clusters = []                    #list of lists each containing every point of cluster as a list of two coordinates
centroids = []                   #list of the centroids of all the clusters
r = []                           #list of radius of all the clusters
fig = plt.figure()
ax1 = fig.add_subplot(1, 3, 1)
ax2 = fig.add_subplot(1, 3, 2)
ax3 = fig.add_subplot(1, 3, 3)
unique_labels = {}
x1 = []                          #list of all x coordinates of centroid of clusters
y1 = []                          #list of all y coordinates of centroid of clusters

#class BTXYZ:

def callback(data):
    global x
    global y
    global clusters
    global r
    global unique_labels
    global centroids
    global x1
    global y1
    x = []
    y = []
    c = []
    r = []
    clusters = []
    centroids = []
    x1 = []
    y1 = []

    #print("Start of frame ************* \n")
    for p in pc2.read_points(data, field_names = ("x", "y", "z"), skip_nans=True): #reading data from the rosbag file
        if (p[1]<= 2 and p[1]>= -2 and p[0]<=5):
	    #print " x : %f  y: %f  " %(p[1],p[0])   #x is p[1] and y is p[0]	
            x.append(p[1])
            y.append(p[0])
    x = np.array(x)
    y = np.array(y)
    z = [[x[j], y[j]] for j in range(len(x))]                              #z is the list of of lists containing all the points 
    cls = DBSCAN(eps=0.5, min_samples=3, algorithm='kd_tree').fit(z)   #clustering algorithm, eps is the maximum distance between two points for them to be in the same cluster and min_samples is the minimum points for it to be called a cluster
    labels = cls.labels_                                                   #gives an integer to all the points of a particular cluster. Cluster integers start from 0. -1 is given to all the points classified as noise
    unique_labels = set(labels)
    for i in unique_labels:
        if i is not (-1):                #not considering points marked as noise
            clusters.append([])
            centroids.append([0, 0])
            r.append([])
            for j in range(len(labels)):      
                if labels[j] == i:
                    clusters[i].append(z[j])            #grouping points that belong to a particular cluster together
                    centroids[i][0]+= z[j][0]           #adding x coordinates of points in a particular cluster
                    centroids[i][1]+= z[j][1]           #adding y coordinates of points in a particular cluster
            centroids[i][0]/= len(clusters[i])          #calculating x coordinate of centroid
            centroids[i][1]/= len(clusters[i])          #calculating y coordinate of centroid
            x1.append(centroids[i][0])                  #list of x coordinates of centroids
            y1.append(centroids[i][1])                  #list of y coordinates of centroids 
            r[i]=max([(((clusters[i][k][0] - centroids[i][0])**2) + ((clusters[i][k][1] - centroids[i][1])**2)) for k in range(len(clusters[i]))])  #finding radius using the distance between the centroid and farthest point from it
    print(r[i])                                         #approximate radius of the cluster 
    #print("End of frame ************* \n")
    	
def update_plot(i):                              
    global x 
    global y
    global centroids
    global unique_labels
    global r
    global x1
    global y1

    ax1.clear()
    ax2.clear()
    ax2.set_xlim([-10, 10])                             
    ax2.set_ylim([0, 10])
    ax1.set_xlim([-10, 10])
    ax1.set_ylim([0, 10])
    ax1.scatter(x1, y1)                                #plotting centroids 
    ax2.scatter(x, y)                                  #plotting all the points

def listener():
    global x
    global y
    rospy.init_node('listener', anonymous=True)	
    rospy.Subscriber("/mmWaveDataHdl/RScan", PointCloud2, callback)
    anim = animation.FuncAnimation(fig, update_plot, interval = 1)       #animation of the plot of points. "interval" can be changed to change the time in which update_plot is called everytime 
    plt.show()
    rospy.spin()


if __name__=='__main__' or __name__=='listener':
    listener()
