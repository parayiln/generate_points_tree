#!/usr/bin/env python
import pybullet as p
import time
import math
import pybullet_data
import ray_trace as ray
import create_cylinder as cylinder
import rospy
from geometry_msgs.msg import PointStamped
import numpy as np

useGui = True
def getCenterAndOrient(points, line_count):
    mid_pt=[0,0,0]
    global line
    # points =[[0,0,0],[1,1,1]]
    l=p.addUserDebugLine(points[0], points[1],lineColorRGB =[1,0,0],lineWidth =1000,replaceItemUniqueId=0)
    print(l,line_count)
    pose=p.getBasePositionAndOrientation(0)
    p.removeUserDebugItem(0)
    for i in range(3):
        mid_pt[i]=(points[0][i]+points[1][i])/2
    return mid_pt, pose[1]


def point_wrt_world(points):
    scale=1
    tree_orient =p.getQuaternionFromEuler([0,0,0])
    new_pt=[]
    for i in range(len(points)):
        pt=p.multiplyTransforms([0,0,0],tree_orient,[points[i][0]*scale,points[i][1]*scale,points[i][2]*scale],[0,0,0,1])
        new_pt.append(pt[0])
    return new_pt

def run_pyBullet():
    global ray_count
    if (useGui):
      p.connect(p.GUI)
    else:
      p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-10)
    planeId = p.loadURDF("plane.urdf")
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    robot = p.loadURDF("tree.urdf")
    cy = p.loadURDF("cylinder.urdf")
    tree_orient =p.getQuaternionFromEuler([0,0,1.5])
    # end points of the fitted line
    j =0
    for i in range(int(len(last_pt)/2)):
        print("rays")
        center_pt = last_pt[j]
        center_pt_2 = last_pt[j+1]
        j=i+2

        length=.01
        radius= .03

        point=[]
        point.append(center_pt)
        point.append(center_pt_2)
        point_w=point_wrt_world(point)
        print(point_w)
        center_pt, orient=getCenterAndOrient(point_w, i)
        ray_count=ray.ray_trace(useGui, center_pt, length, orient, cy, ray_count)
        print("done")



def point_to_pt_array(point):
    if isinstance(point, PointStamped):
        point = point.point

    pt = point
    return ([pt.x, pt.y, pt.z])

def next_pose(point):
    global last_pt
    last_pt.append(point_to_pt_array(point))




def get_tree_points():
    rospy.init_node('get_points', anonymous=True)
    rospy.Subscriber("clicked_point", PointStamped, next_pose, queue_size=1)
    rospy.spin()





if __name__ == '__main__':
    global last_pt
    global line
    last_pt=[]
    line=[]
    while(True):
        user_input=input("enter 1 to get points; 2 to run PyBullet; any other key to show the options again\n")
        if user_input=="1":
            get_tree_points()
        elif user_input=="2":
            global ray_count
            ray_count=1
            run_pyBullet()
        else:
            pass
