# -*- coding: utf-8 -*-
"""
bender

@author: YA2148
"""
import numpy as np
import gc

b_radius = 19.05
#pts = np.array([[0,0,0],[10,50,30],[10,50,100],[10,100,100],[-66,170,0],[-66,170,-556]], float)
pts = np.array([[0,0,0],[0,0,60],[151.299,0,289.944],[170.881,-2.612,477.909],[188.059,104.054,505.755],[215.229,245.949,575.032],[180.276,250.610,578.739]],float)
rows = int(pts.shape[0])
vector = np.empty([rows,3])
v_mag = np.empty([rows,1])
norms = np.empty([rows,3])
bends = np.empty([rows,1])
bend_offset = np.empty([rows+1,1])
dir_test = np.empty([rows,1])
rot = np.empty([rows,1])
feed = np.empty([rows,1])
pnt_num = np.empty([rows,1])
bend_radius = np.ones((rows,1))*b_radius
bend_num = np.empty([rows,1])
count = 0

while count < rows:
    if count > 0:
        vector[count] = pts[count]-pts[count-1]
        v_mag[count] = np.linalg.norm(vector[count])
    if count > 1:
        norms[count] = np.cross(vector[count-1],vector[count])
        bends[count] = np.dot(vector[count],vector[count-1])/(np.linalg.norm(vector[count])*np.linalg.norm(vector[count-1]))
        bends[count] = np.arccos(bends[count])
        bends[count] = np.rad2deg(bends[count])
        bend_offset[count] = (b_radius*np.sin(np.deg2rad(bends[count])/2))/(np.sin(np.deg2rad(180-bends[count])/2)) 
    if count > 2:
        dir_test[count] = np.dot(vector[count],vector[count-2])/(np.linalg.norm(vector[count])*np.linalg.norm(vector[count-2]))
        dir_test[count] = np.arccos(dir_test[count])
        dir_test[count] = np.rad2deg(dir_test[count])
        rot[count] = np.dot(norms[count],norms[count-1])/(np.linalg.norm(norms[count])*np.linalg.norm(norms[count-1]))  
        dir_test = np.round(dir_test,3)
        if dir_test[count] <= 90:
            rot[count] = np.arccos(rot[count])
            rot[count] = 360-np.rad2deg(rot[count])
        else:
            rot[count] = np.arccos(rot[count])
            rot[count] = np.rad2deg(rot[count])

    pnt_num[count] = count+1

    count+=1

#Need a separate loop for calculating feed since feed requires n and n+1 values to calculate.    
count = 0
while count < rows:
    if count > 0:
        feed[count] = v_mag[count]-bend_offset[count+1]-bend_offset[count]
    count+=1

bends = np.round(bends,3)
rot = np.round(rot,3)
norms = np.round(norms,3)
feed = np.round(feed,3)
#print bends
#print dir_test
#print rot
#print feed

point_tbl = np.hstack((pnt_num,vector,bends,bend_radius))
#point_tbl = np.delete(point_tbl,(0),axis=0)
cnc_tbl = np.hstack((feed,bends,rot))
np.savetxt("point_tbl.csv", point_tbl, fmt="%10.3f", delimiter=",",header="Point,X,Y,Z,Bend Angle,Bend Radius",comments="")
np.savetxt("cnc_tbl.csv", cnc_tbl, fmt="%10.3f", delimiter=",",header="Feed, Bend Angle, Twist Angle",comments="")
gc.collect()