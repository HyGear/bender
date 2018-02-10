# -*- coding: utf-8 -*-
"""
bender: Small script for converting a set of points for a tube into CNC bend data.
This script will read a point file and calculate feed, bend angle, and twist angle.
The results will be outputed to a CSV file which can be imported into Pro/E or other
CAD programs.
"""
import numpy as np
import gc
import re

b_radius = 19.05
f = open('proe.pts.1','r')
temp = []

#The following works for Pro/E point files.
for line in f:
    match = re.findall('(\-?[0-9]+\.[0-9]+)( +)(\-?[0-9]+\.[0-9]+)( +)(\-?[0-9]+\.[0-9]+)',line)
    if match:
        temp.append([float(match[0][0]),float(match[0][2]),float(match[0][4])])
f.close()
pts = np.array(temp,float)

#pts = np.array([[0,0,0],[10,50,30],[10,50,100],[10,100,100],[-66,170,0],[-66,170,-556]], float)
#pts = np.array([[0,0,0],[0,0,60],[151.299,0,289.944],[170.881,-2.612,477.909],[188.059,104.054,505.755],[215.229,245.949,575.032],[180.276,250.610,578.739]],float)
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
        #bend_offset[count] = (b_radius*np.cos(np.deg2rad(bends[count])/2))
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
bends = np.roll(bends,-1)
rot = np.round(rot,3)
rot = np.roll(rot,-1)
norms = np.round(norms,3)
feed = np.round(feed,3)


point_tbl = np.hstack((pnt_num,vector,bends,bend_radius))
#point_tbl = np.delete(point_tbl,(0),axis=0)
cnc_tbl = np.hstack((feed,rot,bends))
cnc_tbl = np.delete(cnc_tbl,0,0)
print (point_tbl)
print (cnc_tbl)
np.savetxt("point_tbl.csv", point_tbl, fmt="%10.3f", delimiter=",",header="Point,X,Y,Z,Bend Angle,Bend Radius",comments="")
np.savetxt("cnc_tbl.csv", cnc_tbl, fmt="%10.3f", delimiter=",",header="Feed, Twist Angle, Bend Angle",comments="")
gc.collect()
