#!/usr/bin/env python3

import numpy as np
import open3d as o3d
import copy
import time
#from scipy.ndimage import minimum_filter
from scipy.signal import find_peaks,argrelmax

flatten = lambda x: [z for y in x for z in (flatten(y) if hasattr(y, '__iter__') else (y,))]

def pmap(data):
  res=[]
  start=-1
  for n,x in enumerate(data):
    if start<0:
      if x>10:
        start=n
    elif x<10:
      res.append((start,data[start:n]))
      start=-1
  return res

def peaks_min(image, crossing=False):
  if len(image)==0: return []
  data=image-np.max(image)
  data=-data
  xmap=[]
  ymap=[]
  for d in data.T:
    ymap.append(flatten(list(map(lambda p:np.asarray(find_peaks(p[1])[0])+p[0],pmap(d)))))
#    ymap.append(flatten(find_peaks(d)))
#  print("peaks_min ymap",ymap)
  psel=[]
  if crossing:
    for d in data:
#      print("peaks_min xarray",pmap(d))
#      xmap.append(flatten(list(map(lambda p:np.asarray(find_peaks(p[1])[0])+p[0],pmap(d)))))
      xmap.append(flatten(find_peaks(d)))
#    print("peaks_min xmap",xmap)
    for j,l in enumerate(ymap):
      for i in l:
        dist=100
        for k in xmap[i-1]+xmap[i]+xmap[i+1]:
          d=abs(j-k)
          if dist>d:
            dist=d
        if dist<=1: psel.append([i,j])
  else:
    for j,l in enumerate(ymap):
      for i in l:
        psel.append([i,j])
  return psel

def check(pc,xlst,ylst,zlst,width,thres):
  xmap=[]
  ymap=[]
  zmap=[]
  for x in xlst:
    pc1=pc[ np.ravel(np.abs(pc.T[0]-x)<width/2) ]
    for z in zlst:
      pc2=pc1[ np.ravel(np.abs(pc1.T[2]-z)<width/2) ]
      py2=np.ravel(pc2.T[1])
      py2=py2[py2.argsort()]
      xmap.append(int(x))
      zmap.append(int(z))
      if len(py2)<thres:
        ymap.append(int(ylst[-1]))
      else:
        py2=py2[:thres]
        ymap.append(int(py2[-1]))
  nm=(len(xlst),len(zlst))
  return np.array(xmap).reshape(nm),np.array(ymap).reshape(nm),np.array(zmap).reshape(nm)

def crop(pc,org):
  pc1=pc[ np.ravel(pc.T[0]-org[0])>mastercrop[0][0] ]
  pc2=pc1[ np.ravel(pc1.T[0]-org[0])<mastercrop[1][0] ]
  pc3=pc2[ np.ravel(pc2.T[1]-org[1])>mastercrop[0][1] ]
  pc4=pc3[ np.ravel(pc3.T[1]-org[1])<mastercrop[1][1] ]
  pc5=pc4[ np.ravel(pc4.T[2]-org[2])>mastercrop[0][2] ]
  pc6=pc5[ np.ravel(pc5.T[2]-org[2])<mastercrop[1][2] ]
  return pc6

def evaluate(pc,org,thres,radius=0):
#  pc1=crop(pc,org)
  if len(pc)<100:
    return 0,9.99,np.eye(4)
  pcd=o3d.geometry.PointCloud()
  pcd.points=o3d.utility.Vector3dVector(pc)
  rt=np.eye(4)
  rt[:3,3]=org
  thres2=thres*3
  rule=o3d.pipelines.registration.TransformationEstimationPointToPoint()
  result=o3d.pipelines.registration.registration_icp(masterpcd,pcd,thres2,rt,rule)
  result=o3d.pipelines.registration.evaluate_registration(masterpcd,pcd,thres,result.transformation)
  if radius>0:
    rule=o3d.pipelines.registration.TransformationEstimationPointToPlane()
    o3d.geometry.PointCloud.estimate_normals(pcd,o3d.geometry.KDTreeSearchParamRadius(radius=radius))
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamRadius(radius=radius))
    result2=o3d.pipelines.registration.registration_icp(masterpcd,pcd,thres2,result.transformation,rule)
    result2=o3d.pipelines.registration.evaluate_registration(masterpcd,pcd,thres,result2.transformation)
    if result.fitness<result2.fitness: result=result2
  return result.fitness,result.inlier_rmse,result.transformation

def learn(pc,fat=30):
  global masterpcd,mastercrop
  masterpcd=o3d.geometry.PointCloud()
  masterpcd.points=o3d.utility.Vector3dVector(pc)
  mins=np.array(list(map(np.min,pc.T)))-fat
  maxs=np.array(list(map(np.max,pc.T)))+fat
  mastercrop=[mins,maxs]
  print("mapper::learn",mastercrop)

def remesh(pc,mesh=0):
  if mesh==0: return pc
  pcd=o3d.geometry.PointCloud()
  pcd.points=o3d.utility.Vector3dVector(pc)
  pcd=pcd.voxel_down_sample(voxel_size=mesh)
  return np.array(pcd.points)

