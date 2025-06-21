#!/usr/bin/env python3

import numpy as np
import open3d as o3d
import time

def pTr(RT,pc):
  if len(RT)<4:
    R4=np.eye(4)
    R4[:3,:3]=RT
  else: R4=RT
  return np.dot(R4[:3],np.vstack((pc.T,np.ones((1,len(pc)))))).T

def pSliceZ(pc,th,points,step=1,top=False):
  zmin=int(np.min(pc[:,2]))
  zmax=int(np.max(pc[:,2]))
  ps=list(map(lambda z: pc[np.ravel(np.abs(pc.T[2]-z)<th)],range(zmin,zmax,step)))
  if top:
    for pl in ps[::-1]:
      if len(pl)>points:
        print("prepro::pSliceZ top",np.argmax(pl,axis=0))
        return np.ravel(np.mean(pl,axis=0))
    return None
  else:
    pn=np.array(list(map(len,ps)))
    print("prepro::pSliceZ layer",np.argmax(pn))
    pl=ps[np.argmax(pn)]
    if len(pl)<points: return None
    return np.ravel(np.mean(pl,axis=0))

def capture(bTc,bTu,Config,Param,Scene):
  bscn=pTr(bTc,Scene)
  bscn=bscn[ np.ravel(bscn.T[2]<Param["bucket_height"]-50) ]
  uTb=np.linalg.inv(bTu)
  uscn=pTr(uTb,bscn)
#  uTc=uTb.dot(np.linalg.inv(bTc))
#  cTu=np.linalg.inv(uTc)
  uscn1=uscn[ np.ravel(uscn.T[1]>Param["bucket_yclip"]) ]
  uscn2=uscn1[ np.ravel(uscn1.T[0]>Param["level_crop"]) ]
  uscn3=uscn2[ np.ravel(uscn2.T[0]<Param["bucket_width"]-Param["level_crop"]) ]
  report={}
  if len(uscn3)<Param["box0_points"]:
    report["probables"]=0
    report["prob_m"]=0
    return report
  ez0=pSliceZ(uscn3,Param["level_vcrop"],Param["level_points"],step=10,top=True)  #Cluster edge for Z
  report["probables"]=0 if ez0 is None else 1
  report["prob_m"]=0 if ez0 is None else ez0[2]
  report["prob_x"]=0
  report["prob_z"]=0
  return report

