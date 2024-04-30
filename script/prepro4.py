#!/usr/bin/env python3

import numpy as np
import scipy
import roslib
import rospy
import tf
import tf2_ros
import time
from scipy.spatial.transform import Rotation as R
from rovi.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from rovi_utils import tflib
import f_mapper as mapper
import bucket_solver as b_solver

Param={
  "enable":0,
  "mesh":0,
  "box0_width":60,
  "box0_points":300,
  "box0_step":10,
  "box0_crop":150,  #horizontal margin
  "box0_vcrop":50,  #vertial margin
  "peak_length":500,  #shaft length
  "bucket_height":0,
  "bucket_width":900,
  "bucket_yclip":-200,
  "ss_dist":500,      #Distance from camera to bottom of V plane
  "ss_thick":200,
  "ss_ext":50,
  "peak_fitness":0.5,  #fitness lower limit
  "peak_rmse":1.5,     #rmse upper limit
  "peak_tolerance":30, #initial pose tolerance
  "peak_threshold":3,  #icp threshold
  "peak_surpress":60,  #peak surpress angle
  "capt_angle":30,     #1/2 capture angle
  "capt_center":40,    #1/2 capture center X offset
  "level_points":1000, #top level finder
  "level_crop":30,     #horizontal crop
  "level_vcrop":30,    #vertical crop
  "view_crop":150,  #view horizontal crop
}
Config={
  "axis_frame_id":"prepro",
  "camera_frame_id":"camera",
  "cap0_frame_id":"camera/capture0",
  "cap1_frame_id":"camera/capture1",
  "master_frame_id":"camera/master0",
  "journal_frame_id":"camera/master0/journal",
  "base_frame_id":"base",
  "icp_threshold":"/searcher/icp_threshold",
  "normal_radius":"/searcher/normal_radius",
  "near_dist":10.0,    #candidate number
}

P0=np.array([]).reshape((-1,3))
Scene=P0
Scene1=None     #Scene of mode 1 operation
Master=P0

def np2F(d):  #numpy to Floats
  f=Floats()
  f.data=np.ravel(d)
  return f

def Pnul():
  return np.reshape([],(-1,3))

def getRT(base,ref):
  try:
    ts=tfBuffer.lookup_transform(base,ref,rospy.Time())
    RT=tflib.toRT(ts.transform)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    RT=None
  return RT

def pTr(RT,pc):
  if len(RT)<4:
    R4=np.eye(4)
    R4[:3,:3]=RT
  else: R4=RT
  return np.dot(R4[:3],np.vstack((pc.T,np.ones((1,len(pc)))))).T

def pubPose(coord,publisher):
  parray=PoseArray()
  parray.header.frame_id=Config['axis_frame_id']
  if len(coord)>0:
    if len(np.ravel(coord[0]))==3:
      for t in coord:
        p=Pose()
        p.position.x=t[0]
        p.position.y=t[1]
        p.position.z=t[2]
        p.orientation.x=0
        p.orientation.y=0
        p.orientation.z=-0.707
        p.orientation.w=0.707
        parray.poses.append(p)
    else:
      for t in coord:
        p=Pose()
        ta=np.eye(4)   #Arrow vector
        ta[:3,:3]=R.from_euler('Z',[90],degrees=True).as_matrix()
        tr=tflib.fromRT(t.dot(ta))
        p.position.x=tr.translation.x
        p.position.y=tr.translation.y
        p.position.z=tr.translation.z
        p.orientation.x=tr.rotation.x
        p.orientation.y=tr.rotation.y
        p.orientation.z=tr.rotation.z
        p.orientation.w=tr.rotation.w
        parray.poses.append(p)
  exec(publisher+'.publish(parray)')

def pubVplane(id,x,y,z,rx,ry,delay=0.1):
  RT=np.eye(4)
  RT[:3,:3]=R.from_euler('XY',[rx,ry],degrees=True).as_matrix()
  RT[0,3]=x
  RT[1,3]=y
  RT[2,3]=z
  tr=TransformStamped()
  tr.header.frame_id=''
  tr.child_frame_id=id
  tr.transform=tflib.fromRT(RT)
  rospy.Timer(rospy.Duration(delay),lambda ev:pub_axis.publish(tr),oneshot=True)
  return RT[:3,:3]

def pubReport(report):
  global Scene
  if context==1 or context==2:
    if len(report)>0:
      msg=String()
      msg.data=str(report)
      pub_str.publish(msg)
  if context==1: rospy.Timer(rospy.Duration(0.2),lambda ev: pub_capture.publish(mTrue),oneshot=True)

def pMap(pc,zlim=True):
  st=Param["box0_step"]
  th=Param["box0_points"]
  sz=Param["box0_width"]
  if len(pc)<Param["box0_points"]:
    print("prepro::pMap",len(pc))
    return [],[],[]
  xdiv=np.arange(np.min(pc.T[0])-sz,np.max(pc.T[0])+sz,st)
  ydiv=np.array([0,999])
  if not zlim:
    zdiv=np.arange(np.min(pc.T[2])-sz,np.max(pc.T[2])+sz,st)
#    print("prepro::pMap::zdiv",zdiv)
  else:
    zdiv=np.arange(Param["ss_dist"]-Param["ss_thick"],np.max(pc.T[2])+sz,st)
  xmap,ymap,zmap=mapper.check(pc,xdiv,ydiv,zdiv,sz,th)
#  print("pMap::xmap",xmap)
#  print("pMap::ymap",ymap)
#  print("pMap::zmap",zmap)
  imk=np.array(ymap)<999-10
  xmk=np.array(xmap)[imk]
  ymk=np.array(ymap)[imk]
  zmk=np.array(zmap)[imk]
  xb=np.ravel(xmk)
  yb=np.ravel(ymk)
  zb=np.ravel(zmk)
  pubPose(np.array([xb,yb,zb]).T,'pub_contours')
  return (xmap,ymap,zmap),(xb,yb,zb)

def pCollectPeakX(pc):
  cmap,_=pMap(pc)
  xcod=[]
  ycod=[]
  zcod=[]
  peaks=mapper.peaks_min(cmap[1],crossing=False)   #search peaks along Y-coord
  for i,j in peaks:
    xcod.append(cmap[0][i,j])
    ycod.append(cmap[1][i,j])
    zcod.append(cmap[2][i,j])
# convolve doubled peaks
  pcod=np.array([xcod,ycod,zcod]).T
  scod=[-1]*len(pcod)
  gcod=[]
  kcod=[]
  for i,p in enumerate(pcod):
    if scod[i]<0:
      gcod.append(len(gcod))
      scod[i]=gcod[-1]
      nlst=[p]
      for j,q in enumerate(pcod):
        if scod[j]<0:
          ds=np.linalg.norm(np.array(nlst-q).T,axis=0)
          if np.min(ds)<Param["box0_width"]:
#          if np.min(ds)<Param["box0_step"]*2:
            scod[j]=gcod[-1]
            nlst.append(q)
  kcod=[]
  for i in gcod:
    pp=pcod[np.array(scod)==i]
    kcod.append(pp[np.argmin(pp.T[1])])   #pick y min
#    kcod.append(np.mean(pp,axis=0))   #pick mean
# check peaks by ICP
  thres=Param["peak_threshold"]
  tcod=[]
  nrad=rospy.get_param(Config["normal_radius"])
  kcod=np.array(kcod)
  print("peak con",len(pcod),len(kcod))
  for p in kcod:
    if thres>0:
      pcro=mapper.crop(pc,p)
      fit,rmse,rt=mapper.evaluate(pcro,p,thres,radius=0)
      if fit>Param["peak_fitness"] and rmse<Param["peak_rmse"]:
        tcod.append(rt)
        for n,t in enumerate(tcod[:-1]):
          if np.sqrt((rt[0,3]-t[0,3])**2+(rt[2,3]-t[2,3])**2)<Param["box0_width"]:
            if rt[1,3]<t[1,3]: tcod[n]=rt
            tcod.pop()
            break
    else:
      rt=np.eye(4)
      rt[:3,3]=p
      tcod.append(rt)
  tcod=np.array(tcod)
  print("peak icp",len(tcod))
  fmap=[]
  rmap=[]
  tmap=[]
  if len(tcod)>0:
    for i,rt in enumerate(tcod):
      tmap.append(rt)
      fmap.append(rt.T[3,:3])
      rmap.append(pTr(rt,np.array([[0,Param["peak_length"],0]]))[0])
    fmap=np.array(fmap).T
    rmap=np.array(rmap).T
    xsort=rmap[0].argsort()   #sort along X-coord of R ends
    fmap=fmap.T[xsort].T
    rmap=rmap.T[xsort].T
    tmap=np.array(tmap)[xsort]
    pubPose(tmap,'pub_peaks')
  return fmap,rmap,tmap

def stickL(p):  #stick condition left
  return p[0]<view_border[0] and (not slipL(p))
def stickR(p):  #stick condition right
  return p[0]>view_border[1] and (not slipR(p))
def slipL(p):  #slip condition left
  return p[0]<bucket_border[0]
def slipR(p):  #slip condition right
  return p[0]>bucket_border[1]
def slippy(p):
  return slipL(p) or slipR(p)

def stickAll(l,f,r):
  for k in list(range(len(l))):
    if stickL(f[k]) or stickL(r[k]): l[k]=False
    if stickR(f[k]) or stickR(r[k]): l[k]=False

def vsDefeat(p1,p2,q1,q2):
  d1=np.abs(p1[0]-q1[0])
  d2=np.abs(p2[0]-q2[0])
  if d1==0 or d2==0:
    return 0
  elif d1<Param["box0_crop"] or d2<Param["box0_crop"]:
    h1=q1[2]-p1[2]
#    f1=-h1*np.exp(-d1/Param["box0_crop"])
    f1=-h1*Param["box0_crop"]/d1
    h2=q2[2]-p2[2]
#    g1=-h2*np.exp(-d2/Param["box0_crop"])
    g1=-h2*Param["box0_crop"]/d2
    return f1+g1
  else:
    return 0

def listDefeat(coord1,coord2):
  cod1=np.array(coord1)
  cod2=np.array(coord2)
  lvlst=[]
  for n in range(len(cod1)):
    lvl=np.array(list(map(lambda q:vsDefeat(cod1[n],cod2[n],q[0],q[1]),zip(cod1,cod2))))
    print("lvl",lvl)
    if sum(lvl<-Param["box0_vcrop"])==0: lvlst.append(-1)
    else: lvlst.append(np.argmin(lvl))
  return np.array(lvlst)

def zLift(p,coord):
  b1=coord.T[0]<p[0]
  b2=coord.T[0]>p[0]
  b3=np.abs(coord.T[0]-p[0])<Param["box0_crop"]
  cod=coord[b1+b2]
  return Param["ss_dist"] if len(cod)==0 else max(np.max(cod.T[2]),Param["ss_dist"])

def zSelect(coord1,coord2,zlim,strict=False):
  zsel=(coord1.T[2]>zlim)+(coord2.T[2]>zlim)
  if sum(zsel)==0: return np.array([False]*len(coord1))
  if not strict:
    zmin=min(np.min(coord1[zsel]),np.min(coord2[zsel]))
    zsel=(coord1.T[2]>zmin)+(coord2.T[2]>zmin)
#  return zsel
  zsel1=np.array(zsel)
  for f,c in zip(zsel,coord1):
    if f:
      dx=np.abs(coord1.T[0]-c[0])*np.tan(np.deg2rad(Param["peak_surpress"]))
      dz=coord1.T[2]-c[2]
      zsel1=((dz>=0)+(dx+dz>=0))*zsel1
  zsel2=np.array(zsel)
  for f,c in zip(zsel,coord2):
    if f:
      dx=np.abs(coord2.T[0]-c[0])*np.tan(np.deg2rad(Param["peak_surpress"]))
      dz=coord2.T[2]-c[2]
      zsel2=((dz>=0)+(dx+dz>=0))*zsel2
  return zsel1+zsel2

def isPoseClose(coord1,coord2):
  if (coord1 is None): return False
  if (coord2 is None): return False
  print("isPoseClose::pose dist.",np.linalg.norm(coord1[0:3,3]-coord2[0:3,3]))
  return np.linalg.norm(coord1[0:3,3]-coord2[0:3,3])<Config["near_dist"]

def pScanX(pc,uTc):
  global bucket_border,view_border,cand_pose
  bucket_border=np.array([Param["box0_crop"],Param["bucket_width"]-Param["box0_crop"]])
# datermine V-plane
  uZ=Param["ss_dist"]  #bottom z of Vplane
# search peaks
  fmap,rmap,tmap=pCollectPeakX(pc)
  if len(fmap)==0:
    print("pScanX::no F-end")
    return 0,0,0,0,0,0
  zmax=max(np.max(fmap[2]),np.max(rmap[2]))
  if zmax<(uZ if capt_pos==0 else uZ-Param["ss_ext"]):
    print("pScanX::no above z-slice")
    cx=np.mean(capt_border)
    pubVplane(Config['axis_frame_id']+'V1',cx,0,uZ,0,0,delay=0.1)
    pubVplane(Config['axis_frame_id']+'V2',cx,0,uZ,0,0,delay=0.2)
    return 0,0,0,0,0,0
  print("sliceZ",uZ)
  print("bucket_border",bucket_border)
  print("capt_border",capt_border)
  view_border=capt_border+np.array([Param["view_crop"],-Param["view_crop"]])
  pubVplane(Config['axis_frame_id']+'V1',view_border[0],0,uZ,0,90,delay=0.1)
  pubVplane(Config['axis_frame_id']+'V2',view_border[1],0,uZ,0,-90,delay=0.2)
  zrange=min(np.max(fmap[2]),np.max(rmap[2]))-Param["ss_thick"]
#  nsel=(fmap[2]>=zrange)+(rmap[2]>=zrange)
  nsel=zSelect(fmap.T,rmap.T,zrange)
  rmapt=rmap.T[nsel]  #close to the top
  fmapt=fmap.T[nsel]  #close to the top
  tmap=tmap[nsel]
  xsort=np.argsort(fmapt.T[0])
  fmapt=fmapt[xsort]
  rmapt=rmapt[xsort]
  tmap=tmap[xsort]
  smap=listDefeat(rmapt,fmapt)
  kmap=smap<0
  print("def list",smap)
  if len(smap)==1:
    f=fmapt[0]
    r=rmapt[0]
    if not kmap[0]:
      print("pScanX::return 1 candidate untouchable")
      return 0,0,0,0,0,0
    elif stickL(f) or stickL(r):
      print("pScanX::return 1 recapt left",smap[0])
      if (f[2]>uZ)+(r[2]>uZ):
        pubPose([tmap[0]],'pub_apos')
        if (capt_num==1): cand_pose=tmap[0]
        if capt_pos==0: return 0,0,0,0,0,0       #If capture center
        return 1,view_border[0],0,0,0,0
      else:
        return 0,0,0,0,0,0
    elif stickR(f) or stickR(r):
      print("pScanX::return 1 recapt right",smap[0])
      if (f[2]>uZ)+(r[2]>uZ):
        pubPose([tmap[0]],'pub_apos')
        if (capt_num==1): cand_pose=tmap[0]
        if capt_pos==0: return 0,0,0,0,0,0       #If capture center
        return 1,view_border[1],0,0,0,0
      else:
        return 0,0,0,0,0,0
    else:
      ztop=max(zLift(f,fmap.T),zLift(r,rmap.T))
      print("pScanX::return 1 success",ztop)
      pubPose([tmap[0]],'pub_apos')
      if (capt_num==1): cand_pose=tmap[0]
      return 1,f[0],f[1],f[2],r[2]-f[2],ztop-max(r[2],f[2])
  else:
    lsel=np.array([True]*len(kmap))  #False on view border
#    if stickL(fmapt[0]) or stickL(rmapt[0]): lsel[0]=False
#    if stickR(fmapt[-1]) or stickR(rmapt[-1]): lsel[-1]=False
    stickAll(lsel,fmapt,rmapt)
    zsel=zSelect(fmapt,rmapt,uZ,strict=True)
    fmapz=fmapt[zsel]
    rmapz=rmapt[zsel]
    tmapz=tmap[zsel]
    kmapz=kmap[zsel]
    lselz=lsel[zsel]
    kmapl=kmapz[lselz]
    print("def list(>uZ)",kmap,zsel,kmapz,lsel,kmapl)
    k1len=len(kmapz)
    z2sel=zSelect(fmapt,rmapt,uZ-Param["ss_ext"],strict=True)
    k2len=len(kmap[z2sel])
    klen=k1len if capt_pos==0 else k2len
    if k1len>0:
      if sum(kmapl)>0:
        fmapl=fmapz[lselz]
        rmapl=rmapz[lselz]
        tmapl=tmapz[lselz]
        zsort=np.argsort(fmapl.T[2])[::-1]
        kmapl=kmapl[zsort]
        tmapl=tmapl[zsort]
        fmapl=fmapl[zsort]
        rmapl=rmapl[zsort]
        f=fmapl[kmapl][0]
        r=rmapl[kmapl][0]
        ztop=max(zLift(f,fmap.T),zLift(r,rmap.T))
        print("pScanX::return success",ztop)
        pubPose([tmapl[kmapl][0]],'pub_apos')
        # check near or far
        if isPoseClose(cand_pose,tmapl[kmapl][0]): klen=1; print("pScanX::force return 1")
        print(cand_pose,tmapl[kmapl][0],isPoseClose(cand_pose,tmapl[kmapl][0]),klen)
        if capt_pos==0:          #If capture center
          klen=sum(lselz)
        return klen,f[0],f[1],f[2],r[2]-f[2],ztop-max(r[2],f[2])
      else:
        zsort=np.argsort(fmapz.T[2])[::-1]
        print("pScanX::return recapt",zsort,lselz)
        if capt_pos==0:          #If capture center
          if sum(lselz)!=1:
            klen=sum(lselz)
        for l,f,tr in zip(lselz[zsort],fmapz[zsort],tmapz[zsort]):
          if not l:
            pubPose([tr],'pub_apos')
            return klen,f[0],0,0,0,0
        return 0,0,0,0,0,0
    elif capt_pos==0:
      print("pScanX::return recapt 0")
      return 0,0,0,0,0,0
    else:
      fmapz=fmapt[z2sel]
      rmapz=rmapt[z2sel]
      tmapz=tmap[z2sel]
      kmapz=kmap[z2sel]
      lselz=lsel[z2sel]
      kmapl=kmapz[lselz]
      k2len=len(kmapz)-sum(lselz^True)
      print("def list(>uZ+a)",kmap,lsel,kmapz,lselz)
      if sum(kmapl)>0:
        fmapl=fmapz[lselz]
        rmapl=rmapz[lselz]
        tmapl=tmapz[lselz]
        zsort=np.argsort(fmapl.T[2])[::-1]
        kmapl=kmapl[zsort]
        tmapl=tmapl[zsort]
        fmapl=fmapl[zsort]
        rmapl=rmapl[zsort]
        f=fmapl[kmapl][0]
        r=rmapl[kmapl][0]
        ztop=max(zLift(f,fmap.T),zLift(r,rmap.T))
        print("pScanX::return2 success",ztop)
        pubPose([tmapl[kmapl][0]],'pub_apos')
        # check near or far
        if isPoseClose(cand_pose,tmapl[kmapl][0]): k2len=1; print("pScanX::force return 1")
        print(cand_pose,tmapl[kmapl][0],isPoseClose(cand_pose,tmapl[kmapl][0]),k2len)
        return k2len,f[0],f[1],f[2],r[2]-f[2],ztop-max(r[2],f[2])
      else:
        print("pScanX::return2 recapt neither")
        return 0,0,0,0,0,0

def pubMaster(uTc,pc):
  global mTf
  upc=pTr(uTc,pc)
#  pub_master.publish(np2F(upc))
  cmap,_=pMap(upc,zlim=False)
#  print("pubMaster::cmap[1]",cmap[1])
  peaks=mapper.peaks_min(cmap[1],crossing=True)
  cod=[[],[],[]]
  for i,j in peaks:
    cod[0].append(cmap[0][i,j])
    cod[1].append(cmap[1][i,j])
    cod[2].append(cmap[2][i,j])
  org=np.mean(np.array(cod),axis=1)
  uTf=np.eye(4)
  uTf[0,3]=org[0]
  uTf[1,3]=org[1]
  uTf[2,3]=org[2]
  fpc=pTr(np.linalg.inv(uTf),upc)
  pub_master.publish(np2F(fpc))
  mapper.learn(fpc,fat=Param["peak_tolerance"])
  mTf=np.linalg.inv(uTc).dot(uTf)

pubSceneMutex={}
def pubScene(pc,immediate=False):
  global pubSceneMutex
  print("prepro::pubScene",pubSceneMutex,len(pc))
  if immediate or (not "lock" in pubSceneMutex):
    pubSceneMutex["lock"]=True
    pub_ps.publish(np2F(pc))
    rospy.Timer(rospy.Duration(1),lambda ev:pubSceneMutex.clear(),oneshot=True)

def prepro():
  global Param,Scene,uCrop,uTc,uKvec
  print("prepro::prepro",len(Scene))
  pubScene(Scene)
  if len(Scene)<Param["box0_points"]:
    print("prepro::prepro Scene few",len(Scene))
    pubReport({'probables':0,'prob_m':0,'prob_x':0,'prob_z':-1})
    return
  if Param["enable"]==1:    #search F-end
    uTc=getRT(Config["axis_frame_id"],Config["cap0_frame_id"])
    cTu=np.linalg.inv(uTc)
    uscn=pTr(uTc,Scene)
#    pubMaster(uTc,Master)
    uTj=np.array([[0,-1,0,0],[-1,0,0,0],[0,0,-1,0],[0,0,0,1]])  #bucket=>journal(rotation)
    jTm=getRT(Config["journal_frame_id"],Config["master_frame_id"])
    pubMaster(uTj.dot(jTm),Master)
    if len(uscn)<Param["box0_points"]:
      print("prepro::prepro uscn few",len(uscn))
      pubVplane(Config['axis_frame_id']+'V1',0,0,Param["ss_dist"],0,0,delay=0.1)
      pubVplane(Config['axis_frame_id']+'V2',0,0,Param["ss_dist"],0,0,delay=0.2)
      pubReport({'probables':0,'prob_m':0,'prob_x':0,'prob_z':-2})
      return
    t0=time.time()
    pbn,cx0,cy0,cz0,lz0,bz0=pScanX(uscn,uTc)  #Scan PC peak of F-end of workpiece
    t1=time.time()-t0
    print("prepro::prepro::pScanX done",pbn,cx0,cy0,cz0,lz0,bz0,t1)
    report={}
    report["tfeat"]=t1
    report["probables"]=pbn
    report["prob_x"]=lz0
    report["prob_z"]=bz0
    if pbn>0: #probable point clusters
      xlen=Param["bucket_width"]
      margin=cx0 if cx0<xlen/2 else cx0-xlen
      report["prob_m"]=margin
      uKvec=np.array([cx0,cy0,cz0])
      uCrop=mapper.crop(uscn,uKvec)
      print("prepro::prepro::crop",len(uCrop))
      if len(uCrop)<len(Master)*Param["peak_fitness"]:
        print("prepro::prepro::few points",len(Master))
        uKvec=None  #few points
    else:
      print("prepro::prepro None")
#      report["prob_m"]=Param["bucket_width"]/2;
      bTc=getRT(Config['base_frame_id'],Config['cap0_frame_id'])
      bTu=getRT(Config['base_frame_id'],Config['axis_frame_id'])
      report=b_solver.capture(bTc,bTu,Config,Param,Scene)
      if report["probables"]==0:  #failed to detect top layer
        report["prob_m"]=Param["ss_dist"]
      else:
        report["probables"]=0
      uKvec=None
      uCrop=Pnul()
    pubReport(report)
    return
  elif Param["enable"]==2:    #search Top layer
    bTc=getRT(Config['base_frame_id'],Config['cap0_frame_id'])
    bTu=getRT(Config['base_frame_id'],Config['axis_frame_id'])
    report=b_solver.capture(bTc,bTu,Config,Param,Scene)
    pubVplane(Config['axis_frame_id']+'V1',Param["box0_crop"],0,-300,0,90,delay=0.1)
    pubVplane(Config['axis_frame_id']+'V2',Param["bucket_width"]-Param["box0_crop"],0,-300,0,-90,delay=0.2)
    pubReport(report)
    return
  else:
    pubReport({})
    return

def pubScore(result,direct_transform=False):
  print("prepro::score",result)
  if "transform" in result: bTa=tflib.toRT(result["transform"])
  elif "RT" in result: bTa=result["RT"]
  else: bTa=np.eye(4)
  if direct_transform:
    cTm=getRT(Config["cap0_frame_id"],Config["master_frame_id"])
    mTb=getRT(Config["master_frame_id"],Config["base_frame_id"])
    tf=tflib.fromRT(cTm.dot(mTb).dot(bTa).dot(np.linalg.inv(mTb)))
  else: tf=tflib.fromRT(bTa)
  stats={
    "proc":[1],
    "fitness":[result["fitness"]],
    "Tx":[tf.translation.x],"Ty":[tf.translation.y],"Tz":[tf.translation.z],
    "Qx":[tf.rotation.x],"Qy":[tf.rotation.y],"Qz":[tf.rotation.z],"Qw":[tf.rotation.w]}
  score=Float32MultiArray()
  score.layout.data_offset=0
  for key in stats:
    score.layout.dim.append(MultiArrayDimension())
    score.layout.dim[-1].label=key
    score.layout.dim[-1].size=len(stats[key])
    score.layout.dim[-1].stride=1
    score.data.extend(stats[key])
  pub_score.publish(score)

def cb_capture(msg):
  global context,capt_border,capt_pos,capt_num
  context=1
  print("prepro::cb_capture")
  try:
    Param.update(rospy.get_param("/prepro"))
  except Exception as e:
    print("get_param exception:",e.args)
  uTc=getRT(Config["axis_frame_id"],Config["camera_frame_id"])
  cP=np.array([[Param["capt_center"],0,0]])    #camera center
  cV=np.array([[0,0,1]])    #camera Z axis
  cDs=[['y',Param["capt_angle"]],['y',-Param["capt_angle"]]]   #cV deviation list
  cVs=np.array(list(map(lambda d: pTr(np.linalg.inv(R.from_euler(d[0],d[1],degrees=True).as_matrix()),cV),cDs))).reshape((-1,3))
  cVs=cVs+cP
  uP=pTr(uTc,cP)
  uVs=pTr(uTc,cVs)-uP
  uZ=np.array([[0,0,Param["ss_dist"]]])  #bottom z of Vplane
  hs=(uZ-uP).T[2]
  ks=hs/uVs.T[2].T
  cs=(np.multiply(ks,uVs)+uP).T[0]
  capt_num=capt_num+1
  if capt_border is None:
    capt_border=np.array([np.min(cs),np.max(cs)])
    capt_pos=1 if capt_border[0]<0 else (-1 if capt_border[1]>Param["bucket_width"] else 0)
    print("cb_capture::no border",capt_border,capt_pos,capt_num)
  else:
    capt_border[0]=min(capt_border[0],np.min(cs))
    capt_border[1]=max(capt_border[1],np.max(cs))
    print("cb_capture::extend border",capt_border,capt_num)
  prepro()

def cb_clear(msg):
  global Scene,context,capt_border,capt_num,cand_pose
  capt_border=None
  capt_num=0
  context=0
  pubPose(np.array([]),'pub_contours')
  pubPose(np.array([]),'pub_peaks')
  pubPose(np.array([]),'pub_apos')
  cand_pose=None
  Scene=Pnul()
  print("prepro::cb_clear")
  prepro()

def cb_ps(msg):
  global Scene
  Scene=np.reshape(msg.data,(-1,3))
  print("prepro::cb_ps",len(Scene))
  pubScene(Scene)

def save_temp(startswith):
    try:
      yf=open(filename, "r")
    except:
      rospy.logwarn("ezui::open exception "+filename)
      return
    try:
      param=yaml.safe_load(yf)
    except:
      yf.close()
      rospy.logwarn("ezui::parser exception")
      return
    yf.close()
    try:
      yf=open(filename,"w")
      dictlib.cross(param,rtkWidget.Param)
      yaml.dump(param,yf,default_flow_style=False)
      rtkWidget.Origin=param
    except:
      rospy.logwarn("ezui::dump exception")
    yf.close()

def cb_solve(msg):
  global context
  context=2
  print("prepro::cb_solve",Param)
  if Param["enable"]==1:  #crop a boundary
    t0=time.time()
    cTu=np.linalg.inv(uTc)
#    pubScene(pTr(cTu,uCrop),immediate=True)
#    rospy.Timer(rospy.Duration(0.1),lambda ev: pub_solve.publish(mTrue),oneshot=True)
    if uKvec is not None:
      thres=rospy.get_param(Config["icp_threshold"])
      nrad=rospy.get_param(Config["normal_radius"])
      fit,rmse,uTf=mapper.evaluate(uCrop,uKvec,thres,radius=nrad)
      print("fit",fit,rmse,uTf)
      tr=cTu.dot(uTf).dot(np.linalg.inv(mTf))
      t1=time.time()-t0
      rospy.Timer(rospy.Duration(0.1),lambda ev:pubScore({'RT':tr,'fitness':fit}),oneshot=True)
      rospy.Timer(rospy.Duration(0.2),lambda ev:pubReport({'rmse':rmse,'tmatch':t1}),oneshot=True)
    else:
      pubScore({'fitness':0.111})
    return
  elif Param["enable"]==2:  #search bucket column
    Param["level_crop"]=Param["box0_crop"]
    Param["level_vcrop"]=Param["box0_vcrop"]
    Param["level_points"]=Param["peak_fitness"]
    bTc0=getRT(Config["base_frame_id"],Config["cap0_frame_id"])
    bTc1=getRT(Config["base_frame_id"],Config["cap1_frame_id"])
    if bTc1 is None:
      pubScore({'fitness':0})   #no 2nd capture
      return
    c1Tc0=getRT(Config["cap1_frame_id"],Config["cap0_frame_id"])
    delm0=np.linalg.norm(Scene.T[0:2].T,axis=1)
    delm1=np.linalg.norm(pTr(c1Tc0,Scene).T[0:2].T,axis=1)
    scn0=Scene[delm0<Param["box0_width"]]
    scn1=Scene[delm1<Param["box0_width"]]
    if len(scn0)<Param["box0_points"] or len(scn1)<Param["box0_points"]:
      pubScore({'fitness':0.1})   #not enough points
      return
    bscn0=pTr(bTc0,scn0)
    bscn1=pTr(bTc0,scn1)
#    bscn0=bscn0[ np.ravel(bscn0.T[2]>Param["bucket_height"]-50) ]
#    bscn1=bscn1[ np.ravel(bscn1.T[2]>Param["bucket_height"]-50) ]
    bscn=np.vstack((bscn0,bscn1))
    pubScene(pTr(np.linalg.inv(bTc0),bscn))
    if Param["box0_step"]>0:
      cog0=b_solver.pSliceZ(bscn0,Param["box0_step"],Param["box0_points"],top=True)
      cog1=b_solver.pSliceZ(bscn1,Param["box0_step"],Param["box0_points"],top=True)
    if (cog0 is None) or (cog1 is None):
      pubScore({'fitness':0.2})   #points on the planes too few
      return
    cog0[2]=(cog0[2]+cog1[2])/2
    cog1[2]=cog0[2]
    Param["bucket_height"]=int(cog0[2])
    rospy.set_param("/prepro/bucket_height",Param["bucket_height"])
    xvec=cog1-cog0
    xnorm=np.linalg.norm(xvec)
    if xnorm<Param["box0_width"]:
      pubScore({'fitness':0.4})   #cogs too close
      return
    Param["bucket_width"]=int(xnorm)
    rospy.set_param("/prepro/bucket_width",Param["bucket_width"])
    xbasis=xvec/xnorm
    zbasis=np.array([0,0,1])
    ybasis=np.cross(zbasis,xbasis)
    RTt=np.eye(4)
    RTt[0,:3]=xbasis
    RTt[1,:3]=ybasis
    RTt[2,:3]=zbasis
    RTt[3,:3]=cog0
    tr=TransformStamped()
    tr.header.frame_id=''
    tr.child_frame_id=Config['axis_frame_id']
    tr.transform=tflib.fromRT(RTt.T)
    pub_axis.publish(tr)
    rospy.Timer(rospy.Duration(0.1),lambda ev:pubScore({'transform':tr.transform,'fitness':1},direct_transform=True),oneshot=True)
    return
  else:  #bypass this node
    rospy.Timer(rospy.Duration(0.1),lambda ev: pub_solve.publish(mTrue),oneshot=True)

def cb_master(msg):
  global Master
  Master=np.reshape(msg.data,(-1,3))

def cb_redraw(msg):
  global context
  context=-1
  print("prepro::cb_redraw")
#  rospy.Timer(rospy.Duration(1),lambda ev:prepro(),oneshot=True)

########################################################
rospy.init_node("prepro",anonymous=True)
###Load params
try:
  Config.update(rospy.get_param("/config/prepro"))
except Exception as e:
  print("get_param exception:",e.args)
###Topics
rospy.Subscriber("~in/floats",numpy_msg(Floats),cb_ps)
rospy.Subscriber("~clear",Bool,cb_clear)  #clear
rospy.Subscriber("~capture",Bool,cb_capture)  #search cluster
rospy.Subscriber("~solve",Bool,cb_solve)
rospy.Subscriber("~redraw",Bool,cb_redraw)
rospy.Subscriber("/master/surface/floats",numpy_msg(Floats),cb_master)
pub_ps=rospy.Publisher("~out/floats",numpy_msg(Floats),queue_size=1)
pub_str=rospy.Publisher("/report",String,queue_size=1)
pub_capture=rospy.Publisher("~captured",Bool,queue_size=1)
pub_solve=rospy.Publisher("~solved",Bool,queue_size=1)
pub_score=rospy.Publisher("~score",Float32MultiArray,queue_size=1)
pub_apos=rospy.Publisher("/prepro/apos",PoseArray,queue_size=1)
pub_peaks=rospy.Publisher("/prepro/peaks",PoseArray,queue_size=1)
pub_contours=rospy.Publisher("/prepro/contour",PoseArray,queue_size=1)
pub_axis=rospy.Publisher("/update/config_tf",TransformStamped,queue_size=1)
pub_master=rospy.Publisher("/prepro/master/floats",numpy_msg(Floats),queue_size=1)
###Bool message
mTrue=Bool();mTrue.data=True
mFalse=Bool();mFalse.data=False
###TF
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)
broadcaster=tf2_ros.StaticTransformBroadcaster()

#globals
capt_border=None
capt_num=0
cand_pose=None

#if __name__=="__main__":

try:
  rospy.spin()
except KeyboardInterrupt:
  print("Shutting down")
