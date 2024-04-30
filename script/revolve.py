#!/usr/bin/python

import sys; import numpy as np; from scipy.spatial.transform import Rotation as R
import rospy; import roslib;import tf; import tf2_ros; from std_msgs.msg import Bool; from std_msgs.msg import String; from geometry_msgs.msg  import Transform; from geometry_msgs.msg import TransformStamped
from rovi_utils import tflib

Param={
  "enable":0,
  "deg":0
}

def lookup(a,b):
  try:
    sys.stdout.write("//lookup "+a+" "+b+"\n")
    sys.stdout.flush()
    aTb=tfBuffer.lookup_transform(a,b,rospy.Time(0))
  except Exception as e:
    sys.stdout.write("//lookup exception\n")
    sys.stdout.flush()
    raise Exception(e)
    return
  return aTb

def chkrev(msg):
  rtb=lookup("camera/capture0/solve0/revolve","base")
  jtm=lookup("camera/master0/journal","camera/master0")
  rTb=tflib.toRT(rtb.transform)
  jTm=tflib.toRT(jtm.transform)
  lTb=jTm.dot(rTb)
  lbz=np.ravel(lTb[:3,2])
  lbz[0]=0
  lbz=lbz/np.linalg.norm(lbz)
  theta2=np.arcsin(np.cross(np.array([0,0,-1]),lbz)[0])
  print("revolve::theta2",theta2)

def cb_revolve(msg):
  try:
    Param.update(rospy.get_param("/revolver"))
  except Exception as e:
    print("revolver::get_param exception:",e.args)
  print("revolve::enable",Param["enable"])
  if Param["enable"]==0:
    return
  try:
    mtj=lookup("camera/master0","camera/master0/journal")
    jtb=lookup("camera/master0/journal","base")
    bts=lookup("base","camera/capture0/solve0")
  except Exception as e:
    print("revolve::lookup failed")
    return 
  bTs=tflib.toRT(bts.transform)
  mTj=tflib.toRT(mtj.transform)
  jTb=tflib.toRT(jtb.transform)
  jbz=np.ravel(jTb[:3,2])
  jbz[0]=0  #project to YZ plane of coord J
  jbz=jbz/np.linalg.norm(jbz)
  theta0=np.arcsin(np.cross(np.array([0,0,-1]),jbz)[0])
  print("revolve::theta0",theta0)
  kTb=mTj.I.dot(bTs.I)
  kbz=np.ravel(kTb[:3,2])
  kbz[0]=0  #project to YZ plane of coord K
  kbz=kbz/np.linalg.norm(kbz)
  theta1=np.arcsin(np.cross(np.array([0,0,-1]),kbz)[0])
  print("revolve::theta1",theta1)
  kTl=np.eye(4)
  kTl[:3,:3]=R.from_euler('X',-theta0+theta1+np.deg2rad(Param["deg"])).as_matrix()
  kta=TransformStamped()
  kta.header.frame_id="camera/capture0/solve0/journal"
  kta.child_frame_id="camera/capture0/solve0/jalign"
  kta.header.stamp=rospy.Time.now()
  kta.transform=tflib.fromRT(kTl)
  atr=TransformStamped()
  atr.header.frame_id="camera/capture0/solve0/jalign"
  atr.child_frame_id="camera/capture0/solve0/revolve"
  atr.header.stamp=rospy.Time.now()
  atr.transform=tflib.fromRT(mTj.I)
  stk=TransformStamped()
  stk.header.frame_id="camera/capture0/solve0"
  stk.child_frame_id="camera/capture0/solve0/journal"
  stk.header.stamp=rospy.Time.now()
  stk.transform=tflib.fromRT(mTj)
  broadcaster.sendTransform([stk,kta,atr])

def cb_do(msg):
  global Param
  if msg.data is False:
    pub_done.publish(mFalse)
    return
  try:
    Param.update(rospy.get_param("/revolver"))
  except Exception as e:
    print("revolver::get_param exception:",e.args)
  if Param["enable"]==0:
    bts=lookup("base","camera/capture0/solve0")
    ktr=TransformStamped()
    ktr.header.frame_id="camera/capture0/solve0"
    ktr.child_frame_id="camera/capture0/solve0/revolve"
    ktr.header.stamp=rospy.Time.now()
    ktr.transform=tflib.fromRT(np.eye(4))
    broadcaster.sendTransform([ktr])
    rospy.Timer(rospy.Duration(0.1),lambda event: pub_done.publish(mTrue),oneshot=True)
  else:
    cb_revolve(msg)
    rospy.Timer(rospy.Duration(0.1),lambda event: pub_done.publish(mTrue),oneshot=True)
    rospy.Timer(rospy.Duration(1),chkrev,oneshot=True)

########################################################
rospy.init_node("revolver",anonymous=True)
#Config.update(parse_argv(sys.argv))
#try:
#  Config.update(rospy.get_param("~config"))
#except Exception as e:
#  print("get_param exception:",e.args)
#try:
#  Param.update(rospy.get_param("~param"))
#except Exception as e:

###Topics Service
rospy.Subscriber("~do",Bool,cb_do)
pub_done=rospy.Publisher("~done",Bool,queue_size=1)

###Globals
mTrue=Bool();mTrue.data=True
mFalse=Bool();mFalse.data=False
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)
broadcaster=tf2_ros.StaticTransformBroadcaster()

try:
  rospy.spin()
except KeyboardInterrupt:
  print("Shutting down")
