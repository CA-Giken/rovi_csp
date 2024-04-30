#!/usr/bin/env python3

# Python includes
import numpy as np
import sys

# ROS includes
import roslib
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import Vector3
from tf import transformations
from rviz_tools_py import rviz_tools
from rovi_utils import tflib

Config={
  "__name":"placer",
  "mesh":"package://rovi_csp/mesh/Bucket.stl",
  "color":"white",
  "alpha":0.33,
  "x0":None,
  "y0":None,
  "z0":None,
  "frame":"base"
}

def cleanup_node():  # Exit handler
  print("Shutting down node")
  markers.deleteAllMarkers()

def getRT(base,ref):
  try:
    ts=tfBuffer.lookup_transform(base,ref,rospy.Time())
    RT=tflib.toRT(ts.transform)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    rospy.logwarn("placer3d::TF lookup error")
    RT=None
  return RT

def updateConfig():
  global Config
  try:
    Config.update(rospy.get_param("/config/"+Config["__name"]))
  except Exception as e:
    print("get_param exception:",e.args)

def parse_argv(argv):
  args={}
  for arg in argv:
    tokens = arg.split(":=")
    if len(tokens) == 2:
      key = tokens[0]
      args[key]=tokens[1]
  return args

### Initialize the ROS Node ###
Config.update(parse_argv(sys.argv))
print("place3d::Config",Config)
rospy.init_node('marker_'+Config["__name"], anonymous=False, log_level=rospy.INFO, disable_signals=False)
rospy.on_shutdown(cleanup_node)

updateConfig()
markers=rviz_tools.RvizMarkers(Config["frame"],Config["__name"]+'_marker')

tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)
rospy.sleep(2)

markers.alpha=Config["alpha"]

while not rospy.is_shutdown():
  updateConfig()
  RT=getRT("world",Config["frame"])
  tr=np.array([0,0,0])
  for n,key in enumerate(["x0","y0","z0"]):
    elm=Config[key]
    if type(elm) is str: tr[n]=rospy.get_param(elm)-RT[n,3]
    elif type(elm) is int or type(elm) is float: tr[n]=elm-RT[n,3]
  T = transformations.translation_matrix(tr)
  scale = Vector3(1,1,1)
#  markers.alpha=0.2
  markers.publishMesh(T, Config["mesh"], Config["color"], scale, 0.5)
  rospy.Rate(5).sleep() #1 Hz
