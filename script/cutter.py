#!/usr/bin/env python3
# 2021/03/17 hato #!/usr/bin/env python -> !/usr/bin/env python3

# Python includes
import numpy as np
import copy

# ROS includes
import roslib
import rospy
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon, Transform
from tf import transformations # rotation_matrix(), concatenate_matrices()
from rovi_utils import tflib
from rviz_tools_py import rviz_tools


# Initialize the ROS Node
rospy.init_node('cutter', anonymous=False, log_level=rospy.INFO, disable_signals=False)

# Define exit handler
def cleanup_node():
  print("Shutting down node")
  markers.deleteAllMarkers()

rospy.on_shutdown(cleanup_node)

markers = rviz_tools.RvizMarkers('camera/master0', 'cutter_marker')

Param={
  "refs":"/cropper/master",
  "base":-100,
  "offset":200,
  "width":10,
  "span":150,
  "distance":550,
  "align":"x"
}
Refs={}

color=(0.1,0.1,0.1)

while not rospy.is_shutdown():
  rospy.Rate(5).sleep() #1 Hz
  try:
    Param.update(rospy.get_param("/cutter"))
  except Exception as e:
    pass
  try:
    Refs=rospy.get_param(Param["refs"])
  except Exception as e:
    pass

  tr1=Transform()
  tbase=Param["base"] if "base" not in Refs else Refs["base"]
  if Refs["align"]=="y":
    tr1.rotation.x=np.sqrt(2)/2
    tr1.rotation.w=np.sqrt(2)/2
    tr1.translation.y=tbase
  else:
    tr1.rotation.y=np.sqrt(2)/2
    tr1.rotation.w=np.sqrt(2)/2
    tr1.translation.x=tbase
  tr1.translation.z=Param["distance"] if "distance" not in Refs else Refs["distance"]
  span=Param["span"] if "span" not in Refs else Refs["span"]
  cwid=Param["width"] if "width" not in Refs else Refs["width"]
  T1=tflib.toRT(tr1)
  if cwid>0: markers.publishPlane(T1,span,span,color,1.0) # pose, color, scale, lifetime
  tr2=copy.copy(tr1)
  offset=Param["offset"] if "offset" not in Refs else Refs["offset"]
  if Refs["align"]=="y":
    tr2.translation.y=tr2.translation.y+offset
  else:
    tr2.translation.x=tr2.translation.x+offset
  T2=tflib.toRT(tr2)
  if cwid>0: markers.publishPlane(T2,span,span,color,1.0)

