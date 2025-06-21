#!/usr/bin/env python3
 
import numpy as np
import sys
import os
import subprocess
 
import roslib
import rospy
from std_msgs.msg import Bool

devname="/sda2"
mTrue=Bool();mTrue.data=True

########################################################
rospy.init_node("diskfree",anonymous=True)
pub_df=rospy.Publisher('/diskfree/stat',Bool,queue_size=1)

while not rospy.is_shutdown():
  df=subprocess.getoutput("df")
  dfs=df.split('\n')
  dft=[l for l in dfs if devname in l][0]
  print("df",devname,dft)
  dfn=[s for s in dft.split() if '%' in s][0]  
  print("df used",dfn[:-1])
  if int(dfn[:-1])>90: pub_df.publish(mTrue)
  rospy.sleep(60)
