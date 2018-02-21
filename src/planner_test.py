#!/usr/bin/env python

import rospy 
import numpy as np
from ta_lab4.srv import *

PLANNER_SERVICE_TOPIC = '/planner_node/get_plan'
SOURCE = [13.76, 44.88, 0]
TARGET = [64,    11,    0]
#SOURCE = [62.72,  53.6, 0.0]
#TARGET = [17.52,  26.48,0.0]

if __name__ == '__main__':

  rospy.init_node('planner_test', anonymous=True)
  rospy.wait_for_service(PLANNER_SERVICE_TOPIC)
  get_plan = rospy.ServiceProxy(PLANNER_SERVICE_TOPIC, GetPlan)
  
  try:
    resp = get_plan(SOURCE,TARGET)
    print np.array(resp.plan).reshape(-1,3)
    print resp.success
  except rospy.ServiceException, e:
    print 'Service call failed: %s'%e
    

