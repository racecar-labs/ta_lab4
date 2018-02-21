#!/usr/bin/env python

import rospy 
import numpy as np
from nav_msgs.srv import GetMap
from ta_lab4.srv import *
from HaltonPlanner import HaltonPlanner
from HaltonEnvironment import HaltonEnvironment
import KinematicModel as model

PLANNER_SERVICE_TOPIC = '/planner_node/get_plan'

class PlannerNode(object):

  def __init__(self):
    map_service_name = rospy.get_param("~static_map", "static_map")
    print("Getting map from service: ", map_service_name)
    rospy.wait_for_service(map_service_name)
    
    graph_file = rospy.get_param("~graph_file", None)
    self.map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map
    self.environment = HaltonEnvironment(self.map_msg, graph_file, None, None)
    self.planner = HaltonPlanner(self.environment)
    self.server = rospy.Service(PLANNER_SERVICE_TOPIC, GetPlan, self.plan_cb)
    print 'Ready to plan'
    
  def plan_cb(self, req):
    # Check that target and source have correct dimension
    if len(req.source) != model.SPACE_DIM or len(req.target) != model.SPACE_DIM:
      return [[],False]
    
    # Check for source == target
    if req.source == req.target:
      result = []
      result.extend(req.source)
      result.extend(req.target)
      return [result, True]
      
    source = np.array(req.source).reshape(3)
    target = np.array(req.target).reshape(3)

    self.environment.set_source_and_target(source, target)
    
    # Check if planning is trivially infeasible on this environment
    if not self.environment.manager.get_state_validity(source):
      print 'Source in collision'
      return [[],False]

    if not self.environment.manager.get_state_validity(target):
      print 'Target in collision'
      return [[],False]
      
    plan = self.planner.plan()
    
    if plan:
      plan = self.planner.post_process(plan, 5)
      self.planner.simulate(plan)
      flat_plan = [el for config in plan for el in config]
      return [flat_plan, True]
    else:
      return [[],False]
    
if __name__ == '__main__':
  rospy.init_node('planner_node', anonymous=True)

  pn = PlannerNode()
  
  rospy.spin()  
