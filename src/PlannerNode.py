#!/usr/bin/env python

import rospy 
import numpy as np
from nav_msgs.srv import GetMap
from ta_lab4.srv import *
from HaltonPlanner import HaltonPlanner
from HaltonEnvironment import HaltonEnvironment
import KinematicModel as model
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
import Utils

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
    
    '''
    pub = rospy.Publisher('node_test', PoseArray, queue_size=1)
    pa = PoseArray()
    pa.header.frame_id = "/map"
    for i in xrange(self.environment.graph.number_of_nodes()):
      config = self.environment.get_config(i)
      pose = Pose()
      pose.position.x = config[0]
      pose.position.y = config[1]
      pose.position.z = 0.0
      pose.orientation = Utils.angle_to_quaternion(config[2])
      pa.poses.append(pose)
    print 'viz nodes'
    while not rospy.is_shutdown():
      pub.publish(pa)
      rospy.sleep(1.0)    
    '''
    
    # Check if planning is trivially infeasible on this environment
    '''
    pub = rospy.Publisher('pose_test', PoseStamped, queue_size=1)
    ps = PoseStamped()
    ps.header.frame_id = 'map'
    ps.header.stamp = rospy.Time.now()
    ps.pose.position.x = source[0]
    ps.pose.position.y = source[1]
    ps.pose.orientation = Utils.angle_to_quaternion(source[2])
    print 'viz pose'
    while not rospy.is_shutdown():
      pub.publish(ps)
      rospy.sleep(1.0)    
    '''
    if not self.environment.manager.get_state_validity(source):
      print 'Source in collision'
      return [[],False]

    if not self.environment.manager.get_state_validity(target):
      print 'Target in collision'
      return [[],False]
      
    plan = self.planner.plan()
    
    if plan:
      plan = self.planner.post_process(plan, 5)
      #self.planner.simulate(plan)
     ''' 
      pub = rospy.Publisher('node_test', PoseArray, queue_size=1)
      pa = PoseArray()
      pa.header.frame_id = "/map"
      for i in xrange(len(plan)):
        config = plan[i]
        pose = Pose()
        pose.position.x = config[0]
        pose.position.y = config[1]
        pose.position.z = 0.0
        pose.orientation = Utils.angle_to_quaternion(config[2])
        pa.poses.append(pose)
      print 'viz nodes'
      while not rospy.is_shutdown():
        pub.publish(pa)
        rospy.sleep(1.0) 
      '''
      flat_plan = [el for config in plan for el in config]
      return [flat_plan, True]
    else:
      return [[],False]
    
if __name__ == '__main__':
  rospy.init_node('planner_node', anonymous=True)

  pn = PlannerNode()
  
  rospy.spin()  
