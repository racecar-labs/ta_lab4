import math
import numpy
import IPython
import Dubins
import KinematicModel as model
from matplotlib import pyplot as plt
import cv2
import Utils
import time
import random

class HaltonPlanner(object):
  
  def __init__(self, planningEnv):
    self.planningEnv = planningEnv
    self.sid = self.planningEnv.graph.number_of_nodes() - 2
    self.tid = self.planningEnv.graph.number_of_nodes() - 1
    self.planIndices = []
    self.cost = 0
    self.open = {}
    self.closed = {}
    self.parent = {}
    self.gValues = {}

  def plan(self):
    self.sid = self.planningEnv.graph.number_of_nodes() - 2
    self.tid = self.planningEnv.graph.number_of_nodes() - 1
    # Initialize
    self.planIndices = []
    self.cost = 0    
    self.closed = {}
    self.parent = {self.sid:None}
    self.open = {self.sid: 0 + self.planningEnv.get_heuristic(self.sid, self.tid)}
    self.gValues = {self.sid:0}

    while len(self.open) != 0:

      # Pop the top key
      top = min(self.open, key = self.open.get)
      self.closed[top] = self.open[top]
      del self.open[top]

      if top == self.tid:
        plan = self.get_solution(self.tid)
        return plan

      for vid in self.planningEnv.get_successors(top):
        if vid in self.closed:
          continue

        tempG = self.gValues[top] + self.planningEnv.get_distance(top, vid)
        if vid in self.open:
          if tempG > self.gValues[vid]:
            continue

        self.open[vid] = tempG + self.planningEnv.get_heuristic(vid, self.tid)
        self.gValues[vid] = tempG
        self.parent[vid] = top

  def post_process(self, plan, timeout):

    t1 = time.time()
    elapsed = 0
    while elapsed < timeout:
      i = random.randint(0, len(plan)-2)
      j = random.randint(0, len(plan)-2)
      if j == i:
        continue
      if j < i:
        i,j = j,i

      if self.planningEnv.manager.get_edge_validity(plan[i], plan[j]):
        px, py, pyaw, clen = Dubins.dubins_path_planning(plan[i], plan[j], 1.0/model.TURNING_RADIUS)
        localPlan = [list(a) for a in zip(px,py,pyaw)]
        plan = numpy.concatenate((plan[:i-1], localPlan, plan[j+1:]), 0)

      elapsed = time.time() - t1
      return plan

  def get_solution(self, vid):

    planID = []
    while vid is not None:
      planID.append(vid)
      vid = self.parent[vid]

    plan = []
    planID.reverse()
    for i in range(len(planID)-1):
      startConfig = self.planningEnv.get_config(planID[i])
      goalConfig = self.planningEnv.get_config(planID[i+1])
      px, py, pyaw, clen = Dubins.dubins_path_planning(startConfig, goalConfig, 1.0/model.TURNING_RADIUS)
      plan.append([list(a) for a in zip(px,py,pyaw)])
      self.planIndices.append(len(plan))
      self.cost += clen

    flatPlan = [item for sublist in plan for item in sublist]
    return flatPlan

  def simulate(self, plan):
    envMap = 255*(self.planningEnv.manager.mapImageBW+1)
    envMap = cv2.cvtColor(envMap, cv2.COLOR_GRAY2RGB)
    
    #envMap = cv2.imread('/home/patrick/racecar_ws/src/ta_lab1/maps/real-floor4_corridor.pgm')
    for i in range(numpy.shape(plan)[0]-1):
      startPixel = Utils.world_to_map(plan[i], self.planningEnv.manager.map_info)
      goalPixel = Utils.world_to_map(plan[i+1], self.planningEnv.manager.map_info)
      cv2.line(envMap,(startPixel[0],startPixel[1]),(goalPixel[0],goalPixel[1]),(255,0,0),5)

    # Generate window
    cv2.namedWindow('Simulation', cv2.WINDOW_NORMAL)
    cv2.imshow('Simulation', envMap)

    # Terminate and exit elegantly
    cv2.waitKey(20000)
    cv2.destroyAllWindows()
