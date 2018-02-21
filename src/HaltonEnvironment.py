# Take map and call the obstacle manager and pass it to the obstacle manager
# Create the successor function for different representations of the configuration space

import os
import math
import numpy
import IPython
import Dubins
import GraphGenerator
import networkx as nx
import KinematicModel as model
from ObstacleManager import ObstacleManager

class HaltonEnvironment(object):

  def __init__(self, mapMsg, graphFile, source, target):

    # Setup member variables
    self.source = source
    self.target = target
    self.manager = ObstacleManager(mapMsg)

    # Generate the Graph on the fly if required
    self.radius = 100
    if graphFile is None:
      n = 500
      bases = [2,3,5]
      lower = [0,0,0]
      upper = [64,75,2*numpy.pi]

      G = GraphGenerator.euclidean_halton_graph(n, self.radius, bases, lower, upper, source, target, mapMsg)
      nx.write_graphml(G, "haltonGraph.graphml")
      self.graph = nx.read_graphml("haltonGraph.graphml")

    else:
      # Check if graph file exists
      if not os.path.isfile(graphFile):
        print "ERROR: map file not found!"
        quit()
      self.graph = nx.read_graphml(graphFile)
      
      if source is not None:
        GraphGenerator.insert_vertices(self.graph, [source], self.radius)

      if target is not None:
        GraphGenerator.insert_vertices(self.graph, [target], self.radius)
 

  def set_source_and_target(self, source, target):
    self.source = source
    self.target = target
    GraphGenerator.insert_vertices(self.graph, [source, target], self.radius)  

  def get_config(self, vid):
    return [float(a) for a in self.graph.node[str(vid)]["state"].split()]

  def get_successors(self, vid):
    successors = [int(i) for i in self.graph.neighbors(str(vid))]
    freeSuccessors = []
    for i in successors:
      config1 = self.get_config(vid)
      config2 = self.get_config(i)
      if self.manager.get_edge_validity(config1, config2):
        freeSuccessors.append(i)

    return freeSuccessors

  def get_state_validity(self, config2D):
    return self.manager.get_state_validity(config2D)

  def get_distance(self, vid1, vid2):
    G = self.graph
    return float(G[str(vid1)][str(vid2)]['length'])

  def get_heuristic(self, vid, tid):
    config1 = self.get_config(vid)
    config2 = self.get_config(tid)
    return Dubins.path_length(config1, config2, 1.0/model.TURNING_RADIUS)
