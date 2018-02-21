#!/usr/bin/env python

import rospy
import numpy as np
import tf.transformations
import tf

def quaternion_to_angle(q):
    """Convert a quaternion _message_ into an angle in radians.
    The angle represents the yaw.
    This is not just the z component of the quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    roll, pitch, yaw = tf.transformations.euler_from_quaternion((x, y, z, w))
    return yaw

def world_to_map(config, map_info):

  # TODO(avk): simple coversion. NOT consistent with APK.
  scale = 0.02

  xPosition = int(np.floor(config[0]/scale))
  yPosition = int(np.floor(config[1]/scale))
  orientation = config[2]

  return [xPosition, yPosition, orientation]

if __name__ == '__main__':
  print world_to_map([64,64,20])

'''
def world_to_map(config, map_info):

  scale = map_info.resolution
  angle = -quaternion_to_angle(map_info.origin.orientation)

  xPosition = (1.0/float(scale))*(config[0] - map_info.origin.position.x)
  yPosition = (1.0/float(scale))*(config[1] - map_info.origin.position.y)

  # rotation
  c, s = np.cos(angle), np.sin(angle)
  # we need to store the x coordinates since they will be overwritten
  temp = xPosition
  xPosition =  c*xPosition - s*yPosition
  yPosition =  s*temp      + c*yPosition
  orientation = config[2] + angle
  
  return [int(xPosition), int(yPosition), orientation]
'''
