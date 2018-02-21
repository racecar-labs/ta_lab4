import cv2
import math
import numpy
import IPython
import Dubins
import Utils
import KinematicModel as model

class ObstacleManager(object):

  def __init__(self, mapMsg):
    
    self.map_info = mapMsg.info
    self.mapImageGS = numpy.array(mapMsg.data, dtype=numpy.uint8).reshape((mapMsg.info.height, mapMsg.info.width,1))


    # Retrieve the map dimensions
    height, width, channels = self.mapImageGS.shape
    self.mapHeight = height
    self.mapWidth = width
    self.mapChannels = channels

    # Binarize the Image
    self.mapImageBW = 255*numpy.ones_like(self.mapImageGS, dtype=numpy.uint8)
    self.mapImageBW[self.mapImageGS==0] = 0
    self.mapImageBW = self.mapImageBW[::-1,:,:] # Need to flip across the y-axis    
    
    # Obtain the car length and width in pixels
    # TODO (avk): Get the right numbers
    self.robotWidth = int(model.CAR_WIDTH/self.map_info.resolution + 0.5)
    self.robotLength = int(model.CAR_LENGTH/self.map_info.resolution + 0.5)
  '''
  def get_state_validity(self, config):

    # Convert the configuration to map-coordinates
    mapConfig = Utils.world_to_map(config, self.map_info)

    if mapConfig[1] >= self.mapHeight or mapConfig[0] >= self.mapWidth:
      return False

    halfDiagonal = math.sqrt(self.robotWidth**2 + self.robotLength**2)/2.0
    topLeftX = mapConfig[0] - halfDiagonal*math.cos(numpy.pi - config[2] - numpy.pi/4)
    topLeftY = mapConfig[1] - halfDiagonal*math.sin(numpy.pi - config[2] - numpy.pi/4)
    topRightX = mapConfig[0] + halfDiagonal*math.cos(config[2] - numpy.pi/4)
    topRightY = mapConfig[1] - halfDiagonal*math.sin(config[2] - numpy.pi/4)

    topLeft = [int(topLeftX), int(topLeftY)]
    topRight = [int(topRightX), int(topRightY)]

    if topLeft[1] >= self.mapHeight or topLeft[0] >= self.mapWidth:
      return False
    if topRight[1] >= self.mapHeight or topRight[0] >= self.mapWidth:
      return False

    testPixel1 = self.mapImageBW[topLeft[1],topLeft[0]]#[0]
    testPixel2 = self.mapImageBW[topRight[1],topRight[0]]#[0]
    testPixel3 = self.mapImageBW[mapConfig[1],mapConfig[0]]#[0]

    if testPixel1 or testPixel2 or testPixel3:
      return False
    else:
      return True
  '''

  def get_state_validity(self, config):

    # Convert the configuration to map-coordinates
    mapConfig = Utils.world_to_map(config, self.map_info)

    if mapConfig[1] >= self.mapHeight or mapConfig[0] >= self.mapWidth:
      return False

    additionalClearance = 0
    halfDiagonal = int(math.sqrt(self.robotWidth**2 + self.robotLength**2)/2.0) + additionalClearance
    topLeft = [mapConfig[0] - halfDiagonal, mapConfig[1] - halfDiagonal]
    bottomRight = [mapConfig[0] + halfDiagonal, mapConfig[1] + halfDiagonal]

    if topLeft[1] >= self.mapHeight or topLeft[0] >= self.mapWidth:
      return False
    if bottomRight[1] >= self.mapHeight or bottomRight[0] >= self.mapWidth:
      return False

    rect = self.mapImageBW[topLeft[1]:bottomRight[1], topLeft[0]:bottomRight[0]]
    if rect.sum():
      return False
    else:
      return True  

  def get_edge_validity(self, config1, config2):

    if not self.get_state_validity(config2):
      return False

    px, py, pyaw, cost = Dubins.dubins_path_planning(config1, config2, 1.0/model.TURNING_RADIUS)

    idx = 0
    while idx < len(px):
      if not self.get_state_validity([px[idx], py[idx], pyaw[idx]]):
        return False
      idx += 1

    return True

  def get_dubins_path_validity(self, px, py, pyaw):
    # Repeatedly call state validity checker
  	return True


# Test
if __name__ == '__main__':
  # Load the image
  mapImageGS = cv2.imread('../maps/real-floor4_corridor.pgm')

  # Access its shape
  height, width, channels = mapImageGS.shape
  print height, width, channels

  # Set the threshold
  threshold = 205

  # Binarize the image
  mapImageBW = cv2.threshold(mapImageGS, threshold, 255, cv2.THRESH_BINARY_INV)[1]

  # EndPoints of the robot [top left , bottom right] (column, row) 
  # topLeft = Utils.world_to_map([54.60,22,0])
  # bottomRight = Utils.world_to_map([55.40, 23, 0])
  topLeft = [840, 1260]
  bottomRight = [880, 1300]
  print topLeft, bottomRight

  # Draw a rectangle
  robotImageBW = numpy.ones((height,width,channels), numpy.uint8)*255
  cv2.rectangle(robotImageBW,(topLeft[0],topLeft[1]),(bottomRight[0],bottomRight[1]),(0,0,0),-1)

  # Rotate the image by given angle [anti-clockwise]
  angle = 0 # Degrees
  M = cv2.getRotationMatrix2D((width/2,height/2),angle,1)
  dstGS = cv2.warpAffine(mapImageGS,M,(width,height))
  dstBW = cv2.warpAffine(mapImageBW,M,(width,height))

  # Visualize for sanity check
  cv2.rectangle(dstGS,(topLeft[0],topLeft[1]),(bottomRight[0],bottomRight[1]),(150,150,150),-1)

  # Combine the two images
  andOperatedImageBW = cv2.bitwise_or(dstBW, robotImageBW)

  # Create Window
  cv2.namedWindow('testMap', cv2.WINDOW_NORMAL)
  cv2.namedWindow('testRect', cv2.WINDOW_NORMAL)
  cv2.namedWindow('testTogether', cv2.WINDOW_NORMAL)
  cv2.namedWindow('andOperated', cv2.WINDOW_NORMAL)

  # SHow the images
  cv2.imshow('testMap', dstBW)
  cv2.imshow('testRect', robotImageBW)
  cv2.imshow('testTogether', dstGS)
  cv2.imshow('andOperated', andOperatedImageBW)

  # When extracting from image: (row, column)
  rect = andOperatedImageBW[topLeft[1]:bottomRight[1], topLeft[0]:bottomRight[0]]
  if rect.sum() == 0:
    print "Collision Free"
  else:
    print "Collision"

  # Terminate and exit elegantly
  cv2.waitKey()
  cv2.destroyAllWindows()
