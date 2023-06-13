#! /usr/bin/env python

import rospy
import tf
import tf2_ros
import tf_conversions
from std_msgs.msg import Float32MultiArray, Bool
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import Image
import numpy as np
from functools import partial

START_MARKERS = [13, 14, 15, 16, 17]
globals()['started'] = False
globals()['pathPoses'] = list()
globals()['seen'] = {}

def _multiarray_to_numpy(pytype, dtype, multiarray):
    dims = tuple(map(lambda x: x.size, multiarray.layout.dim))
    return np.array(multiarray.data, dtype = pytype).reshape(dims).astype(dtype)
to_numpy_f32 = partial(_multiarray_to_numpy, float, np.float32)

def transpose(pose: Pose, poseFrameId: str = "odom", fromFrame: str = "/odom", toFrame: str = "/map"):
  try:
    delay = 0.5
    poseStamped = PoseStamped()
    poseStamped.header.stamp = rospy.Time().now() - rospy.Duration(delay)
    poseStamped.header.frame_id = poseFrameId
    poseStamped.pose = pose
    listener.waitForTransform(toFrame, fromFrame, rospy.Time().now, rospy.Duration(10.0))
    transposed = listener.transformPose(toFrame, poseStamped)
    return transposed
  except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    rospy.loginfo("could not load transform: " + fromFrame + " -> " + toFrame)

def odomCallback(odometry: Odometry):
  if globals()['started'] == True:
    globals()['odometry'] = odometry
    transposed = transpose(odometry.pose.pose)
    pathPoses = []
    pathPoses = pathPoses + globals()['pathPoses']
    pathPoses.append(transposed)
    globals()['pathPoses'] = pathPoses
    path = Path()
    path.header.frame_id = "map"
    path.header.stamp = rospy.Time.now()
    path.poses = pathPoses
    # print("Publishing path ...")
    globals()['pathPub'].publish(path)

def estimatePose():
  pose = globals()['odometry'].pose.pose
  # transposed = transpose(cameraPose, "camera_link", "/camera_link")
  # cameraDepthImage = globals()['cameraDepthImage']
  return transpose(pose)

def cameraDepthImageCallback(image: Image):
  globals()['cameraDepthImage'] = image

def objectsCallback(msg: Float32MultiArray):
  if (msg.data):
      print("Found object(s) ...")
      # objects = to_numpy_f32(msg.data)
      # for object in objects:
      id = msg.data[0]
      if (id in START_MARKERS and globals()['started'] == False):
        print("Found start marker " + str(id))
        globals()['started'] = True
        globals()['readyPub'].publish(Bool(True))
      elif (not(id in START_MARKERS) and globals()['started'] == True):
        print("Found hazard " + str(id))
        if not id in globals()['seen']:
          globals()['seen'][id] = True
          marker = Marker()
          marker.header.frame_id = "map"
          marker.id = int(id)
          marker.pose = estimatePose()
          marker.type=2
          marker.scale.x = 1.0
          marker.scale.y = 1.0
          marker.scale.z = 1.0
          marker.color.r = 0.0
          marker.color.g = 1.0
          marker.color.b = 0.0
          print("Publishing hazard " + str(id))
          globals()['hazardsPub'].publish(marker)
        else:
          print("Hazard " + str(id) + " has already been published")

def initialise():
  print("Initialising detection ...")
  globals()['readyPub'] = rospy.Publisher('/challenge/ready', Bool, queue_size=1)
  globals()['hazardsPub'] = rospy.Publisher('/hazards', Marker, queue_size=1)
  globals()['pathPub'] = rospy.Publisher('/path', Path, queue_size=1)
  globals()['cancelGoalsPub'] = rospy.Publisher('/challenge/cancelGoals', Bool, queue_size=1)
  globals()['objectsSub'] = rospy.Subscriber('/objects', Float32MultiArray, objectsCallback)
  globals()['odomSub'] = rospy.Subscriber('/odom', Odometry, odomCallback)
  globals()['cameraDepthImageSub'] = rospy.Subscriber('/camera/depth/image', Image, cameraDepthImageCallback)
  while not rospy.is_shutdown():
    rospy.spin()

if __name__ == '__main__':
  rospy.init_node('detection')
  listener = tf.TransformListener()
  listener.waitForTransform("/map", "/odom", rospy.Time(), rospy.Duration(10.0))
  initialise()
