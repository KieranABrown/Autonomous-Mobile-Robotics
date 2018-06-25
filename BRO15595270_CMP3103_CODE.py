
#!/usr/bin/env python
import rospy, cv2, cv2.cv, cv_bridge, numpy
from math import radians
from numpy import nanmean
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from cv_bridge import CvBridge
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import Empty
from cv2 import moments, waitKey, imshow, namedWindow


class Follower:

  #RED, GREEN, BLUE, YELLOW
  mask_str = ["RED", "GREEN", "BLUE", "YELLOW"] # Names each of the mask values in the array
  masks = [[numpy.array((0, 100, 50)), numpy.array((5, 255, 255))],   # Mask value for Red
            [numpy.array((60, 100, 50)),numpy.array((70, 255, 255))],   # Mask value for Green
            [numpy.array((110, 100, 50)),numpy.array((130, 255, 255))], # Mask value for Blue
            [numpy.array((25, 100, 50)), numpy.array((30, 255, 255))]]; # Mask value for Yellow
  mflags = [True] * len(masks) # Creates 4 'flag' values for the array


  def __init__(self):
    # Window Creators  
    self.bridge = cv_bridge.CvBridge()
    namedWindow("Image", 1)
    namedWindow("Target", 2)
    namedWindow("Depth", 3)

    # Subscriber and Publishers
    self.image_sub = rospy.Subscriber('/turtlebot/camera/rgb/image_raw', 
                                      Image, self.image_callback)
    self.depth_sub = rospy.Subscriber('/turtlebot/camera/depth/image_raw', 
                                      Image, self.depth_callback)
    self.laser_sub = rospy.Subscriber("/turtlebot/scan", 
                                      LaserScan, self.distance)
    self.cmd_vel_pub = rospy.Publisher('/turtlebot/cmd_vel_mux/input/teleop',
                                       Twist, queue_size=10)
    reset_gazebo = rospy.ServiceProxy('/gazebo/reset_world', Empty)

    # Movement Variables 
    self.twist = Twist()
    self.detection = False

    # Obstacle avoidance
    self.acknowledgement = "Obstacle!"
    self.current_distance = 0
    self.minimum_distance = 1
    self.roam = 0
    self.close = 0

    # Camera 
    self.h = 0
    self.w = 0
    self.d = 0
   
    # Global Variables
    global Turn
    Turn = 0
    global Forward 
    Forward = 0 
    global r 
    r = rospy.Rate(5)

  def depth_callback(self, ros_image):
    try:
        depth_image = self.bridge.imgmsg_to_cv2(ros_image, "passthrough")        
    except CvBridgeError, e:
        print e

    depth_array = numpy.array(depth_image, dtype=numpy.float32)
    cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
    depth_display_image = self.process_depth_image(depth_array)
    imshow("Depth", depth_display_image)

  def process_depth_image(self, frame):
    return frame

  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # height, width and depth of image shape
    h, w, d = image.shape
    # Create an empty mask 
    mask = numpy.zeros((h, w), dtype=numpy.uint8)

    #Populate mask with ranges that are still flagged 'true' in mflags
    for i in range(0, len(self.masks)):
        if self.mflags[i]:
            mask += cv2.inRange(hsv, self.masks[i][0], self.masks[i][1])
            
    # Once all 4 flags are found close the program
    if self.mflags[0:3] == False:
      print("ALL POLES FOUND")
      cv2.destroyAllWindows()
        
    #Begins the calculations to find the mass of the pillars
    M = moments(mask)

    if M['m00'] == 0:
      self.roam = 1
      self.detection = True
      #print ("No Pillar Found")
      #self.movement_commands(0.4, 0)
      self.roaming()
    
    # For every object which mass is greater than 0 perform operation
    if M['m00'] >= 1: 
      self.roam = 0
      #print ("Pillar Found")
      self.detection = False 
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      # Create the target circle for the robot to find the pillars with.
      cv2.circle(image, (cx, cy), 10, (0,0,255), -1)
      
      # Rotate to avoid objects 
      if self.current_distance < self.minimum_distance:
          self.movement_commands(-self.minimum_distance, radians(180))
      # Course correction of robot to keep red circle on object
      err = cx - w/2
      if self.detection == False:
        self.movement_commands(0.5, -float(err) / 100)

      # Sets the distance for the robot by comparing its mass. 
      if M['m00'] >= 6500000.0:
        tx = h / 2
        ty = w / 2
        self.movement_commands(0.0, 0.0)

        for i in range(0, len(self.masks)):
          if self.mflags[i]:  
            test = cv2.inRange(hsv, self.masks[i][0], self.masks[i][1])
            if numpy.sum(test) > 5000:
                print("Found: {}".format(self.mask_str[i]))
                self.mflags[i] = False
                self.movement_commands(0.0, radians(180))
                r.sleep()
                self.roaming()
                
                #go search

    # Display the Image and the Target
    imshow("Image", image)
    imshow("Target", mask)
    waitKey(3)
        
  # Sets the robot to roam the environment to find objects to
  def roaming(self):
    if self.detection == True:
      while not self.roam == 1:
        self.movement_commands(0.4,0)
      if self.roam == 1:
        if self.current_distance < self.minimum_distance:
          self.movement_commands(-self.minimum_distance, radians(180))
    # else:
      self.roam = 1
      self.movement_commands(0.4, radians(90))

  def distance(self, dat):
    min_range = dat.range_max
    for v in dat.ranges:
      if v < min_range:
        self.current_distance = v

  def movement_commands(self, Forward, Turn):
      self.twist.linear.x = Forward # Move forward
      self.twist.angular.z = Turn # Turn
      self.cmd_vel_pub.publish(self.twist) # Publish movement

  def stuck_bot(self):
    if not (self.timer):
      print ("Bot is Stuck")
      self.detection = True
      self.roam = 1
      self.timer = rospy.Timer(rospy.Duration(2), self.movement_commands(-0.2, 0.0))

def main():        
    rospy.init_node('follower')
    Follower()
    rospy.spin()
    r = rospy.Rate(5)

if __name__ == "__main__":
    main()



