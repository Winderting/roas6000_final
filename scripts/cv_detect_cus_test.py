#!/usr/bin/env python

# Import ROS libraries and messages
import rospy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Quaternion

# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError

import math
import numpy as np
import tf2_ros
from tf2_ros.buffer import Buffer
from visualization_msgs.msg import Marker


# Define a function to show the image in an OpenCV Window
def show_image(img):
    cv2.imshow('Image Window', img)
    cv2.waitKey(1)

# Define a callback for the Image message
def image_callback(img_msg):
    # log some info about the image topic
    # rospy.loginfo(img_msg.header)

    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, 'passthrough')
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    # Flip the image 90deg
    # cv_image = cv2.transpose(cv_image)
    cv_image = cv2.flip(cv_image,1)


    # Detection using general harr
    # no recognition
    det_img = face_detection_harr(cv_image)

    #  image color
    det_img = cv2.cvtColor(det_img, cv2.COLOR_BGR2RGB)

    # Show the detected image
    # show_image(det_img)

# face detection
def face_detection_harr(img):
    global theta_x
    # pub = rospy.Publisher('visualization_marker', Marker, queue_size=0)
    # rospy.init_node('marker', anonymous=True)

    face_cascade = cv2.CascadeClassifier('/home/erwin/catkin_ws/src/roas6000_final/haarcascade_frontalface_default.xml')
    # face_cascade.load("../trainer.yml")
    # Read the input image
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(img_gray, 1.5, 3)
    
    font = cv2.FONT_HERSHEY_SIMPLEX 
    if len(faces)!=0:
        rospy.loginfo('face found')
        # rospy.loginfo(type(rospy.Time.now()))
        # rospy.loginfo(rospy.Time(0))
        # temporarily, then use NMS
        faces = [faces[0]]

    face_recognizer = cv2.face.createLBPHFaceRecognizer()
    face_recognizer.load('/home/erwin/catkin_ws/src/roas6000_final/face_recognizer_file.xml')

    # similar to NMS

    # Draw rectangle around the faces

    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x+w, y+h), (0, 0, 255), 2)
        Id, conf = face_recognizer.predict(img_gray[x:x+w,y:y+h])
        img = cv2.putText(img, 'face detected', (img.shape[0]-120,10), font, 0.5, (255, 0, 0), 2)
        img = cv2.putText(img, '%s'%str(Id), (x,y-5), font, 0.5, (255, 0, 0), 2)
        rospy.loginfo('x' +str(x))
        rospy.loginfo('y' + str(y))
        theta_x = np.arctan(((x+w/2) / (0.5 * 512) - 1) / np.tan(0.5 * math.pi / 4))
        theta_y = np.arctan(((y+h/2) / (0.5 * 512) - 1) / np.tan(0.5 * math.pi / 4))
        rospy.loginfo('theta_x' + str(theta_x))
        rospy.loginfo('theta_y' + str(theta_y))

        if len(faces)!=0:
            ROI_center_x = - depth_ROI_center * np.sin(theta_x)
            ROI_center_z = 0.55
            ROI_center_y = - np.sqrt((depth_ROI_center * np.cos(theta_y)) ** 2 - ROI_center_x ** 2)
            # ROI_center_x = 2.0
            # ROI_center_z = 0.0
            # ROI_center_y = 1.0
            # ROI_center_y = - depth_ROI_center / np.sqrt(1 + np.arctan(theta_x) ** 2 + np.arctan(theta_y) ** 2)
            # ROI_center_z = - ROI_center_y / np.arctan(theta_y)
            # ROI_center_x = - ROI_center_y / np.arctan(theta_x)
            # Get the current information of transform from map to camera_link
            tf_buffer = Buffer()
            tf_listener = tf2_ros.TransformListener(tf_buffer)
            # now = rospy.Time.now()
            tf_msg = tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
            # rotation and translation from map to camera_link
            q = Quaternion()

            # q = tf_msg.transform.rotation
            q.x = 0
            q.y = 0
            q.z = 0
            q.w = 1

            x = tf_msg.transform.rotation.x
            y = tf_msg.transform.rotation.y
            z = tf_msg.transform.rotation.z
            w = tf_msg.transform.rotation.w
            R = quaternion_rotation_matrix([w, x, y, z])
            d = [[tf_msg.transform.translation.x], [tf_msg.transform.translation.y], [tf_msg.transform.translation.z]]
            # d = [[1], [0], [0]]
            rospy.loginfo('translation: ' + str(d))
            # ROI_center_pos_map += [tf_msg.transform.translation.x, tf_msg.transform.translation.y, tf_msg.transform.translation.z]
            # coordinate of ROI_center in map frame
            ROI_center_pos_map = np.dot(np.linalg.inv(R),
                                        [[ROI_center_x - d[0][0]], [ROI_center_y - d[1][0]], [ROI_center_z - d[2][0]]])
            # ROI_center_pos_map = d + np.dot(np.linalg.inv(R), [[1], [1], [0]])
            # rospy.loginfo('ros_pos_base: ' + str([[ROI_center_x], [ROI_center_y], [ROI_center_z]]))
            rospy.loginfo('ros_pos_base: ' + str([[ROI_center_x], [ROI_center_y], [ROI_center_z]]))
            rospy.loginfo('ros_pos_map: ' + str(ROI_center_pos_map))
            inside_list = False
            k = 0
            # for pos in ROI_center_list:
            #     distance = np.linalg.norm(ROI_center_pos_map - pos)
            #     if distance < 0.5:
            #         # if the ROI_center is close to recorded ROI position, update this position.
            #         ROI_center_list.pop(k)
            #         update_ROI_center_pos_map = (pos + ROI_center_pos_map) / 2
            #         ROI_center_list.append(update_ROI_center_pos_map)
            #         pubMarker(update_ROI_center_pos_map, q, depth_ROI_center, face_name)
            #         inside_list = True
            #     k += 1
            #
            # #  if the ROI_center is not close to any one ROI position recorded, add this pos into the list
            # if not inside_list:
            #     ROI_center_list.append(ROI_center_pos_map)
            #     rospy.loginfo('ROI_list = ' + str(ROI_center_list))

            # pubMarker(ROI_center_pos_map, q, depth_ROI_center, face_name)
            pubMarker([ROI_center_x, ROI_center_y, ROI_center_z], q, depth_ROI_center, 'face_name')
        else:
            clr = Marker()
            clr.id = 0
            clr.color.a = 0.0
            clr.action = Marker.DELETEALL
            pub_marker.publish(clr)
            pub_text.publish(clr)
            rospy.loginfo('refreshing marker')
    return img
    # Display the output
    # cv2.imshow('img', img)
    # cv2.waitKey()

def laser_callback(laser_msg):
    global depth_ROI_center, theta_x
    # rospy.loginfo(laser_msg.header)
    data = laser_msg
    angle_increment = data.angle_increment
    ranges = data.ranges
    depth_ROI_center = ranges[int((theta_x+math.pi*5/9)/angle_increment)]
    # rospy.loginfo(laser_msg.angle_increment)
    # rospy.loginfo(laser_msg.angle_max)
    # rospy.loginfo(laser_msg.angle_min)


def pubMarker(p, q, l, tx):

    arrow = Marker()
    arrow.header.frame_id = "/base_link"
    arrow.header.stamp = rospy.Time(0)
    arrow.id = 0
    arrow.ns = 'marker'
    arrow.type = Marker.CUBE
    arrow.action = Marker.ADD
    arrow.pose.position.x = p[0]
    arrow.pose.position.y = p[1]
    arrow.pose.position.z = p[2]
    arrow.color.a = 1.0  # Don't forget to set the alpha!
    arrow.color.r = 0
    arrow.color.g = 1
    arrow.color.b = 0
    arrow.pose.orientation.x = q.x
    arrow.pose.orientation.y = q.y
    arrow.pose.orientation.z = q.z
    arrow.pose.orientation.w = q.w
    arrow.scale.x = 0.3
    arrow.scale.y = 0.3
    arrow.scale.z = 0.3

    text = Marker()
    text.header.frame_id = "map"
    text.header.stamp = rospy.Time(0)
    text.id = 1
    text.ns = 'text'
    text.type = Marker.TEXT_VIEW_FACING
    text.action = Marker.ADD
    text.pose.position.x = p[0]
    text.pose.position.y = p[1]
    text.pose.position.z = p[2] + 0.5
    text.color.a = 1.0
    text.color.r = 1
    text.color.g = 0
    text.color.b = 0
    text.text = tx
    text.scale.x = 10.0
    text.scale.y = 10.0
    text.scale.z = 1.0
    rospy.loginfo('publishing marker')
    pub_marker.publish(arrow)
    pub_text.publish(text)

def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.

    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3)

    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix.
             This rotation matrix converts a point in the local reference
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]

    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)

    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)

    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])

    return rot_matrix


if __name__ =='__main__':
    global depth_ROI_center, theta_x
    depth_ROI_center = 0
    theta_x = 0
    # Initialize the ROS Node named 'camera_opencv', allow multiple nodes to be run with this name
    rospy.init_node('camera_opencv', anonymous=True)
    # Print "Hello ROS!" to the Terminal and to a ROS Log file located in ~/.ros/log/loghash/*.log
    rospy.loginfo('Hello ROS!')

    # Initialize the CvBridge class
    bridge = CvBridge()
    # Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
    sub_image = rospy.Subscriber('/vrep/image', Image, image_callback, queue_size=0)
    pub_marker = rospy.Publisher('visualization_marker', Marker, queue_size=0)
    pub_text = rospy.Publisher('visualization_text', Marker, queue_size=0)

    sub_laser = rospy.Subscriber('/vrep/scan', LaserScan, laser_callback)
    # Initialize an OpenCV Window named "Image Window"
    cv2.namedWindow('Image Window', 1)

    # Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
    while not rospy.is_shutdown():
        rospy.spin()
