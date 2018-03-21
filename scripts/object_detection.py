#!/usr/bin/env python

# Module for doing object detection on an image stream, then extract position data
# to publish on ros topics.
#
# Uses rgb images and depth images from a zed camera.
# Uses a pretrained neural network for object detection.
# Uses ROS to get position data and to publish id and position of objects.
# Publishes id and position of objects on several different forms.

from math import cos, sin, pi

import rospy
from cyborg_ros_object_detector.msg import detected_object_polar, detected_objects_polar
from cyborg_ros_object_detector.msg import detected_object_cartesian, detected_objects_cartesian


# Function converts from polar to cartesian coordinates. angle has unit radians, NOT degrees.
def polar2cartesian(dist, angle):
    x = dist*cos(angle)
    y = dist*sin(angle)
    return x, y

'''
def predict_single_image(image):
    # TODO
    object_predictions = []
    return object_predictions


def discard_uncertain_predictions(object_predictions, threshold):
    # TODO:
    # 1. Discard objects with probability lower than threshold
    # 2. Store back into object_predictions(maybe) and then return?!
    return object_predictions


def calculate_midpoint_of_bounding_boxes(object_predictions):
    # TODO:
    # 1. Calculate midpoint of object based on bounding box info.
    # 2. Store back into object_predictions(maybe) and then return?!

    return object_predictions


def extract_depth_info_at_objects(object_predictions):
    # TODO:
    # 1. Extract depth from depth-map at objects location.
    # 2. Store back into object_predictions(maybe) and then return?!
    return object_predictions


# This form of object_predictions = [{'id: , angle: , depth: '},
#                                    {'id: , angle: , depth: '}, ... ]
# could be nice for the next part.


def create_detected_objects_polar_from_predictions_rel_zed(object_predictions):
    objs_rel_zed_pol = detected_objects_polar()

    # TODO:
    # Iterate through object_predictions. For each detected object
    #   1. define object of class detected_object_polar
    #   2. populate object with id, distance, angle
    #   3. append object to objs_rel_zed_pol.objects 

    return objs_rel_zed_pol


def create_detected_objects_cartesian_from_predictions_rel_zed(object_predictions):
    objs_rel_zed_cart = detected_objects_cartesian()

    # TODO:
    # Iterate through object_predictions. For each detected object
    #   1. define object of class detected_object_cartesian
    #   2. Calculate cartesian coordinates from polar with polar2cartesian()
    #   3. populate object with id, x, y
    #   4. append object to objs_rel_zed_cart

    return objs_rel_zed_cart

def create_detected_objects_cartesian_from_predictions_rel_world(object_predictions):
    objs_rel_world_cart = detected_objects_cartesian()

    # TODO:
    # Iterate through object_predictions. For each detected object
    #   1. define object of class detected_object_cartesian
    #   2. Calculate cyborg world cartesian coordinates from angle and depth relative zed
    #      in addition to data from the cyborg base regarding position and orientation.       
    #      (Is this data available at all at the base tho?! have not checked yet!)
    #   4. populate object with id, x, y
    #   5. append object to objs_rel_world_cart

    return objs_rel_world_cart

def detect_and_publish_single_image(image):
    threshold = 80

    object_predictions                = predict_single_image(image)
    best_object_predictions           = discard_uncertain_predictions(object_predictions, threshold)
    midpoint_object_predictions       = calculate_midpoint_of_bounding_boxes(best_object_predictions)
    depth_and_angle_object_pedictions = extract_depth_info_at_objects(midpoint_object_predictions)

    objs_rel_zed_pol    = create_detected_objects_polar_from_predictions_rel_zed(depth_and_angle_object_pedictions)
    object_detect_relative_zed_polar.publish(objs_rel_zed_pol)

    objs_rel_zed_cart   = create_detected_objects_cartesian_from_predictions_rel_zed(depth_and_angle_object_pedictions)
    object_detect_relative_zed_cartesian.publish(objs_rel_zed_cart)

    objs_rel_world_cart = create_detected_objects_cartesian_from_predictions_rel_world(depth_and_angle_object_pedictions)
    object_detect_relative_world_cartesian.publish(objs_rel_world_cart)

    return
'''

def run_object_detection_and_data_publishing():
    object_detect_relative_zed_polar       = rospy.Publisher('object_detector_relative_zed_polar', detected_objects_polar, queue_size=5)
    object_detect_relative_zed_cartesian   = rospy.Publisher('object_detect_relative_zed_cartesian', detected_objects_cartesian, queue_size=5)
    object_detect_relative_world_cartesian = rospy.Publisher('object_detect_relative_world_cartesian', detected_objects_cartesian, queue_size=5)
    
    rospy.init_node('object_detector', anonymous=True)
    rate = rospy.Rate(1.0/3.0) # For every 3rd second, recive one message

    angle1    = pi/10
    distance1 = 10.5

    angle2    = -pi/8
    distance2 = 13.3

    i = True
    j = True
    while not rospy.is_shutdown():
        #===============[How to populate detected_objects msg and publish]===============
        objs_rel_zed_pol = detected_objects_polar()

        obj1_rel_zed_pol = detected_object_polar()
        obj1_rel_zed_pol.id = "obj1 %s" % rospy.get_time()
        obj1_rel_zed_pol.distance   = distance1
        obj1_rel_zed_pol.angle      = angle1
        objs_rel_zed_pol.objects.append(obj1_rel_zed_pol)

        obj2_rel_zed_pol = detected_object_polar()
        obj2_rel_zed_pol.id = "obj2 %s" % rospy.get_time()
        obj2_rel_zed_pol.distance   = distance2
        obj2_rel_zed_pol.angle      = angle2
        objs_rel_zed_pol.objects.append(obj2_rel_zed_pol)

        object_detect_relative_zed_polar.publish(objs_rel_zed_pol)

        if i == True:
            print(objs_rel_zed_pol)
            i = False
        #===============[How to populate detected_objects msg and publish]===============
        objs_rel_zed_cart = detected_objects_cartesian()

        obj1_rel_zed_cart = detected_object_cartesian()
        obj1_rel_zed_cart.id = "obj1 %s" % rospy.get_time()
        obj1_rel_zed_cart.x, obj1_rel_zed_cart.y = polar2cartesian(distance1, angle1)
        objs_rel_zed_cart.objects.append(obj1_rel_zed_cart)

        obj2_rel_zed_cart = detected_object_cartesian()
        obj2_rel_zed_cart.id = "obj2 %s" % rospy.get_time()
        obj2_rel_zed_cart.x, obj2_rel_zed_cart.y = polar2cartesian(distance2, angle2)
        objs_rel_zed_cart.objects.append(obj2_rel_zed_cart)

        object_detect_relative_zed_cartesian.publish(objs_rel_zed_cart)
        
        if j == True:
            print(objs_rel_zed_cart)
            j = False
        #===============[How to populate detected_objects msg and publish]===============
        
        rate.sleep()


'''
def run_object_detection_and_data_publishing():
    object_detect_relative_zed_polar       = rospy.Publisher('object_detector_relative_zed_polar', detected_objects_polar, queue_size=5)
    object_detect_relative_zed_cartesian   = rospy.Publisher('object_detect_relative_zed_cartesian', detected_objects_cartesian, queue_size=5)
    object_detect_relative_world_cartesian = rospy.Publisher('object_detect_relative_world_cartesian', detected_objects_cartesian, queue_size=5)
    
    rospy.init_node('object_detector', anonymous=True)
    rate = rospy.Rate(1.0/3.0) # For every 3rd second, recive one message

    while not rospy.is_shutdown():
        #get Image from chosen image source
        detect_and_publish_single_image(image)
        rate.sleep()
'''



if __name__ == '__main__':
    try:
        run_object_detection_and_data_publishing()
    except rospy.ROSInterruptException:
        pass
