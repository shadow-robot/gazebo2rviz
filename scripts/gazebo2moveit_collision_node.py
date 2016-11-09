#!/usr/bin/env python
"""
Publish all collision elements in gazebo as MoveIt CollisionObjects
"""

from __future__ import print_function

import argparse

import rospy
from tf.transformations import *
from sdf2moveit_collision_node import Srdf2moveit

import pysdf
from gazebo_msgs.msg import ModelStates

lastUpdateTime = None
updatePeriod = 0.5
model_cache = {}
srdf2moveit = None

def on_model_states_msg(model_states_msg):
    global lastUpdateTime
    global updatePeriod
    global srdf2moveit

    sinceLastUpdateDuration = rospy.get_rostime() - lastUpdateTime
    if sinceLastUpdateDuration.to_sec() < updatePeriod:
        return

    lastUpdateTime = rospy.get_rostime()

    for (model_idx, modelinstance_name) in enumerate(model_states_msg.name):
        model_name = pysdf.name2modelname(modelinstance_name)
 
        if not modelinstance_name in model_cache:
            # Add new collision object
            model_cache[modelinstance_name] = srdf2moveit.add_new_collision_object(model_name, modelinstance_name)
 
        # Move existing object
        model = model_cache[modelinstance_name]
        srdf2moveit.update_collision_object_with_pose(model, modelinstance_name, model_states_msg.pose[model_idx])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--freq', type=float, default=2, help='Frequency Markers are published (default: 2 Hz)')
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('gazebo2moveit')
    
    global srdf2moveit
    srdf2moveit = Srdf2moveit()

    global ignored_submodels
    ignored_submodels = rospy.get_param('~ignore_submodels', '').split(';')
    srdf2moveit.ignored_submodels = ignored_submodels
    rospy.loginfo('Ignoring submodels of: %s' % ignored_submodels)

    global updatePeriod
    updatePeriod = 1. / args.freq

    global lastUpdateTime
    lastUpdateTime = rospy.get_rostime()
    modelStatesSub = rospy.Subscriber('gazebo/model_states', ModelStates, on_model_states_msg)

    rospy.loginfo('Spinning')
    rospy.spin()


if __name__ == '__main__':
    main()
