#!/usr/bin/env python

import rospy
import rospkg
import os
import sys
import yaml
import random
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

yamlpath = 'lab2_data.yaml'

if __name__ == '__main__':

    # Initialize rospack
    rospack = rospkg.RosPack()
    # Get path to yaml
    lab2_path = rospack.get_path('lab2pkg_py')
    yamlpath = os.path.join(lab2_path, 'scripts', 'lab2_data_proj.yaml')

    with open(yamlpath, 'r') as f:
        try:
            # Load the data as a dict
            data = yaml.load(f)
            # Load block position
            block_xy_pos = data['block_xy_pos']
            
        except:
            sys.exit()

    # Initialize ROS node
    rospy.init_node('ur3_gazebo_spawner', anonymous=True)
    # Initialize ROS pack
    rospack = rospkg.RosPack()
    # Get path to block
    ur_path = rospack.get_path('ur_description')
    block_perfect_path = os.path.join(ur_path, 'urdf', 'block_perfect.urdf')
    block_largerHole_path = os.path.join(ur_path, 'urdf', 'block_largerHole.urdf')
    screw_M8_path = os.path.join(ur_path, 'urdf', 'screw_M8.urdf')
    # Wait for service to start
    rospy.wait_for_service('gazebo/spawn_urdf_model')
    spawn = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)
    delete = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
   
    # block_name = 'block_perfect'
    block_name = 'block_largerHole'
    screw_name = 'screw_M8'

    # Delete previous blocks
    delete('block_perfect')
    delete('block_largerHole')
    delete(screw_name)

    # Spawn block
    if block_name== 'block_perfect':
        pose = Pose(Point(block_xy_pos[0][0], 
                        block_xy_pos[0][1], 0), Quaternion(0, 0, 0, 0))
        # import pdb;pdb.set_trace()
        spawn(block_name, open(block_perfect_path, 'r').read(), 'block', pose, 'world')
    elif block_name == 'block_largerHole':
        pose = Pose(Point(block_xy_pos[0][0], 
                        block_xy_pos[0][1], 0), Quaternion(0, 0, 0, 0))
        spawn(block_name, open(block_largerHole_path, 'r').read(), 'block', pose, 'world')


    # Spawn screw_M8
    screw_name = 'screw_M8' 
    pose = Pose(Point(block_xy_pos[0][0]+0.03, 
                    block_xy_pos[0][1]+0.05, 0.1), Quaternion(0, 0, 0, 0))
    # import pdb;pdb.set_trace()
    spawn(screw_name, open(screw_M8_path, 'r').read(), 'block', pose, 'world')

