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
    screw_box_path = os.path.join(ur_path, 'urdf', 'screw_box.urdf')
    human_path = os.path.join(ur_path, 'urdf', 'human.urdf')
    screw_M8_path = os.path.join(ur_path, 'urdf', 'screw_M8.urdf')
    leg_path = os.path.join(ur_path, 'urdf', 'leg.urdf')
    table_path = os.path.join(ur_path, 'urdf', 'table.urdf')

    # Wait for service to start
    rospy.wait_for_service('gazebo/spawn_urdf_model')
    spawn = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)
    delete = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
   
    # block_name = 'block_perfect'
    # block_name = 'block_largerHole'
    box_name = 'screw_box'
    screw1_name = 'screw1_M8'
    screw2_name = 'screw2_M8'
    screw3_name = 'screw3_M8'
    screw4_name = 'screw4_M8'
    human_name = 'Mario'
    leg1_name = 'leg1'
    leg2_name = 'leg2'
    leg3_name = 'leg3'
    leg4_name = 'leg4'
    table_name = 'table'


    # Delete previous blocks
    # delete('block_perfect')
    # delete('block_largerHole')
    delete(box_name)
    delete(human_name)
    delete(screw1_name)
    delete(screw2_name)
    delete(screw3_name)
    delete(screw4_name)
    delete(leg1_name)
    delete(leg2_name)
    delete(leg3_name)
    delete(leg4_name)
    delete(table_name)

    box_x = 0.2
    box_y = 0.5
    box_off = 0.03 - 0.005
    # Spawn screw_box
    box_pose = Pose(Point(box_x, box_y, 0), Quaternion(0, 0, 0, 0))
    spawn(box_name, open(screw_box_path, 'r').read(), 'screw_box', box_pose, 'world')
    # Spawn screw_M8
    screw_name = 'screw_M8' 
    screw1_pose = Pose(Point(box_x+box_off, box_y+box_off, 0.035), Quaternion(0, 0, 0, 0))
    spawn(screw1_name, open(screw_M8_path, 'r').read(), 'screw1', screw1_pose, 'world')
    screw2_pose = Pose(Point(box_x+box_off, box_y-box_off, 0.035), Quaternion(0, 0, 0, 0))
    spawn(screw2_name, open(screw_M8_path, 'r').read(), 'screw2', screw2_pose, 'world')
    screw3_pose = Pose(Point(box_x-box_off, box_y+box_off, 0.035), Quaternion(0, 0, 0, 0))
    spawn(screw3_name, open(screw_M8_path, 'r').read(), 'screw3', screw3_pose, 'world')
    screw4_pose = Pose(Point(box_x-box_off, box_y-box_off, 0.035), Quaternion(0, 0, 0, 0))
    spawn(screw4_name, open(screw_M8_path, 'r').read(), 'screw4', screw4_pose, 'world')



    # Spawn block
    # if block_name== 'block_perfect':
    #     pose = Pose(Point(block_x, block_y, 0), Quaternion(0, 0, 0, 0))
    #     # import pdb;pdb.set_trace()
    #     spawn(block_name, open(block_perfect_path, 'r').read(), 'block', pose, 'world')
    # elif block_name == 'block_largerHole':
    #     pose = Pose(Point(block_x, block_y, 0), Quaternion(0, 0, 0, 0))
    #     spawn(block_name, open(block_largerHole_path, 'r').read(), 'block', pose, 'world')

    # spawn human
    human_pose = Pose(Point(1, 1, 0), Quaternion(0, 0, 0, 0))
    # human_pose = Pose(Point(0.3, 0.3, 0), Quaternion(0, 0, 0, 0))
    # import pdb;pdb.set_trace()
    #spawn(human_name, open(human_path, 'r').read(), 'human', human_pose, 'world')

    
    #spawn legs and table'
    x_table = 0.3
    y_table = 0.2
    x_off = 0.09-0.005
    y_off = 0.165-0.005
    leg1_pose = Pose(Point(x_table+x_off, y_table+y_off, 0), Quaternion(0, 0, 0, 0))
    spawn(leg1_name, open(leg_path, 'r').read(), 'leg1', leg1_pose, 'world')
    leg2_pose = Pose(Point(x_table+x_off, y_table-y_off, 0), Quaternion(0, 0, 0, 0))
    spawn(leg2_name, open(leg_path, 'r').read(), 'leg2', leg2_pose, 'world')
    leg3_pose = Pose(Point(x_table-x_off, y_table+y_off, 0), Quaternion(0, 0, 0, 0))
    spawn(leg3_name, open(leg_path, 'r').read(), 'leg3', leg3_pose, 'world')
    leg4_pose = Pose(Point(x_table-x_off, y_table-y_off, 0), Quaternion(0, 0, 0, 0))
    spawn(leg4_name, open(leg_path, 'r').read(), 'leg4', leg4_pose, 'world')


    table_pose = Pose(Point(x_table, y_table, 0.035), Quaternion(0, 0, 0, 0))
    spawn(table_name, open(table_path, 'r').read(), 'table', table_pose, 'world')

    # screw1_pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 0))
    # spawn(screw1_name, open(screw_M8_path, 'r').read(), 'screw1', screw1_pose, 'world')
    # screw2_pose = Pose(Point(0.1,  0, 0), Quaternion(0, 0, 0, 0))
    # spawn(screw2_name, open(screw_M8_path, 'r').read(), 'screw2', screw2_pose, 'world')
    

