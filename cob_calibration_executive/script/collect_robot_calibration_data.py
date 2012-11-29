#!/usr/bin/env python
#################################################################
##\file
#
# \note
#   Copyright (c) 2011-2012 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#################################################################
#
# \note
#   Project name: care-o-bot
# \note
#   ROS stack name: cob_calibration
# \note
#   ROS package name: cob_calibration_executive
#
# \author
#   Author: Jannik Abbenseth, email:jannik.abbenseth@gmail.com
# \author
#   Author: Sebastian Haug, email:sebhaug@gmail.com
# \author
#   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: January 2012
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer. \n
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution. \n
#     - Neither the name of the Fraunhofer Institute for Manufacturing
#       Engineering and Automation (IPA) nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see <http://www.gnu.org/licenses/>.
#
#################################################################
PKG = 'cob_calibration_executive'
NODE = 'collect_robot_calibration_data_node'
import roslib
roslib.load_manifest(PKG)
import rospy

from simple_script_server import simple_script_server
from cob_calibration_srvs.srv import *
import numpy as np
import yaml


def capture_loop(positions, sss, visible, capture_kinematics, capture_image):
    '''
    Moves arm to all positions using script server instance sss
    and calls capture() to capture samples
    '''

    counter_camera = 0
    counter_kinematics = 0
    for index in range(len(positions)):
        print "--> moving arm to sample #%s" % index
        pos = positions[index]
        joint_pos = [[((a + (np.pi)) % (2 * np.pi)) - (np.pi)
                      for a in positions[index]['joint_position']]]
        print pos
        print type(joint_pos[0][0])
        nh = sss.move_planned("arm", joint_pos)
        while nh.get_state() == 0:
            rospy.sleep(0.2)
        if nh.get_state() != 3:
            sss.move("torso", "home")
            nh = sss.move_planned("arm", joint_pos)
            rospy.sleep(1)
            if nh.get_state() != 3:
                continue

        sss.move("torso", [positions[index]['torso_position']])

        sss.sleep(2)

        visible_response = visible().visible
        if visible_response == 3:
            print "3 Checkerboards found"
            capture_kinematics()
            capture_image()
            print "--> captured 1 sample for camera calibration"
            print "--> captured 1 sample for kinematics calibration"
            counter_camera += 1
            counter_kinematics += 1
        elif visible_response == 2:
            print "2 Checkerboards found"
            capture_kinematics()
            print "--> captured 1 sample for kinematics calibration"
            counter_kinematics += 1

            #capture()


def main():
    rospy.init_node(NODE)
    print "==> %s started " % NODE

    # service client
    checkerboard_checker_name = "/image_capture/visibility_check"
    visible = rospy.ServiceProxy(checkerboard_checker_name, Visible)
    rospy.wait_for_service(checkerboard_checker_name, 2)
    print "--> service client for for checking for chessboards initialized"

    kinematics_capture_service_name = "/collect_data/capture"
    capture_kinematics = rospy.ServiceProxy(
        kinematics_capture_service_name, Capture)
    rospy.wait_for_service(kinematics_capture_service_name, 2)
    print "--> service client for capture robot_states initialized"

    image_capture_service_name = "/image_capture/capture"
    capture_image = rospy.ServiceProxy(image_capture_service_name, Capture)
    rospy.wait_for_service(image_capture_service_name, 2)
    print "--> service client for capture images initialized"

    # init
    print "--> initializing sss"
    sss = simple_script_server()
    sss.init("base")
    sss.init("torso")
    sss.init("head")
    sss.recover("base")
    sss.recover("torso")
    sss.recover("head")

    print "--> setup care-o-bot for capture"
    sss.move("head", "back")

    # get position from parameter server
    position_path = rospy.get_param('position_path', None)
    if position_path is None:
        print "[ERROR]: no path for positions set"
        return
    with open(position_path, 'r') as f:
        positions = yaml.load(f)
    print "==> capturing samples"
    start = rospy.Time.now()
    capture_loop(positions, sss, visible, capture_kinematics, capture_image)
    print "finished after %s seconds" % (rospy.Time.now() - start).to_sec()


if __name__ == '__main__':
    main()
    print "==> done exiting"
