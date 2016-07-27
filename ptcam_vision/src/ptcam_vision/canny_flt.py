#!/usr/bin/env python

""" canny_flt.py - Version 1.0 2016-03-23

    Canny filter for a video stream.
    
    Copyright (c) 2016 Seiichiro Ishijima  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy
import cv2
import cv2.cv as cv
from ptcam_vision.ros2opencv2 import ROS2OpenCV2
import numpy as np
import dynamic_reconfigure.server
from ptcam_vision.cfg import CannyFltConfig

class CannyFlt(ROS2OpenCV2):
    def __init__(self, node_name): 
        super(CannyFlt, self).__init__(node_name)
        
        # Do we show text on the display?
 #       self.show_text = rospy.get_param("~show_text", True)
        # How big should the feature points be (in pixels)?
  #      self.feature_size = rospy.get_param("~feature_size", 1)

        self.high_th = rospy.get_param("~high_th", 30)
        self.low_th  = rospy.get_param("~low_th",  10)
        
        dcfg_srv = dynamic_reconfigure.server.Server(CannyFltConfig, self.dcfg_cb)
        
        cv.NamedWindow("Canny", cv.CV_WINDOW_NORMAL)

    def process_image(self, cv_image):
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        gray = cv2.blur(gray, (7, 7))
        edges = cv2.Canny(gray, self.low_th, self.high_th)
        cv2.imshow("Canny", edges)
        return cv_image
        
    def dcfg_cb(self, config, level):
        self.high_th = config['high_th']
        self.low_th = config['low_th']
        
        return config

if __name__ == '__main__':
    try:
        node_name = "canny_filter"
        CannyFlt(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down " + node_name + " node."
        cv.DestroyAllWindows()
