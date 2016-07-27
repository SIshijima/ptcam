#!/usr/bin/env python

''' follower.py - Version 1.0 2016-03-27

'''

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import RegionOfInterest, JointState, CameraInfo
from math import radians, degrees
from ros_arduino_python.servo_controller import Servo
import dynamic_reconfigure.server
from ptcam_arduino.cfg import FollowerConfig

class Follower(object):
    def __init__(self, name, cmdpub, reso):
        self.name = name
        self.cmdpub = cmdpub
        prmns = "~follower/" + name + "/"
        self.reso_2 = reso / 2
        self.range_max = radians(rospy.get_param(prmns + "range_max", 90.0))
        self.range_min = radians(rospy.get_param(prmns + "range_min", -90.0))
        self.setparam(rospy.get_param(prmns + "gain", 30.0), rospy.get_param(prmns + "dzone", 10.0))
        
    def follow(self, roipos, crrpos):
        fb = (roipos - self.reso_2) * self.gain
        if abs(fb) > self.dzone:
            cmd = crrpos - fb
            cmd = min(self.range_max, cmd)
            cmd = max(self.range_min, cmd)
            try:
                self.cmdpub.publish(cmd)
            except:
                rospy.loginfo("Publishing command failed")
                
    def setparam(self, gain, dzone):
        self.gain = radians(gain) / self.reso_2
        self.dzone = radians(dzone)


class FollowServo(object):
    def __init__(self, node_name):
        self.node_name = node_name
        self.roi_minarea = rospy.get_param("~follower/roi_minratio", 0.07)  # area ratio in %
        rospy.init_node(node_name)
        rospy.loginfo("Starting node " + str(node_name))
        rospy.Subscriber("/camera/rgb/camera_info", CameraInfo, self.camera_info_cb)
        self.pancmd_pub = rospy.Publisher("/pan/command", Float64, queue_size=1)
        self.tilcmd_pub = rospy.Publisher("/tilt/command", Float64, queue_size=1)
        self.crrpos = (0.0, 0.0)
        self.camreso = None
        while not rospy.is_shutdown() and self.camreso is None:
            rospy.sleep(0.1)
        self.follow_pan = Follower("pan", self.pancmd_pub, self.camreso[0])
        self.follow_til = Follower("tilt", self.tilcmd_pub, self.camreso[1])
        self.roi_minarea = self.roi_minarea * self.camreso[0] * self.camreso[1] / 100
        rospy.Subscriber("/roi", RegionOfInterest, self.roi_cb)
        rospy.Subscriber("/joint_states", JointState, self.jointstat_cb)
        dyncfg_srv = dynamic_reconfigure.server.Server(FollowerConfig, self.dyncfg_cb)
        
    def roi_cb(self, roi):
        if roi.width * roi.height > self.roi_minarea:
            x = roi.x_offset + roi.width / 2
            y = roi.y_offset + roi.height / 2
            self.follow_pan.follow(x, self.crrpos[1])
            self.follow_til.follow(y, self.crrpos[0])

    def jointstat_cb(self, msg):
        self.crrpos = msg.position

    def camera_info_cb(self, msg):
        self.camreso = (msg.width, msg.height)
        
    def dyncfg_cb(self, config, level):
        if self.camreso is not None:
            self.roi_minarea = config.roi_minarea * self.camreso[0] * self.camreso[1] / 100
        self.follow_pan.setparam(config.pan_gain, config.pan_dzone)
        self.follow_til.setparam(config.tilt_gain, config.tilt_dzone)
        return config
        
if __name__ == '__main__':
    try:
        node_name = "follower"
        FollowServo(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print"Shutting down " + str(node_name)
