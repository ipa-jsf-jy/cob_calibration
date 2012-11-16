#!/usr/bin/env python
PKG = "cob_laser_calibration"
import roslib
roslib.load_manifest(PKG)
from sensor_msgs.msg import LaserScan
from math import sin, cos
import numpy as np


class LaserscanToImage():
    """Converts Laserscan Message to Gigapixel Image"""
    def __init__(self, resolution):
        ''' Resolution in pixel per meter '''
        self.resolution = resolution
        self.laserscan = None
        self.image = None

    def set_laserscan(self, laserscan):
        if isinstance(laserscan, LaserScan):
            self.laserscan = laserscan
            self.image = None
        else:
            return

    def calculate_angles(self):
        min_angle = self.laserscan.angle_min
        max_angle = self.laserscan.angle_max
        angle_increment = self.laserscan.angle_increment
        self.angles = [min_angle]
        while self.angles[-1] <= max_angle:
            self.angles.append(self.angles[-1] + angle_increment)

    def calculate_carthesian(self):
        self.x = []
        self.y = []
        for r, phi in zip(self.laserscan.ranges, self.angles):
            self.x.append(round(r * cos(phi) * self.resolution)
                          + self.resolution * self.laserscan.range_max)
            self.y.append(round(r * sin(phi) * self.resolution)
                          + self.resolution * self.laserscan.range_max)

    def fill_image(self):
        self.center = (self.resolution * self.laserscan.range_max,
                       self.resolution * self.laserscan.range_max)
        dx = max(self.center[0] - min(self.x), max(self.x) - self.center[0])
        dy = max(self.center[1] - min(self.y), max(self.y) - self.center[1])
        x_roi = (self.center[0] - dx, self.center[0] + dx)
        y_roi = (self.center[1] - dy, self.center[1] + dy)

        self.image = np.zeros((1 + 2 * dx, 1 + 2 * dy, 3), np.uint8)
        pt_old = None
        for x, y in zip(self.x, self.y):
            x -= x_roi[0]
            y -= y_roi[0]
            pt = (x, y)
            #print pt, pt_old
            self.interpolate(pt, pt_old)
            pt_old = pt

    def interpolate(self, pt1, pt2):
        if pt2 is None:
            pt2 = pt1
        dx = (pt1[0] - pt2[0])
        dy = (pt1[1] - pt2[1])
        max_d = max(np.absolute(dx), np.absolute(dy))
        if max_d != 0:
            dsx = dx / max_d
            dsy = dy / max_d
        else:
            dsx = 0
            dsy = 0
        #print pt1, pt2
        for i in range(round(max_d + 1)):
            #print pt2
            #print dsx
            #print i
            x = pt2[0] + (dsx * i)
            y = pt2[1] + (dsy * i)
            #print x,y
            pt = (x, y)
            self.draw_point(pt)

    def draw_point(self, pt):
        #print pt
        self.image[round(pt[0]), round(pt[1]), 0] = 255

    def get_image(self, laserscan=None):
        if laserscan is not None:
            self.set_laserscan(laserscan)
            self.calculate_angles()
            self.calculate_carthesian()
            self.fill_image()
        elif self.laserscan is None:
            print "[ERROR] No Laserscan set"
        return self.image
