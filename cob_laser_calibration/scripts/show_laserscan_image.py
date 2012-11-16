#!/usr/bin/env python
PKG = "cob_laser_calibration"
NODE = "laserscan_image_viewer"
import roslib
roslib.load_manifest(PKG)
import rospy
import cv
import cv2
from laserscan_to_image import LaserscanToImage
from sensor_msgs.msg import LaserScan
import numpy as np


class Laserscan_merge():
    def __init__(self):
        self.clear()

    def clear(self):
        self.result = LaserScan()
        self.counter = 0

    def append_laserscan(self, laserscan):
        if self.result.header.frame_id is not "":
            #print  self.result.header.frame_id
            #print laserscan.header.frame_id
            assert self.result.header.frame_id == laserscan.header.frame_id
            self.result.ranges = list((np.array(self.result.ranges) * self.counter +
                                       np.array(laserscan.ranges)) / (self.counter + 1))
            #print self.result.ranges
        else:
            self.result = laserscan
        self.counter += 1

    def get_count(self):
        return self.counter

    def get_scan(self):
       # print type(self.result)
       # print np.array(self.result.ranges).shape
        return self.result


class Listener():
    """docstring for Listener"""
    def __init__(self):
        self.laserscan = None
        self.laserscan_received = False

    def callback(self, data):
        self.laserscan = data
        self.laserscan_received = True

    def run(self):
        rospy.init_node(NODE)
        rospy.Subscriber("/scan_top", LaserScan, self.callback)
        res = 100
        lti = LaserscanToImage(res)
        merged_scan = Laserscan_merge()

        while not rospy.is_shutdown():
            while merged_scan.get_count() < 10:
                if self.laserscan_received is True:
                    merged_scan.append_laserscan(self.laserscan)
                    self.laserscan_received = False
            scan = merged_scan.get_scan()
            #print "Scan received"
            #print type(scan)
            image = lti.get_image(scan)
            #print "Image received"
            merged_scan.clear()
            if image is not None:
                gray = cv2.equalizeHist(cv2.cvtColor(image, cv.CV_BGR2GRAY))
                blurred = cv2.equalizeHist(cv2.GaussianBlur(gray, (3, 3), 0))
                mean = np.mean(blurred)
                canny = cv2.Canny(blurred, 0.66 * mean, 1.33 * mean)
                lines = cv2.HoughLines(gray, res / 100, cv.CV_PI / 80,
                                       round(res * 0.35))
                circles = cv2.HoughCircles(
                    gray, cv.CV_HOUGH_GRADIENT, 2, res * 0.40, param1=200, param2=40, maxRadius=round(res * 0.40))
                lines = [lines[0]]
                #print len(lines[0])
                corners = cv2.goodFeaturesToTrack(gray, 50, 0.2, 0.05 * res)
                #print len(corners)
                for corner in corners:
                    print corner

                    x = corner[0, 0]
                    y = corner[0, 1]
                    x = int(x)
                    y = int(y)
                    cv2.rectangle(
                        image, (x - 3, y - 3), (x + 3, y + 3), (0, 0, 255))
                #lines = merge_lines(lines[0])
                #print lines
                if circles is None:
                    print "No Circles detected"
                    break
                if lines is None:
                    print "No Lines detected"
                    break
                bestfit = detect_pattern(circles[0], lines[0])
                rho, theta = bestfit
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = x0 + 1000 * (-b)
                y1 = y0 + 1000 * a
                x2 = x0 - 1000 * (-b)
                y2 = y0 - 1000 * a

                cv2.line(image, (x1, y1), (x2, y2), (0, 0, 255), 2)

                if circles is not None:
                    for x, y, r in circles[0]:
                        cv2.circle(image, (x, y), r, (255, 0, 255))
                        cv2.circle(canny, (x, y), r, (255, 255, 255))
                        cv2.circle(blurred, (x, y), r, (255, 255, 255))

                cv.ShowImage("image1", cv.fromarray(image))

                cv.WaitKey()


def get_parameters(line):
    (rho, theta) = line
    alpha = np.pi / 2 - theta
    print 'alpha = ', alpha
    c = rho / np.cos(alpha)

    b = -1
    a = np.tan(-alpha)
    print [a, b, c]
    return [a, b, c]


def get_distance_point_line(point, line):
    print 'line = ', line
    print 'point = ', point
    return (np.absolute(line[0] * point[0] + line[1] * point[1] + line[2]) / np.sqrt(line[0] ** 2 + line[1] ** 2))


def detect_pattern(circles, lines):
    error_line = []
    for l in lines:
        p = get_parameters(l)
        error = 0
        for c in circles:
            center = [c[0], c[1]]
            d = get_distance_point_line(center, p) ** 2
            print 'distance = ', np.sqrt(d)
            error += d
        error_line.append([np.sqrt(error), [l]])
    for e in error_line:
        print e
    error_line.sort()
    best_fit = error_line[0][1]
    print best_fit
    return best_fit[0]

if __name__ == "__main__":
    l = Listener()
    l.run()
    #print get_parameters([0,0])
    #print get_parameters([0,np.pi/2])
    #print get_parameters([0,np.pi/4])
    #assert get_parameters([0,0])==[0,-1,0]
    #assert get_parameters([0,np.pi/2])==[1,0,0]
    #assert get_parameters([0,np.pi/4])==[1,-1,0]
    #print get_distance_point_line([5,6],[-2,3,4])
