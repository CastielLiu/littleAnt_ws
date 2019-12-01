#!/usr/bin/python  
from __future__ import print_function
import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import math
import os
import numpy as np
from little_ant_msgs.msg import Lane

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
global lane_msg
lane_msg=Lane()


def abs_sobel_thresh(img,orient='x',thresh_min=0,thresh_max=255):
    gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
    if orient == 'x':
        abs_sobel = np.absolute(cv2.Sobel(gray,cv2.CV_64F,1,0))
    if orient =='y':
        abs_sobel = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 0, 1))
        
    
    scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel))
    binary_output = np.zeros_like(scaled_sobel)
    
    
    binary_output[(scaled_sobel >= thresh_min) & (scaled_sobel <= thresh_max)] = 255
    
    return binary_output

def hls_select(img,channel='s',thresh=(0, 255)):
    hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
    if channel == 'h':
        channel = hls[:, :, 0]
    elif channel == 'l':
        channel = hls[:, :, 1]
    else:
        channel = hls[:, :, 2]
    binary_output = np.zeros_like(channel)
    binary_output[(channel > thresh[0]) & (channel <= thresh[1])] = 255
    return binary_output
    
def thresholding(img):
	x_thresh = abs_sobel_thresh(img, orient='x', thresh_min=35 ,thresh_max=100)
	
	#cv2.imshow("scaled_sobel",binary_output)
	hls_thresh_white = hls_select(img,channel='l', thresh=(80, 200))
	hls_thresh_yellow = hls_select(img,channel='s', thresh=(180, 255))
	#cv2.imshow("scaled_sobel",hls_thresh_yellow)
	threshholded = np.zeros_like(x_thresh)
	
	threshholded[(hls_thresh_white == 255) & (hls_thresh_yellow == 255) | (x_thresh == 255) ]=255
	return threshholded

def get_M_Minv():  
    src = np.float32([[(58, 480), (286, 223), (353, 223), (602, 480)]])
    dst = np.float32([[(160, 480), (160, 0), (480, 0), (480, 480)]])
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst,src)
    return M,Minv

# --------------------------- Define a lane class---------------------------------#
#-----------------------------------------------------------------------------------#
# Define a class to receive the characteristics of each line detection
class Line():
    def __init__(self):
        # was the line detected in the last iteration?
        self.detected = False
        # x values of the last n fits of the line
        self.recent_fitted = [np.array([False])]
        # average x values of the fitted line over the last n iterations
        self.bestx = None
        # polynomial coefficients averaged over the last n iterations
        self.best_fit = None
        # polynomial coefficients for the most recent fit
        self.current_fit = [np.array([False])]
        # radius of curvature of the line in some units
        self.radius_of_curvature = None
        # distance in meters of vehicle center from the line
        self.line_base_pos = None
        # difference in fit coefficients between last and new fits
        self.diffs = np.array([0, 0, 0], dtype='float')
        # x values for detected line pixels
        self.allx = None
        # y values for detected line pixels
        self.ally = None

    def check_detected(self):
        if (self.diffs[0] < 0.01 and self.diffs[1] < 10.0 and self.diffs[2] < 1000.) and len(self.recent_fitted) > 0:
            return True
        else:
            return False

    def update(self, fit):
        if fit is not None:
            if self.best_fit is not None:
                self.diffs = abs(fit - self.best_fit)
                if self.check_detected():
                    self.detected = True
                    if len(self.recent_fitted) > 10:
                        self.recent_fitted = self.recent_fitted[1:]
                        self.recent_fitted.append(fit)
                    else:
                        self.recent_fitted.append(fit)
                    self.best_fit = np.average(self.recent_fitted, axis=0)
                    self.current_fit = fit
                else:
                    self.detected = False
            else:
                self.best_fit = fit
                self.current_fit = fit
                self.detected = True
                self.recent_fitted.append(fit)

#------------------------find line-------------------------------------------#
#----------------------------------------------------------------------------#

def find_line(binary_warped):
	# Take a histogram of the bottom half of the image
	histogram = np.sum(binary_warped[binary_warped.shape[0] // 2:, :], axis=0) 
	# Find the peak of the left and right halves of the histogram
	# These will be the starting point for the left and right lines
	midpoint = np.int(histogram.shape[0] / 2)
	leftx_base = np.argmax(histogram[:midpoint])
	rightx_base = np.argmax(histogram[midpoint:]) + midpoint

	# Choose the number of sliding windows
	nwindows = 9
	# Set height of windows
	window_height = np.int(binary_warped.shape[0] / nwindows)
	# Identify the x and y positions of all nonzero pixels in the image
	nonzero = binary_warped.nonzero()
	nonzeroy = np.array(nonzero[0])
	nonzerox = np.array(nonzero[1])
	# Current positions to be updated for each window
	leftx_current = leftx_base
	rightx_current = rightx_base
	# Set the width of the windows +/- margin
	margin = 100
	# Set minimum number of pixels found to recenter window
	minpix = 50
	# Create empty lists to receive left and right lane pixel indices
	left_lane_inds = []
	right_lane_inds = []

	# Step through the windows one by one
	for window in range(nwindows):
		# Identify window boundaries in x and y (and right and left)
		win_y_low = binary_warped.shape[0] - (window + 1) * window_height
		win_y_high = binary_warped.shape[0] - window * window_height
		win_xleft_low = leftx_current - margin
		win_xleft_high = leftx_current + margin
		win_xright_low = rightx_current - margin
		win_xright_high = rightx_current + margin
		# Identify the nonzero pixels in x and y within the window
		good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
				          (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
		good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
				           (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
		# Append these indices to the lists
		left_lane_inds.append(good_left_inds)
		right_lane_inds.append(good_right_inds)
		# If you found > minpix pixels, recenter next window on their mean position
		if len(good_left_inds) > minpix:
			leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
		if len(good_right_inds) > minpix:
			rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

	# Concatenate the arrays of indices
	left_lane_inds = np.concatenate(left_lane_inds)
	right_lane_inds = np.concatenate(right_lane_inds)

	# Extract left and right line pixel positions
	leftx = nonzerox[left_lane_inds]
	lefty = nonzeroy[left_lane_inds]
	rightx = nonzerox[right_lane_inds]
	righty = nonzeroy[right_lane_inds]

	# Fit a second order polynomial to each
	left_fit = np.polyfit(lefty, leftx, 1)
	right_fit = np.polyfit(righty, rightx, 1)

	return left_fit, right_fit, left_lane_inds, right_lane_inds


def find_line_by_previous(binary_warped, left_fit, right_fit):
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    margin = 100
    left_lane_inds = ((nonzerox > (left_fit[0] * nonzeroy + left_fit[1] - margin)) & (nonzerox < (left_fit[0] * nonzeroy +
                                                                         left_fit[1] + margin)))

    right_lane_inds = ((nonzerox > (right_fit[0] * nonzeroy + right_fit[1] - margin)) & (nonzerox < (right_fit[0] * nonzeroy +
                                                                           right_fit[1] + margin)))

    # Again, extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]
    # Fit a second order polynomial to each
    left_fit = np.polyfit(lefty, leftx, 1)
    right_fit = np.polyfit(righty, rightx, 1)
    return left_fit, right_fit, left_lane_inds, right_lane_inds


#-------------------------------calculate the result-----------------------------------#
#------------------------------------------------------------------------------------#
def calculate_curv_and_pos(binary_warped, left_fit, right_fit):
	# Define y-value where we want radius of curvature
	
	#ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
	#leftx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
	#rightx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]
	#leftx = np.polyval(left_fit, ploty)
	#rightx = np.polyval(right_fit, ploty)
	# Define conversions in x and y from pixels space to meters
	#ym_per_pix = 20.61 / 480  # meters per pixel in y dimension

	#xm_per_pix = 2.84 / 320  # meters per pixel in x dimension

	# Fit new polynomials to x,y in world space

	#left_fit_cr = np.polyfit(ploty * ym_per_pix, leftx * xm_per_pix, 1)

	#right_fit_cr = np.polyfit(ploty * ym_per_pix, rightx * xm_per_pix, 1)

	# Calculate the new radii of curvature
	#left_curverad = ((1 + (2 * left_fit_cr[0] * y_eval * ym_per_pix + left_fit_cr[1]) ** 2) ** 1.5) / np.absolute(2 * left_fit_cr[0])
	#right_curverad = ((1 + (2 * right_fit_cr[0] * y_eval * ym_per_pix + right_fit_cr[1]) ** 2) ** 1.5) / np.absolute(2 * right_fit_cr[0])
	
	#curvature = ((left_curverad + right_curverad) / 2)
	#lane_width = np.absolute(leftx[479] - rightx[479])
	
	
	left_angle=math.atan(left_fit[0])
	right_angle=math.atan(right_fit[0])

	angle=((left_angle+right_angle)/2)
	
	height = binary_warped.shape[0]
	width = binary_warped.shape[1]
	
	left_bottom_x = np.polyval(left_fit, height)
	right_bottom_x = np.polyval(right_fit, height)
	
	lane_width = np.absolute(left_bottom_x - right_bottom_x)
	
	lane_xm_per_pix = 2.84 / lane_width
	
	veh_pos = ((left_bottom_x + right_bottom_x) * lane_xm_per_pix) / 2.
	cen_pos = (binary_warped.shape[1] * lane_xm_per_pix) / 2.
	distance_from_center = cen_pos - veh_pos
	
	return distance_from_center,angle


#-------------------------------display the result-----------------------------------#
#------------------------------------------------------------------------------------#
def draw_area(undist, binary_warped, Minv, left_fit, right_fit,angle):
	
	Ax = np.polyval(left_fit, 0)
	Bx = np.polyval(right_fit,0)
	Cx = np.polyval(right_fit,undist.shape[0])
	Dx = np.polyval(left_fit,undist.shape[0])
	
	A = [Ax,0]
	B = [Bx,0]
	C = [Cx,undist.shape[0]]
	D = [Dx,undist.shape[0]]
	
	rect = np.array([[A,B,C,D]]).astype(np.int)
	
	pure = np.zeros_like(undist)
	
	cv2.fillPoly(pure, rect, (0, 255, 0))
	
	# Warp the blank back to original image space using inverse perspective matrix (Minv)
	newwarp = cv2.warpPerspective(pure, Minv, (undist.shape[1], undist.shape[0]))
	# Combine the result with the original image

	result = cv2.addWeighted(undist, 1, newwarp, 0.3, 0)
	return result

def draw_values(img, distance_from_center,angle):
    font = cv2.FONT_HERSHEY_SIMPLEX
    #radius_text = "Radius of Curvature: %sm" % (round(curvature))


    if distance_from_center > 0:
        pos_flag = 'right'
    else:
        pos_flag = 'left'

    #cv2.putText(img, radius_text, (100, 100), font, 1, (255,0,255), 2)
    center_text = "Vehicle is %.3fm %s of center" % (abs(distance_from_center), pos_flag)
    cv2.putText(img, center_text, (100, 150), font, 1, (255, 0, 255), 2)
    #langle_text="left angle is:%.3f"%(left_angle*180/math.pi)
    #cv2.putText(img,langle_text,(100,200),font,1,(255,0,255),2)
    #rangle_text="right angle is:%.3f"%(right_angle*180/math.pi)
    #cv2.putText(img,rangle_text,(100,250),font,1,(255,0,255),2)
    angle_text="angle is %.3f"%(angle*180/math.pi)
    cv2.putText(img,angle_text,(100,200),font,1,(255,0,255),2)
    return img

#-----------------------------------------main process----------------------------------#
#---------------------------------------------------------------------------------------#
def processing(img, thresholded_wraped, Minv, left_line, right_line):
	thresholded = thresholding(thresholded_wraped)

	#cv2.imshow("thresholded",thresholded)
	#cv2.waitKey(1)

	if left_line.detected and right_line.detected:
		left_fit, right_fit, left_lane_inds, right_lane_inds = find_line_by_previous(thresholded,
		                                                                                      left_line.current_fit,
		                                                                                      right_line.current_fit)
	else:
		left_fit, right_fit, left_lane_inds, right_lane_inds = find_line(thresholded)
	#left_line.update(left_fit)
	#right_line.update(right_fit)

	pos_from_center,angle = calculate_curv_and_pos(thresholded, left_fit, right_fit)
	
	global lane_msg
	lane_msg.err = -pos_from_center
	lane_msg.theta = angle
	
	
	area_img = draw_area(img, thresholded, Minv, left_fit, right_fit,angle)
	result = draw_values(area_img,pos_from_center,angle)
    
	cv2.imshow("result",result)
	cv2.waitKey(1)

class image_converter:
  global lane_msg
  def __init__(self):
  
    self.image_pub = rospy.Publisher("/lane",Lane,queue_size=1)
   
 	
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/image_raw",Image,self.callback)
    
    self.M,self.Minv = get_M_Minv() 
 
  def callback(self,data):
    try:
      frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    thresholded_wraped = cv2.warpPerspective(frame,self.M, frame.shape[1::-1], flags=cv2.INTER_LINEAR)
    
    #cv2.imshow("2222",thresholded_wraped)

    left_lane=Line()
    right_lane=Line()
    result_video=processing(frame,thresholded_wraped,self.Minv,left_lane,right_lane)
    
    self.image_pub.publish(lane_msg)
	
def main(args):
  ic = image_converter()
  rospy.init_node('lane_detect')
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()
 
if __name__ == '__main__':
    main(sys.argv)
