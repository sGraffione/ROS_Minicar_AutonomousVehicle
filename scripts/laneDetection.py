#!/usr/bin/env python3
import rospy
import numpy as np
import pandas as pd
import cv2 as cv
import os
import glob
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import pickle
from scipy.signal import find_peaks
import time
from minicar.msg import LaneDetection

centerError = 0

# Initialize the perspective transformation matrix
M = []
Minv = []

# camera image size and ROI size
height = 480
heightROI = height-int(height/1.5)
width = 640

# Load undistortion parameters from pickle file
cal_dir='camera_cal/cal_pickle.p'
with open(cal_dir, mode='rb') as f:
        file = pickle.load(f)
mtx = file['mtx']
dist = file['dist']
undist_map1, undist_map2 = cv.initUndistortRectifyMap(mtx, dist, None, None, (width, height),cv.CV_32FC1)

def undistort_img():
    # Prepare object points 0,0,0 ... 8,5,0
    obj_pts = np.zeros((6*9,3), np.float32)
    obj_pts[:,:2] = np.mgrid[0:9, 0:6].T.reshape(-1,2)

    # Stores all object points & img points from all images
    objpoints = []
    imgpoints = []

    # Get directory for all calibration images
    images = glob.glob('camera_cal/*.jpg')

    for indx, fname in enumerate(images):
        img = cv.imread(fname)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        ret, corners = cv.findChessboardCorners(gray, (9,6), None)

        if ret == True:
            objpoints.append(obj_pts)
            imgpoints.append(corners)
    # Test undistortion on img
    img_size = (img.shape[1], img.shape[0])

    # Calibrate camera
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, img_size, None,None)

    dst = cv.undistort(img, mtx, dist, None, mtx)
    # Save camera calibration for later use
    dist_pickle = {}
    dist_pickle['mtx'] = mtx
    dist_pickle['dist'] = dist
    pickle.dump( dist_pickle, open('camera_cal/cal_pickle.p', 'wb') )

def undistort(img):
    dst = cv.remap(img, undist_map1, undist_map2, interpolation=cv.INTER_NEAREST, borderMode=cv.BORDER_CONSTANT, borderValue=(0, 0, 0, 0))
   # dst = cv.undistort(img, mtx, dist, None, mtx)
    return dst

def pipeline(img, s_thresh=(15, 25), sx_thresh=(10, 255), h_thresh=(15, 25)):
    
    # Convert to HLS color space and separate the V channel
    hls = cv.cvtColor(img, cv.COLOR_RGB2HLS)
    l_channel = hls[:,:,1]
    s_channel = hls[:,:,2]
    h_channel = hls[:,:,0]
#     plt.imshow(l_channel)
#     plt.show()
#     plt.imshow(h_channel)
#     plt.show()
    # Sobel x
    sobelx = cv.Sobel(l_channel, cv.CV_64F, 1, 1) # Take the derivative in x
    abs_sobelx = np.absolute(sobelx) # Absolute x derivative to accentuate lines away from horizontal
    scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx))
#     plt.imshow(scaled_sobel)
#     plt.show()
    # Threshold x gradient
    sxbinary = np.zeros_like(scaled_sobel)
    sxbinary[(scaled_sobel >= sx_thresh[0]) & (scaled_sobel <= sx_thresh[1])] = 1
#     cv.imshow("lane",scaled_sobel)
    # Threshold color channel
#     s_binary = np.zeros_like(s_channel)
#     s_binary[(s_channel >= s_thresh[0]) & (s_channel <= s_thresh[1])] = 1
#     h_binary = np.zeros_like(h_channel)
#     h_binary[(h_channel >= h_thresh[0]) & (h_channel <= h_thresh[1])] = 1
# 
#     plt.imshow(s_binary*255)
#     plt.show()
#     plt.imshow(h_binary*255)
#     plt.show()
#     plt.imshow(s_binary & h_binary)
#     plt.show()
#     color_binary = np.dstack((np.zeros_like(sxbinary), sxbinary, s_binary&h_binary)) * 255
#     
#     combined_binary = np.zeros_like(sxbinary)
#     combined_binary[(s_binary == 1) | (sxbinary == 1)] = 1
    return sxbinary


def perspective_warp(dst_size=(640,480),
                     src=np.float32([(1, 0), (0,0), (0,1), (1,1)]),
                     dst=np.float32([(1,0),(0,0),(0.35,1),(0.65,1)])):
    global M, Minv
    
    img_size = np.float32([dst_size])
    src = src* img_size
    
    dst = dst * np.float32(dst_size)
    # Given src and dst points, calculate the perspective transform matrix
    M = cv.getPerspectiveTransform(src, dst)
    Minv = cv.getPerspectiveTransform(dst, src)
    # Warp the image using OpenCV warpPerspective()

def get_hist(img):
#     plt.imshow(img[120:180,:])
#     plt.show()
    hist = np.sum(img[100:180,:], axis=0)
    hist = np.convolve(hist,np.ones(25,dtype=int),'valid')
#     plt.plot(hist)
#     plt.show()
    return hist

left_a, left_b, left_c = [],[],[]
right_a, right_b, right_c = [],[],[]
left_fitx = []
right_fitx = []
center_fitx = []
prev_peak_pos = []
left_fit_ = np.empty(3)
right_fit_ = np.empty(3)

def sliding_window(img, nwindows=9, steps=2, margin=30, minpix = 1, draw_windows=True):
    # steps sono il numero di passi che la finestra i-esima deve fare prima di poter passare alla finestra i+1
    # il numero di finestre effettive sono quindi nwindows*steps ma ognuna ha una parte condivisa con quella precedente e successiva
    global left_a, left_b, left_c, right_a, right_b, right_c, left_fit_, right_fit_, right_fitx, left_fitx, right_fitx

    estimatesRight = False
    estimatesLeft = False
    leftx_current = np.empty(nwindows*steps+1)
    rightx_current = np.empty(nwindows*steps+1)

    ploty = np.linspace(0, img.shape[0] - 1, img.shape[0])
    xm_per_pix = 0.3 / img.shape[1]  # meters per pixel in x dimension
    nominal_lane_width = 0.17 / xm_per_pix  # 18 centimeters

    histogram = get_hist(img)
    # find peaks of left and right halves
    peaks = find_peaks(histogram, height=1, threshold=0, distance=100)
    peaks_height = peaks[1]['peak_heights']  # list of the heights of the peaks
    
    # Indices of N largest elements in list
    # using sorted() + lambda + list slicing
    indexes = sorted(range(len(peaks_height)), key = lambda sub: peaks_height[sub])[-2:]
    peak_pos = (np.linspace(0, img.shape[1], img.shape[1] - 1)[peaks[0]])[indexes] # list of the peaks positions

    if len(peak_pos) == 1:
        diff1 = np.abs(peak_pos[0]-prev_peak_pos[0]) # diff rispetto alla precedente sinistra
        diff2 = np.abs(peak_pos[0]-prev_peak_pos[1]) # diff rispetto alla precedente destra
        if  diff1 < diff2: # quella che ho è la sinistra
            estimatesRight = True
            # Current positions to be updated for each window
            leftx_current[0] = peak_pos[0]
            rightx_current[0] = peak_pos[0]+nominal_lane_width
        else: # quella che ho è la sinistra
            estimatesLeft = True
            # Current positions to be updated for each window
            rightx_current[0] = peak_pos[0]
            leftx_current[0] = nominal_lane_width-peak_pos[0]
    elif len(peak_pos) == 0: # se non ho niente, usa la curva stimata precendentemente
        return (left_fitx, right_fitx), ploty, center_fitx
    else:
        # Current positions to be updated for each window
        leftx_current[0] = peak_pos[0]
        rightx_current[0] = peak_pos[1]

    # Set height of windows
    window_height = int(img.shape[0] / nwindows)
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = img.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])

    win_y_low = np.zeros(nwindows*steps)

    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []
    if draw_windows == True:
        plt.close('all')
        fig, ax = plt.subplots()
        ax.imshow(img)

    if estimatesRight & estimatesLeft: # both lines are not visible
        print("Can't see shit")
    elif estimatesLeft: #only the right one is visible
        for window in range(nwindows * steps):
            # Identify window boundaries in x and y (and right and left)
            win_y_low[window] = img.shape[0] - (window + 1) * window_height / steps
            win_y_high = img.shape[0] - window * window_height / steps

            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin

            nexty = img.shape[0] - (
                    window + 2) * window_height / steps  # next y coordinate of the window (may be usefull if not enough points are found)
            # Draw the windows on the visualization image
            if draw_windows == True:
                ax.add_patch(
                    Rectangle((win_xright_low, win_y_low), margin * 2, window_height, fc='none', color='yellow',
                              linewidth=1))

            # Identify the nonzero pixels in x and y within the window
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
            # Append these indices to the lists
            right_lane_inds.append(good_right_inds)

            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_right_inds) > minpix:
                rightx_current[window] = int(np.mean(nonzerox[good_right_inds]))
            else:
                if len(rightx_current) >= 2:
                    right_fit = np.polyfit(win_y_low, rightx_current, 2)
                    right_a.append(right_fit[0])
                    right_b.append(right_fit[1])
                    right_c.append(right_fit[2])
                    right_fit_[0] = np.mean(right_a[-10:])
                    right_fit_[1] = np.mean(right_b[-10:])
                    right_fit_[2] = np.mean(right_c[-10:])
                rightx_current[window] = right_fit_[0] * nexty ** 2 + right_fit_[1] * nexty + right_fit_[2]
                leftx_current[window] = rightx_current-nominal_lane_width
    elif estimatesRight: # only the left one is visible
        for window in range(nwindows * steps):
            # Identify window boundaries in x and y (and right and left)
            win_y_low[window] = img.shape[0] - (window + 1) * window_height / steps
            win_y_high = img.shape[0] - window * window_height / steps

            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin

            nexty = img.shape[0] - (
                    window + 2) * window_height / steps  # next y coordinate of the window (may be usefull if not enough points are found)
            # Draw the windows on the visualization image
            if draw_windows == True:
                ax.add_patch(Rectangle((win_xleft_low, win_y_low), margin * 2, window_height, fc='none', color='yellow',
                                       linewidth=1))

            # Identify the nonzero pixels in x and y within the window
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)

            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > minpix:
                leftx_current[window] = int(np.mean(nonzerox[good_left_inds]))
            else:
                if len(leftx_current) >= 2:
                    left_fit = np.polyfit(win_y_low, leftx_current, 2)
                    left_a.append(left_fit[0])
                    left_b.append(left_fit[1])
                    left_c.append(left_fit[2])
                    left_fit_[0] = np.mean(left_a[-10:])
                    left_fit_[1] = np.mean(left_b[-10:])
                    left_fit_[2] = np.mean(left_c[-10:])
                leftx_current[window] = left_fit_[0] * nexty ** 2 + left_fit_[1] * nexty + left_fit_[2]
                rightx_current[window] = leftx_current+nominal_lane_width

    else: # both are visible
        for window in range(nwindows * steps):
            # Identify window boundaries in x and y (and right and left)
            win_y_low[window] = img.shape[0] - (window + 1) * window_height / steps
            win_y_high = img.shape[0] - window * window_height / steps

            win_xleft_low = leftx_current[window] - margin
            win_xleft_high = leftx_current[window] + margin

            win_xright_low = rightx_current[window] - margin
            win_xright_high = rightx_current[window] + margin

            nexty = img.shape[0] - (
                    window + 2) * window_height / steps  # next y coordinate of the window (may be usefull if not enough points are found)
            
            # Draw the windows on the visualization image
            if draw_windows == True:
                ax.add_patch(Rectangle((win_xleft_low, win_y_low[window]), margin * 2, window_height, fc='none', color='yellow',
                                       linewidth=1))
                ax.add_patch(Rectangle((win_xright_low, win_y_low[window]), margin * 2, window_height, fc='none', color='yellow',
                              linewidth=1))

            # Identify the nonzero pixels in x and y within the window
            good_left_inds = ((nonzeroy >= win_y_low[window]) & (nonzeroy < win_y_high) &
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low[window]) & (nonzeroy < win_y_high) &
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > minpix:
                leftx_current[window+1] = int(np.mean(nonzerox[good_left_inds]))
            else:
                if window >= 2:
                    left_fit = np.polyfit(win_y_low[0:window], leftx_current[0:window], 2)
                    left_a.append(left_fit[0])
                    left_b.append(left_fit[1])
                    left_c.append(left_fit[2])
                    left_fit_[0] = np.mean(left_a[-10:])
                    left_fit_[1] = np.mean(left_b[-10:])
                    left_fit_[2] = np.mean(left_c[-10:])
                leftx_current[window+1] = left_fit_[0] * nexty ** 2 + left_fit_[1] * nexty + left_fit_[2]

            if len(good_right_inds) > minpix:
                rightx_current[window+1] = int(np.mean(nonzerox[good_right_inds]))
            else:
                if window >= 2:
                    right_fit = np.polyfit(win_y_low[0:window], rightx_current[0:window], 2)
                    right_a.append(right_fit[0])
                    right_b.append(right_fit[1])
                    right_c.append(right_fit[2])
                    right_fit_[0] = np.mean(right_a[-10:])
                    right_fit_[1] = np.mean(right_b[-10:])
                    right_fit_[2] = np.mean(right_c[-10:])
                rightx_current[window+1] = right_fit_[0] * nexty ** 2 + right_fit_[1] * nexty + right_fit_[2]
       
#     left_lane_inds = np.concatenate(left_lane_inds)
#     right_lane_inds = np.concatenate(right_lane_inds)
# 
#     leftx = nonzerox[left_lane_inds]
#     lefty = nonzeroy[left_lane_inds]
#     rightx = nonzerox[right_lane_inds]
#     righty = nonzeroy[right_lane_inds]
# 
#     left_fit = np.polyfit(lefty, leftx, 2)
#     right_fit = np.polyfit(righty, rightx, 2)

    left_fit = np.polyfit(win_y_low, leftx_current[:nwindows*steps], 2)
    right_fit = np.polyfit(win_y_low, rightx_current[:nwindows*steps], 2)

    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
    center_fitx = (right_fitx + left_fitx)/2
    if draw_windows:
        plt.show()
    return (left_fitx, right_fitx), ploty, center_fitx

def get_curve(img, leftx, rightx, centerx):
    ploty = np.linspace(0, img.shape[0]-1, img.shape[0])
    y_eval = np.max(ploty)
    ym_per_pix = 0.7/img.shape[0] # meters per pixel in y dimension
    xm_per_pix = 0.3/img.shape[1] # meters per pixel in x dimension

    # Fit new polynomials to x,y in world space
    left_fit_cr = np.polyfit(ploty*ym_per_pix, leftx*xm_per_pix, 2)
    right_fit_cr = np.polyfit(ploty*ym_per_pix, rightx*xm_per_pix, 2)
    # Calculate the new radii of curvature
    left_curverad = np.absolute(2*left_fit_cr[0]) / ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5)
    right_curverad = np.absolute(2*right_fit_cr[0]) / ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5)

    car_pos = img.shape[1]/2
    l_fit_x_int = left_fit_cr[0]*img.shape[0]**2 + left_fit_cr[1]*img.shape[0] + left_fit_cr[2]
    r_fit_x_int = right_fit_cr[0]*img.shape[0]**2 + right_fit_cr[1]*img.shape[0] + right_fit_cr[2]
    
    lane_center_position = (r_fit_x_int + l_fit_x_int) /2
    center = (centerx[-1] - car_pos) * xm_per_pix * 100
    # Now our radius of curvature is in meters
    
    return (left_curverad, right_curverad, center)

def draw_lanes(img, left_fit, right_fit, center_fit):
    ploty = np.linspace(0, img.shape[0]-1, len(left_fit))
    color_img = np.zeros_like(img)
    left = np.array([np.transpose(np.vstack([left_fit, ploty]))],np.int32)
#     print(left)
#     plt.imshow(img)
#     plt.show()
    right = np.array([np.flipud(np.transpose(np.vstack([right_fit, ploty])))],np.int32)
    points = np.hstack((left, right))
    center_pts = np.array([np.transpose(np.vstack([center_fit, ploty]))],np.int32)
    cv.fillPoly(color_img, np.int_(points), (0,200,255))
    cv.polylines(color_img, center_pts, False, (0,200,255), 2)
#     cv.polylines(color_img, left, False, (255,200,0), 2)
#     cv.polylines(color_img, right, False, (255,200,0), 2)
#     plt.imshow(color_img)
#     plt.show()
    inv_perspective = cv.warpPerspective(color_img, Minv, (img.shape[1], img.shape[0]))
    inv_perspective = cv.addWeighted(img, 1, inv_perspective, 0.7, 0)
    return inv_perspective

def roi(img):
    height = img.shape[0]
    masked_image = img[int(height/1.5):height,:]
    return masked_image

def pipeline_gaussian(img, s_thresh=(20, 40), sx_thresh=(50, 255)):
    img = np.copy(img)
    w,h = (img.shape[1], img.shape[0])

    # Convert to HLS color space and separate the V channel
    gray = (cv.cvtColor(img, cv.COLOR_RGB2GRAY))

    # Difference of Gaussians to enhance borders (https://stackoverflow.com/questions/59516492/difference-of-gaussian-filtering-dog-doesnt-give-the-expected-results)
    # set arguments
    gamma = 0.2
    alpha = 0.1
    tau = 3.0
    
    # gamma correction
    img_gamma = np.power(gray, gamma)
    blur1 = cv.GaussianBlur(img_gamma, (0,0), 1, borderType=cv.BORDER_REPLICATE)
    blur2 = cv.GaussianBlur(img_gamma, (0,0), 2, borderType=cv.BORDER_REPLICATE)
    img_dog = (blur1 - blur2)
    # normalize by the largest absolute value so range is -1 to 
    img_dog = img_dog / np.amax(np.abs(img_dog))
    
    # contrast equalization equation 1
    img_contrast1 = np.abs(img_dog)
    img_contrast1 = np.power(img_contrast1, alpha)
    img_contrast1 = np.mean(img_contrast1)
    img_contrast1 = np.power(img_contrast1,1.0/alpha)
    img_contrast1 = img_dog/img_contrast1
    
    # contrast equalization equation 2
    img_contrast2 = np.abs(img_contrast1)
    img_contrast2 = img_contrast2.clip(0,tau)
    img_contrast2 = np.mean(img_contrast2)
    img_contrast2 = np.power(img_contrast2,1.0/alpha)
    img_contrast2 = img_contrast1/img_contrast2
    img_contrast = tau * np.tanh((img_contrast2/tau))
    
    # Scale results two ways back to uint8 in the range 0 to 255
    img_contrastA = (255.0 * (img_contrast+0.5)).clip(0,255).astype(np.uint8)
    
#     plt.imshow(img_contrastA)
#     plt.show()

    edge = cv.Canny(img_contrastA,700,750)
    binary = np.zeros_like(edge)
    binary[edge==255]=1

    hls = (cv.cvtColor(img, cv.COLOR_RGB2HLS))
    s_channel = hls[:,:,2]
    
    '''plt.imshow(l_channel)
    plt.show()
    plt.imshow(s_channel)
    plt.show()'''

    s_binary = np.zeros_like(s_channel)
    s_binary[(s_channel >= s_thresh[0]) & (s_channel <= s_thresh[1])] = 1

    combined_binary = np.zeros_like(s_channel)
    combined_binary[(s_binary == 1) | (binary == 1)] = 1

    return combined_binary

def main():
	rospy.init_node('laneDetection', anonymous=True)

	pub = rospy.Publisher("laneDetection", Motors, queue_size=1)
	rate = rospy.Rate(10)
    
	laneDect = LaneDetection()
	
	global centerError
	
	rospy.loginfo("Initializing..")
	# Initialize the warping matrices
	perspective_warp()
	
	rospy.loginfo("Opening camera")
	cap = cv.VideoCapture(0)
	cap.set(cv.CAP_PROP_FRAME_WIDTH, width)
	cap.set(cv.CAP_PROP_FRAME_HEIGHT, height)
	#     cap.set(cv.CAP_PROP_FPS, 10)
	
	times = np.zeros([7,1])
	matr = np.array([[-1, 1, 0, 0, 0, 0, 0],
					[0, -1, 1, 0, 0, 0, 0],
					[0, 0, -1, 1, 0, 0, 0],
					[0, 0, 0, -1, 1, 0, 0],
					[0, 0, 0, 0, -1, 1, 0],
					[0, 0, 0, 0, 0, -1, 1]])
	Kp = 2
	Kd = 0
	Ki = 0
	I = 0
	prevError = 0
	meanErr = [0,0,0,0,0,0,0,0,0,0]
	vect = [0.04, 0.08, 0.12, 0.2, 0.45, 0.7, 0.88, 0.92, 0.96, 1]
	draw_result = False
	
	rospy.loginfo("Starting...")
	while not rospy.is_shutdown():
		while(cap.isOpened()):
			
			# Read one frame from the videofile
			ret, image = cap.read()
	#         start_time = time.time()
	#         times[0] = start_time
			image = undistort(image)
	#         times[1] = time.time()
			roi_image = roi(image)
	#         times[2] = time.time()
			pipe = pipeline_gaussian(roi_image)
			plt.imshow(pipe)
			plt.show()
	#         times[3] = time.time()
			warped = cv.warpPerspective(pipe, M, (width, heightROI))
	#         times[4] = time.time()
			curves, ploty, center_fit = sliding_window(warped, draw_windows=False)
	#         times[5] = time.time()
			curverad =get_curve(warped, curves[0], curves[1], center_fit)
	#         times[6] = time.time()
			
			if draw_result:
				result = draw_lanes(roi_image, curves[0], curves[1], center_fit)
				cv.imshow("frame",result)
			centerError = curverad[2]
			
			avg = sum(meanErr) / 10
			mahalaDist = np.sqrt(0.1*(centerError-avg)**2)
			if mahalaDist > 0.3:
				centerError = prevError
			else:
				meanErr.append(centerError)
				meanErr.pop(0)
			centerError = np.dot(vect,meanErr)
			
	#         delta = Kp*centerError
	#         diff=np.dot(matr,(times-start_time))
	#         print("times"+str(diff))
	#         print("total"+str(np.sum(diff)))
			#print(str(centerError))
			laneDect.lateralError = centerError
			laneDect.curvature = 0
			pub.publish(laneDect)
			rate.sleep()
	
main()
	
