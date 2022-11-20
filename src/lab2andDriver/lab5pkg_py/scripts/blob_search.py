#!/usr/bin/env python

import cv2
import numpy as np
import math

# ========================= Student's code starts here =========================

# Params for camera calibration
#theta = 0.0412523    #radians
theta = 0.0    #radians
beta = 75/0.1        # pixel/meter
# tx = -160            # pixels
# ty = -90             # pixels
tx = -219            # pixels
ty = -79             # pixels

# Function that converts image coord to world coord
def IMG2W(col, row):
    Or = 240
    Oc = 320
    
    # (x_pixel, y_pixel) in camera's frame
    xc = row-Or
    yc = col-Oc

    # (x_meter, y_meter) in world frame
    yw = (yc - ty)/beta
    xw = (xc - tx)/beta
    
    print("Input = {}".format([col,row]))
    gama = math.atan(xw/yw)
    alpha = gama - theta
    
    # debug code
    print("xw = {}".format(xw))
    print("yw = {}".format(yw))
    print("gama = {}".format(gama))
    print("alpha = {}".format(alpha))
    
    l = (yw**2 + xw**2)**0.5
    
    yw_rot = l * math.cos(alpha)
    xw_rot = l * math.sin(alpha)
    return [xw_rot,yw_rot]

# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    # Filter by Color
    params.filterByColor = False
    
    params.minThreshold = 0
    params.maxThreshold = 50
    
    # Filter by Area.
    params.filterByArea = True
    ## callibration bar ##
    # params.minArea = 200

    # screw_M8
    params.minArea = 10
    params.maxArea = 1000

    # Filter by Circularity  #square is 0.785
    params.filterByCircularity = True
    params.minCircularity = 0.5
    params.maxCircularity = 1

    # Filter by Inerita
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = False

    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)
    
    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    # ========================= Student's code starts here =========================

    lower_blue = (110,50,50)     # blue lower
    upper_blue = (130,255,255)   # blue upper

    if color == "green":
        lower_green = (40,50,50)     # green lower
        upper_green = (80,255,255)   # green upper
        mask_image = cv2.inRange(hsv_image, lower_green, upper_green)
    elif color == "yellow":
        lower_yellow = (8,100,100)    # yellow lower
        upper_yellow = (25,255,255)  # yellow upper
        mask_image = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
        
    ######## This is for camera callibration  #########
    # lower_orange = (5,200,100)
    # upper_orange = (15,255,255)
    # # Define a mask using the lower and upper bounds of the target color
    # mask_image = cv2.inRange(hsv_image, lower_orange, upper_orange)

    # ========================= Student's code ends here ===========================

    keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))
    
    # Camera extrinsic callibration
    def calculate():
        # Calculate beta ---- pixels/meter
        if len(blob_image_center) == 2 :
            beta = ((blob_image_center[0][0]-blob_image_center[1][0])**2 + (blob_image_center[0][1]-blob_image_center[1][1])**2)**0.5
            print("The beta = {}".format(beta))
            
            theta = math.acos((blob_image_center[0][0]-blob_image_center[1][0])/beta)
            print("The theta = {}".format(theta))
                    
    # calculate()
    print(blob_image_center)
        
    # ========================= Student's code starts here =========================

    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # ========================= Student's code ends here ===========================

    xw_yw = []

    if(num_blobs == 0):
        # print("No block found!")
        pass
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(num_blobs):
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))
            if color == "green":
                print("Green xw_yw is {}".format(xw_yw))
            elif color == "yellow":
                print("Yellow xw_yw is {}".format(xw_yw))
                

    # cv2.namedWindow("Camera View")
    # cv2.imshow("Camera View", image_raw)
    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", mask_image)
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)

    cv2.waitKey(2)

    return xw_yw
