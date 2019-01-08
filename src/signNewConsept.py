#!/usr/bin/python

#import
from __future__ import print_function
from scipy.stats import itemfreq
from math import sqrt
import cv2
print (cv2.__version__)
import numpy as np
import argparse



# Konstanten
filename = '/home/pses/catkin_ws/src/echtzeitsysteme/images/signs.jpg'
#filename = '/home/pses/catkin_ws/src/echtzeitsysteme/images/ErgebnisTmp.jpg'
#filename = '/home/pses/catkin_ws/src/echtzeitsysteme/images/2018-12-05-220138.jpg'



############# Einlesen  ###################
# Image aus File lesen
img = cv2.imread(filename)

# Image lesen und Grauzeichnen
# img = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)


# grayscale
    # Eingehendes Bild in den Grauen Farbraum transferieren
    # erhoeht die Geschwindigkeit der Erkennung
    # kein Unterschied ob das Schild grau oder bunt ist
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #BGR 3 Values per Pixel (Blue, Green, Red)


# contrast and brightness changing for every Pixel -> neues Mat Object
transInput = img
alphaContrast =  1.2   #default 1.0
betaBrightness = 0   #default 0
contrastBrightnessOutput = np.zeros(transInput.shape, transInput.dtype) # initial pixels are Zero # Same size and type as Original Image
""" zu langsam
for y in range(transInput,shape[0]):
    for x in range(transInput.shape[1]):
        for c in range(transInput.shape[2]):
            contrastBrightnessOutput[y,x,c] = np.clip[alphaContrast*transInput[y,x,c] + betaBrightness, 0 ,255)
"""
    #optimierter
contrastBrightnessOutput = cv2.convertScaleAbs(transInput,alpha=alphaContrast, beta=betaBrightness)



# Gamma Transformation    // non linear 0<y<255
# y < 1    original dark regions will be brighter
# y > 1    original dark regions will be darker
# looluptable to improve performance only 256 values
gamma = 0.8
gammaImageInput = img 
lookUpTable = np.empty((1,256), np.uint8)
for i in range(256):
    lookUpTable[0,i] = np.clip(pow(i/255.0, gamma)*255.0, 0, 255)
gammaImgOutput = cv2.LUT(gammaImageInput, lookUpTable)




# cannyDetection/Kanten
ratio = 3               # lower:upper Schwellwert 3:1
kernel_size = 3     
img_gray = gray    
def CannyThreshold(val):
    low_threshold = val
    imgBlurCanny = cv2.blur(img_gray, (3,3))
    detected_edges = cv2.Canny(imgBlurCanny, low_threshold, low_threshold*ratio, kernel_size)
    mask = detected_edges != 0
    dst = img * (mask[:,:,None].astype(img.dtype))
    # dst     -->   ausgeschnittenes Bild nur Kreise       Schwellwert 21 optimal        ####################
    cv2.imshow(window_name, dst)




# hough circle Transform
imgCircles = img
# medianBlur() -> Rauschunterdrueckung des Streams um falsche Kreiserkennung zu verhindern 
    # https://docs.opencv.org/3.1.0/d4/d13/tutorial_py_filtering.html
imgBlurCircles = cv2.medianBlur(gray, 5)
rows = imgBlurCircles.shape[0]


# HoughCircles(InputArray Bild, OutputArray cv2.HOUGH_GRADIENT, int methode, double minDist, ) 
    # Hough Transformation um Kreise in einem grayscale Bild zu finden
    # minDist: minimaler euklidischer Abstand zwischen Mittelpunkt des erkannten Kreises und dessen Rand/Kanten
    #           -> nur ein Schild immer erkennen 
    # cv2.HOUGH_GRADIENT: Vector mit erkannten Raendern (Kreis)
    # parameter1: upper threshold for the INTERNAL CANNY EDGE DETECTOR # Anzahl zu erkennende Kreise kann durch den Wert erhoeht werden
    # parameter2: threshold for center detection # Je hoeher desto geringer die Fehlerkennung
    # parameter1 > parameter2 !!!   
    # min_radius = Minimum radius to be detected  ' if unknown "= 0"
    # max_radius = Maximum radius to be detected  ' if unknown "= 0"
    # https://docs.opencv.org/3.1.0/dd/d1a/group__imgproc__feature.html#ga47849c3be0d0406ad3ca45db65a25d2d
    #
circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows/8, param1=120, param2=40, minRadius=1, maxRadius=70)
# default 100, 30,1, 30
if circles is not None:
    circles = np.uint16(np.around(circles))
    for i in circles [0, :]:  # draw the detected circles
        center = (i[0], i[1])
        # circle center draw green point
        cv2.circle(imgCircles, center, 1, (0, 100, 100), 3)
        # circle outline draw red
        radius = i[2]
        cv2.circle(imgCircles, center, radius, (255, 0, 255), 3)
    

######################### Detection SURF (unstable and no official Module) ###############
"""
imgGerade = cv2.imread('/home/pses/catkin_ws/src/echtzeitsysteme/images/DBSignCompare/geradeaus.jpg')

imgObjectReference = imgGerade  # Reference
imgDetectionInput = img         # finde Referenz in Bild zum vergleichen

# Nr.1  SURF Detection to detect the keypoints # compute the descriptors
minimalHessian = 400
detector = cv2.xfeatures2d_SURF.create(hessianThreshold=minimalHessian)
keypoints_obj, descriptors_obj = detector.detectAndCompute(imgObjectReference, None)
keypoints_scene, descriptors_scene = detector.detectAndCompute(imgDetectionInput, None)

# Nr.2  Matching descriptor vectors with a FLANN based matcher
    # Since SURF is a floating-point descriptor NORM_L2 is used
matcher = cv2.DescriptorMatcher_create(cv2.DescriptorMatcher_FLANNBASED)
knn_matches = matcher.knnMatch(descriptors_obj, descriptors_scene, 2)

# Nr.3  Filter matches using the Lowe's ratio test
ratio_thresh = 0.75
good_matches = []
for m,n in knn_matches:
    if m.distance < ratio_thresh * n.distance:
        good_matches.append(m)

# Nr.4  Draw matches
img_matches = np.empty((max(imgObjectReference.shape[0], imgDetectionInput.shape[0]), imgObjectReference.shape[1]+imgDetectionInput.shape[1], 3), dtype=np.uint8)
cv2.drawMatches(imgObjectReference, keypoints_obj, imgDetectionInput, keypoints_scene, good_matches, img_matches, flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

# Nr.5 Localize the object
obj = np.empty((len(good_matches),2), dtype=np.float32)
scene = np.empty((len(good_matches),2), dtype=np.float32)
for i in range(len(good_matches)):
    #-- Get the keypoints from the good matches
    obj[i,0] = keypoints_obj[good_matches[i].queryIdx].pt[0]
    obj[i,1] = keypoints_obj[good_matches[i].queryIdx].pt[1]
    scene[i,0] = keypoints_scene[good_matches[i].trainIdx].pt[0]
    scene[i,1] = keypoints_scene[good_matches[i].trainIdx].pt[1]
H, _ =  cv.findHomography(obj, scene, cv.RANSAC)

# Nr.6 Get the corners from the image_1 ( the object to be "detected" )
obj_corners = np.empty((4,1,2), dtype=np.float32)
obj_corners[0,0,0] = 0
obj_corners[0,0,1] = 0
obj_corners[1,0,0] = img_object.shape[1]
obj_corners[1,0,1] = 0
obj_corners[2,0,0] = img_object.shape[1]
obj_corners[2,0,1] = img_object.shape[0]
obj_corners[3,0,0] = 0
obj_corners[3,0,1] = img_object.shape[0]
scene_corners = cv.perspectiveTransform(obj_corners, H)

# Nr.7 Draw lines between the corners (the mapped object in the scene - image_2 )
cv.line(img_matches, (int(scene_corners[0,0,0] + imgObjectReference.shape[1]), int(scene_corners[0,0,1])),\
    (int(scene_corners[1,0,0] + imgObjectReference.shape[1]), int(scene_corners[1,0,1])), (0,255,0), 4)
cv.line(img_matches, (int(scene_corners[1,0,0] + imgObjectReference.shape[1]), int(scene_corners[1,0,1])),\
    (int(scene_corners[2,0,0] + imgObjectReference.shape[1]), int(scene_corners[2,0,1])), (0,255,0), 4)
cv.line(img_matches, (int(scene_corners[2,0,0] + imgObjectReference.shape[1]), int(scene_corners[2,0,1])),\
    (int(scene_corners[3,0,0] + imgObjectReference.shape[1]), int(scene_corners[3,0,1])), (0,255,0), 4)
cv.line(img_matches, (int(scene_corners[3,0,0] + imgObjectReference.shape[1]), int(scene_corners[3,0,1])),\
    (int(scene_corners[0,0,0] + imgObjectReference.shape[1]), int(scene_corners[0,0,1])), (0,255,0), 4)

# Output Show detected matches
cv.imshow('Matches form Input and Libary & Object detection', img_matches)
"""


################################# Detection AKAZE ###########################
### compare two images Input & Reference -> get Matches and then deside which sign it could be
imgRef = cv2.imread("/home/pses/catkin_ws/src/echtzeitsysteme/images/DBSignCompare/rechts.jpg", cv2.IMREAD_GRAYSCALE)
imgCompare = gray #cv2.imread("/home/pses/catkin_ws/src/echtzeitsysteme/images/signs.jpg", cv2.IMREAD_GRAYSCALE)
#imgCompare = cv2.imread("/home/pses/catkin_ws/src/echtzeitsysteme/images/ErgebnisTmp.jpg", cv2.IMREAD_GRAYSCALE)
if imgRef is None or imgCompare is None:
    print('Could not open or find the images!')
    exit(0)
fs = cv2.FileStorage("/home/pses/catkin_ws/src/echtzeitsysteme/images/DBSignCompare/Homography1to3p.xml", cv2.FILE_STORAGE_READ)
homography = fs.getFirstTopLevelNode().mat()

akaze = cv2.AKAZE_create()
kpts1, desc1 = akaze.detectAndCompute(imgRef, None)
kpts2, desc2 = akaze.detectAndCompute(imgCompare, None)
matcher = cv2.DescriptorMatcher_create(cv2.DescriptorMatcher_BRUTEFORCE_HAMMING)
nn_matches = matcher.knnMatch(desc1, desc2, 2)
matched1 = []
matched2 = []
nn_match_ratio = 0.8 # Nearest neighbor matching ratio
for m, n in nn_matches:
    if m.distance < nn_match_ratio * n.distance:
        matched1.append(kpts1[m.queryIdx])
        matched2.append(kpts2[m.trainIdx])
inliers1 = []
inliers2 = []
good_matches = []
inlier_threshold = 2.5 # Default 2.5 # Distance threshold to identify inliers with homography check
for i, m in enumerate(matched1):
    col = np.ones((3,1), dtype=np.float64)
    col[0:2,0] = m.pt
    col = np.dot(homography, col)
    col /= col[2,0]
    dist = sqrt(pow(col[0,0] - matched2[i].pt[0], 2) +\
                pow(col[1,0] - matched2[i].pt[1], 2))
    if dist < inlier_threshold:
        good_matches.append(cv.DMatch(len(inliers1), len(inliers2), 0))
        inliers1.append(matched1[i])
        inliers2.append(matched2[i])
res = np.empty((max(imgRef.shape[0], imgCompare.shape[0]), imgRef.shape[1]+imgCompare.shape[1], 3), dtype=np.uint8)
cv2.drawMatches(imgRef, inliers1, imgCompare, inliers2, good_matches, res)
cv2.imwrite("akaze_result.png", res)
inlier_ratio = len(inliers1) / float(len(matched1))
print('A-KAZE Matching Results')
print('*******************************')
print('# Keypoints Reference:                        \t', len(kpts1))
print('# Keypoints Compare:                        \t', len(kpts2))
print('# Matches:                            \t', len(matched1))
print('# Inliers:                            \t', len(inliers1))
print('# Inliers in Ratio:                      \t', inlier_ratio)
cv2.imshow('AKAZE result', res)







################### Ausgabe ##############
# Show Image 
cv2.imshow("Input", img)
cv2.imshow("detected circles", imgCircles)


cv2.namedWindow('Contrast/Brightness', cv2.WINDOW_AUTOSIZE)
cv2.imshow("Contrast/Brightness", contrastBrightnessOutput)


cv2.imshow("GammaOutput", gammaImgOutput)


cv2.namedWindow('Kanten/Canny Detection')
#incl Trackbar -> Regler ausgabe CannyThreshold
title_trackbar = 'Min Schwellwert:'
window_name = 'Kanten/Canny Detection'
max_lowThreshold = 100
cv2.createTrackbar(title_trackbar, window_name , 0, max_lowThreshold, CannyThreshold)
CannyThreshold(0)   # Default Value


cv2.waitKey(0)