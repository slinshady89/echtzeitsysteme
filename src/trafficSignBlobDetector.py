# coding: utf-8

# imports
import cv2
import numpy as np

# Read Image
#image_path =  '/home/pses/catkin_ws/src/echtzeitsysteme/images/signs.ppm'      # ’test_image.ppm’
#img = cv2.imread('/home/pses/catkin_ws/src/echtzeitsysteme/images/signs.ppm')
#img = cv2.imread('/home/pses/catkin_ws/src/echtzeitsysteme/images/WebcamInput.JPG')

#Funktioniert bisher nur blob
img = cv2.imread('/home/pses/catkin_ws/src/echtzeitsysteme/images/blob.jpg')

# convert BGR image to HSV
hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)





# Setup SimpleBlobDetector parameters.
params = cv2.SimpleBlobDetector_Params()

# Filter by Circularity
params.filterByCircularity = True
params.minCircularity = 0.75

# Set up the detector with the parameters - Different CV Versions
ver = (cv2.__version__).split('.')
if int (ver[0])<3:
    detector = cv2.SimpleBlobDetector()#params)
else :
    detector = cv2.SimpleBlobDetector_create()#params)






## Detect blobs.
keypoints = detector.detect(img)#hsv[:,:,1])

# Draw detected blabs with a red circle around                       (B,G,R)
img_with_keypoints = cv2.drawKeypoints(img, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS makes the size of the cirlee to the size of the detected blob



# Show detected Blobs with keypoints 
cv2.imshow("Blobs with Keypoints", img_with_keypoints)
cv2.waitKey(0)