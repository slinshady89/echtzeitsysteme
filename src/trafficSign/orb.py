import cv2
from cv2 import __version__
print (cv2.__version__)
 
## https://rdmilligan.wordpress.com/2015/03/01/road-sign-detection-using-opencv-orb/


# constants
IMAGE_SIZE = 200.0
MATCH_THRESHOLD = 3
 
# load haar cascade and street image
roundabout_cascade = cv2.CascadeClassifier('/home/pses/catkin_ws/src/echtzeitsysteme/src/trafficSign/haarcascade_roundabout.xml')
street = cv2.imread('/home/pses/catkin_ws/src/echtzeitsysteme/src/trafficSign/roundabout1.png')
 
# do roundabout detection on street image
gray = cv2.cvtColor(street,cv2.COLOR_RGB2GRAY)
roundabouts = roundabout_cascade.detectMultiScale(
    gray, 
    scaleFactor=1.4, 
    minNeighbors=3
    )
 
# initialize ORB and BFMatcher
orb = cv2.ORB_create()
bf = cv2.BFMatcher(cv2.NORM_HAMMING,crossCheck=True)
 
# find the keypoints and descriptors for roadsign image
roadsign = cv2.imread('orb/roundabout.jpg',0)
kp_r,des_r = orb.detectAndCompute(roadsign,None)
 
# loop through all detected objects
for (x,y,w,h) in roundabouts:
 
    # obtain object from street image
    obj = gray[y:y+h,x:x+w]
    ratio = IMAGE_SIZE / obj.shape[1]
    obj = cv2.resize(obj,(int(IMAGE_SIZE),int(obj.shape[0]*ratio)))
 
    # find the keypoints and descriptors for object
    kp_o, des_o = orb.detectAndCompute(obj,None)
   if len(kp_o) == 0 or des_o == None : 
       continue
  

    # match descriptors
    matches = bf.match(des_r,des_o)
     
    # draw object on street image, if threshold met
    if(len(matches) >= MATCH_THRESHOLD):
        cv2.rectangle(street,(x,y),(x+w,y+h),(255,0,0),2)
 
# show objects on street image
cv2.imshow('street image', street)
cv2.waitKey(3000)
cv2.destroyAllWindows()