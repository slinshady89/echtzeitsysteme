#!/usr/bin/python
import cv2, os, datetime
import rospy
import sys
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# constants

vLineCount = 6
hLineCount = 10

lineThickness = 1

gridLineColor = (0,0,255)
crossHairsColor = (0,255,0)

captureFolder = './captured_frames/'


class frame_processor:

    def __init__(self):
        self.bridge = CvBridge()
        self.subscriber = rospy.Subscriber('camera/frame', Image, self.onReceiveFrame)
        self.width, self.height = -1,-1
        self.hDist, self.vDist = -1,-1

    def onReceiveFrame(self,imgMsg):
        try:
            frame = self.bridge.imgmsg_to_cv2(imgMsg, 'bgr8')
            height, width, channels = frame.shape

            #initialize the width and height only once
            if self.width ==-1 and width > 0:
                self.width, self.height = width, height
                self.hDist = int(self.width/hLineCount)
                self.vDist = int(self.height/vLineCount)

            # show frame with grid and save it if a key is pressed
            key = cv2.waitKey(1)
            if 0 < key < 255:   # hack to prevent continually '255' key events
                captureFrame(frame)
            drawGrid(frame, self.hDist, self.vDist, self.width, self.height)
            drawCrosshairs(frame, self.width, self.height)
            windowName = 'Width: %d, Height: %d' % (self.width, self.height)
            cv2.imshow(windowName, frame)

        except CvBridgeError as err:
            print(err)


def drawGrid(frame, hDist, vDist, width, height):
    for x in range(hLineCount+1):
        cv2.line(frame, (hDist*x, 0), (hDist*x, height-1), gridLineColor, lineThickness)
    for y in range(vLineCount+1):
        cv2.line(frame, (0, vDist*y), (width-1, vDist*y), gridLineColor, lineThickness)

def drawCrosshairs(frame, width, height):
    midX, midY = int(width/2), int(height/2)
    offset = 10
    cv2.line(frame, (midX-offset, midY), (midX+offset, midY), crossHairsColor, lineThickness)
    cv2.line(frame, (midX, midY-offset), (midX, midY+offset), crossHairsColor, lineThickness)

# saves the passed frame in the folder 'captured_frames', file name is the current date and time
def captureFrame(frame):
    if not os.path.exists(captureFolder):
        os.makedirs(captureFolder)
    now = datetime.datetime.now()
    filename = now.strftime('%m-%d_%H-%M-%S.jpg')
    print filename
    cv2.imwrite(captureFolder+filename, frame)

def main(args):
    fp = frame_processor()
    rospy.init_node('drawGridOnCamera', anonymous=True)
    print('Node started.')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down...')
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)