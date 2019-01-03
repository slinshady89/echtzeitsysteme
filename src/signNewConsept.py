#!/usr/bin/python

#import
from __future__ import print_function
from scipy.stats import itemfreq
import cv2
print (cv2.__version__)
import numpy as np
import argparse



# Konstanten
filename = '/home/pses/catkin_ws/src/echtzeitsysteme/images/signs.jpg'
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
    cv2.imshow(window_name, dst)




# hough circle Transform

# medianBlur() -> Rauschunterdrueckung des Streams um falsche Kreiserkennung zu verhindern 
    # https://docs.opencv.org/3.1.0/d4/d13/tutorial_py_filtering.html
imgBlurCircles = cv2.medianBlur(gray, 5)
rows = imgBlurCircles.shape[0]
# HoughCircles(InputArray Bild, OutputArray cv2.HOUGH_GRADIENT, int methode, double minDist, ) 
    # Hough Transformation um Kreise in einem grayscale Bild zu finden
    # minDist: minimaler euklidischer Abstand zwischen Mittelpunkt des erkannten Kreises und dessen Rand/Kanten
    #           -> nur ein Schild immer erkennen 
    # cv2.HOUGH_GRADIENT: Vector mit erkannten Raendern (Kreis)
    # parameter1: Anzahl zu erkennende Kreise kann durch den Wert erhoeht werden
    # parameter2: Je hoeher desto geringer die Fehlerkennung
    # parameter1 > parameter2 !!!
    # https://docs.opencv.org/3.1.0/dd/d1a/group__imgproc__feature.html#ga47849c3be0d0406ad3ca45db65a25d2d
    #
circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows/8, param1=120, param2=40, minRadius=1, maxRadius=30)
if circles is not None:
    circles = np.uint16(np.around(circles))
    for i in circles [0, :]:
        center = (i[0], i[1])
        # circle center
        cv2.circle(img, center, 1, (0, 100, 100), 3)
        # circle outline
        radius = i[2]
        cv2.circle(img, center, radius, (255, 0, 255), 3)
    
    




################### Ausgabe ##############
# Show Image 
#cv2.imshow("Input", img)
cv2.imshow("detected circles", img)


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