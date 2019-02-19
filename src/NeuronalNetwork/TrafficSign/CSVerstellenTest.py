import cv2
print (cv2.__version__)
import csv

import numpy
import imageio                  # pip3 install imageio
import matplotlib.pyplot
import glob




###############################################   Bild verarbeiten und in .csv schreiben ####################################
#
## sign Cat  ->  Gibt die Art des Schildes an 0=STOP 1=RECHTS 2=LINKS 3=GERADE
def verarbeiteBild(filename, signCat):

    ############# Einlesen  ###################
    # Image aus File lesen
    img = cv2.imread(filename)

    # grayscale
        # Eingehendes Bild in den Grauen Farbraum transferieren
        # erhoeht die Geschwindigkeit der Erkennung
        # kein Unterschied ob das Schild grau oder bunt ist
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #BGR 3 Values per Pixel (Blue, Green, Red)

    # original Bild -> in 28*28 Pixel
    grayResize = cv2.resize(gray, (28,28))

    # Konvertiert 2D -> 1D 
    array1D = grayResize.reshape(-1) 

    # Arrays zusammenfuegen
    marriedArry = numpy.concatenate(([signCat],array1D))

    # Array -> Liste
    results = list(map(int, marriedArry))

    return results




# schreibe CSV Datei
with open('/home/pses/catkin_ws/src/echtzeitsysteme/images/testSchilder/schilder_test.csv', 'w', newline='') as csvfile:
    myPictureWriter = csv.writer(csvfile, delimiter=',')
    # einzelnes Bild reinschreiben
    # myPictureWriter.writerow(verarbeiteBild('/home/pses/catkin_ws/src/echtzeitsysteme/images/TrafficOnWay/RoadRechtsGerade3.jpg', 3))

    # mehrere Bilder reinschreiben
    # Art des Schildes an 
        # 0=30        1=Vorfahrtsstrasse        2=STOP              3=EinfahrtVerboten        4=RECHTS      
        # 5=LINKS     6=GERADE                  7=GeradeRechts      8=GeradeLinks             9=Kreisel
    
    """
    for image_name in glob.glob('/home/pses/catkin_ws/src/echtzeitsysteme/images/testSchilder/30/*.jpg'):  # "?" als wildcard bzw. "/*.ppm"
        #print(image_name)     # Debug
        myPictureWriter.writerow(verarbeiteBild(image_name, 0))

    for image_name in glob.glob('/home/pses/catkin_ws/src/echtzeitsysteme/images/testSchilder/Vorfahrtstrasse/*.jpg'):   
        #print(image_name)     # Debug
        myPictureWriter.writerow(verarbeiteBild(image_name, 1))
    
    for image_name in glob.glob('/home/pses/catkin_ws/src/echtzeitsysteme/images/testSchilder/STOP/*.jpg'):   
        #print(image_name)     # Debug
        myPictureWriter.writerow(verarbeiteBild(image_name, 2))
    """

    for image_name in glob.glob('/home/pses/catkin_ws/src/echtzeitsysteme/images/testSchilder/EinfahrtVerboten/*.jpg'):   
        #print(image_name)     # Debug
        myPictureWriter.writerow(verarbeiteBild(image_name, 3))

    for image_name in glob.glob('/home/pses/catkin_ws/src/echtzeitsysteme/images/testSchilder/RECHTS/*.jpg'):   
        #print(image_name)     # Debug
        myPictureWriter.writerow(verarbeiteBild(image_name, 4))
    """
    for image_name in glob.glob('/home/pses/catkin_ws/src/echtzeitsysteme/images/testSchilder/LINKS/*.jpg'):  
        #print(image_name)     # Debug
        myPictureWriter.writerow(verarbeiteBild(image_name, 5))
    """
    for image_name in glob.glob('/home/pses/catkin_ws/src/echtzeitsysteme/images/testSchilder/GERADE/*.jpg'):  
        #print(image_name)     # Debug
        myPictureWriter.writerow(verarbeiteBild(image_name, 6))

    for image_name in glob.glob('/home/pses/catkin_ws/src/echtzeitsysteme/images/testSchilder/GeradeRechts/*.jpg'):  
        #print(image_name)     # Debug
        myPictureWriter.writerow(verarbeiteBild(image_name, 7))
    """
    for image_name in glob.glob('/home/pses/catkin_ws/src/echtzeitsysteme/images/testSchilder/GeradeLinks/*.jpg'): 
        #print(image_name)     # Debug
        myPictureWriter.writerow(verarbeiteBild(image_name, 8))

    for image_name in glob.glob('/home/pses/catkin_ws/src/echtzeitsysteme/images/testSchilder/Kreisel/*.jpg'):  
        #print(image_name)     # Debug
        myPictureWriter.writerow(verarbeiteBild(image_name, 9))
    """
print('fertig')