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
with open('/home/pses/catkin_ws/src/echtzeitsysteme/images/refSchilder/schilder_ref.csv', 'w', newline='') as csvfile:
    myPictureWriter = csv.writer(csvfile, delimiter=',')
    # einzelnes Bild reinschreiben
    # myPictureWriter.writerow(verarbeiteBild('/home/pses/catkin_ws/src/echtzeitsysteme/images/TrafficOnWay/RoadRechtsGerade3.jpg', 3))

    # mehrere Bilder reinschreiben
    # Art des Schildes an 
        # 0=30        1=Vorfahrtsstrasse        2=STOP              3=EinfahrtVerboten        4=RECHTS      
        # 5=LINKS     6=GERADE                  7=GeradeRechts      8=GeradeLinks             9=Kreisel

    #for image_name in glob.glob('/home/pses/catkin_ws/src/echtzeitsysteme/images/TrafficOnWay/RoadRechtsGerade?.jpg'):  # "?" als wildcard bzw. "/*.ppm"
    for image_name in glob.glob('/home/pses/catkin_ws/src/echtzeitsysteme/images/refSchilder/00001/*.ppm'):  # "?" als wildcard 
        #print(image_name)     # Debug
        myPictureWriter.writerow(verarbeiteBild(image_name, 0))

    for image_name in glob.glob('/home/pses/catkin_ws/src/echtzeitsysteme/images/refSchilder/00012/*.ppm'):   
        #print(image_name)     # Debug
        myPictureWriter.writerow(verarbeiteBild(image_name, 1))

    for image_name in glob.glob('/home/pses/catkin_ws/src/echtzeitsysteme/images/refSchilder/00014/*.ppm'):   
        #print(image_name)     # Debug
        myPictureWriter.writerow(verarbeiteBild(image_name, 2))

    for image_name in glob.glob('/home/pses/catkin_ws/src/echtzeitsysteme/images/refSchilder/00017/*.ppm'):   
        #print(image_name)     # Debug
        myPictureWriter.writerow(verarbeiteBild(image_name, 3))

    for image_name in glob.glob('/home/pses/catkin_ws/src/echtzeitsysteme/images/refSchilder/00033/*.ppm'):   
        #print(image_name)     # Debug
        myPictureWriter.writerow(verarbeiteBild(image_name, 4))

    for image_name in glob.glob('/home/pses/catkin_ws/src/echtzeitsysteme/images/refSchilder/00034/*.ppm'):  
        #print(image_name)     # Debug
        myPictureWriter.writerow(verarbeiteBild(image_name, 5))

    for image_name in glob.glob('/home/pses/catkin_ws/src/echtzeitsysteme/images/refSchilder/00035/*.ppm'):  
        #print(image_name)     # Debug
        myPictureWriter.writerow(verarbeiteBild(image_name, 6))

    for image_name in glob.glob('/home/pses/catkin_ws/src/echtzeitsysteme/images/refSchilder/00036/*.ppm'):  
        #print(image_name)     # Debug
        myPictureWriter.writerow(verarbeiteBild(image_name, 7))

    for image_name in glob.glob('/home/pses/catkin_ws/src/echtzeitsysteme/images/refSchilder/00037/*.ppm'): 
        #print(image_name)     # Debug
        myPictureWriter.writerow(verarbeiteBild(image_name, 8))

    for image_name in glob.glob('/home/pses/catkin_ws/src/echtzeitsysteme/images/refSchilder/00040/*.ppm'):  
        #print(image_name)     # Debug
        myPictureWriter.writerow(verarbeiteBild(image_name, 9))



#############################################################################################################################


# Konstanten Realwold
#TrafficOnWayBilder
#filename = '/home/pses/catkin_ws/src/echtzeitsysteme/images/TrafficOnWay/RoadRechts2.jpg'                    # 1-2
##filename = '/home/pses/catkin_ws/src/echtzeitsysteme/images/TrafficOnWay/RoadRechtsGerade3.jpg'              # 1-3
#filename = '/home/pses/catkin_ws/src/echtzeitsysteme/images/TrafficOnWay/RoadEinfahrtVerboten3.jpg'         # 1-3



########## auf weissen Hintergrund Referenz
##imgRef = cv2.imread("/home/pses/catkin_ws/src/echtzeitsysteme/images/DBSignCompare/rechts.jpg", cv2.IMREAD_GRAYSCALE)                  # rechts
#imgRef = cv2.imread("/home/pses/catkin_ws/src/echtzeitsysteme/images/DBSignCompare/geradeausrechts.jpg", cv2.IMREAD_GRAYSCALE)         # gerade aus rechts #576 ref keypoints Match 88
#imgRef = cv2.imread("/home/pses/catkin_ws/src/echtzeitsysteme/images/DBSignCompare/einfahrtverboten.jpg", cv2.IMREAD_GRAYSCALE)         # einfahrt verboten

########## Bildaufnahme Referenz
#imgRef = cv2.imread("/home/pses/catkin_ws/src/echtzeitsysteme/images/DBSignCompare/einfahrtverbotenRoadref1.jpg", cv2.IMREAD_GRAYSCALE) # 1-2
#imgRef = cv2.imread("/home/pses/catkin_ws/src/echtzeitsysteme/images/DBSignCompare/geradeausrechtsRoadref1.jpg", cv2.IMREAD_GRAYSCALE)  # 1-4       #1 926 keypoints Match 110
#imgRef = cv2.imread("/home/pses/catkin_ws/src/echtzeitsysteme/images/DBSignCompare/rechtsRoadref1.jpg", cv2.IMREAD_GRAYSCALE)           # 1-3



############# Einlesen  ###################
# Image aus File lesen
#img = cv2.imread(filename)

# grayscale
    # Eingehendes Bild in den Grauen Farbraum transferieren
    # erhoeht die Geschwindigkeit der Erkennung
    # kein Unterschied ob das Schild grau oder bunt ist
#gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #BGR 3 Values per Pixel (Blue, Green, Red)


#print(gray)

#anzahlBilder = 5

##pictureCategory = [3] # Gibt die Art des Schildes an 0=STOP 1=RECHTS 2=LINKS 3=GERADE

#img_array_input = imageio.imread('/home/pses/catkin_ws/src/echtzeitsysteme/images/TrafficOnWay/RoadRechtsGerade3.jpg', as_gray=True)

#print(img_array_input)
#matplotlib.pyplot.imshow(img_array_input, cmap='Greys', interpolation = 'None')


#img_array = imageio.imread('/home/pses/catkin_ws/src/echtzeitsysteme/images/TrafficOnWay/RoadRechtsGerade3.jpg', as_gray=True)
    
# reshape from 28x28 to list of 784 values, invert values
#img_data  = 255.0 - img_array.reshape(784)
    
# then scale data to range from 0.01 to 1.0
#img_data = (img_data / 255.0 * 0.99) + 0.01



#cv2.imshow("Ref", imgRef)
#cv2.imshow("Output", gray)
##cv2.waitKey(0)

print('fertig :)')