#!/usr/bin/python

# package scipy installieren ueber console
# Befehl:     sudo apt-get install python-scipy

#import
import cv2
import numpy as np
from scipy.stats import itemfreq



# Hilfsfunktion um den maximalen Farbanteil/groesstes Aufkommen einer Farbe in einem Bild zu erhalten
# https://stackoverflow.com/questions/43111029/how-to-find-the-average-colour-of-an-image-in-python-with-opencv#43111221
def get_dominant_color(image, n_colors):
    pixels = np.float32(image).reshape((-1, 3))
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, .1)
    flags = cv2.KMEANS_RANDOM_CENTERS
    flags, labels, centroids = cv2.kmeans(pixels, n_colors, None, criteria, 10, flags)
    palette = np.uint8(centroids)
    return palette[np.argmax(itemfreq(labels)[:, -1])]

# Maus Interaktion fuer evtl. lernen von Sings
clicked = False
def onMouse(event, x, y, flags, param):
    global clicked
    if event == cv2.EVENT_LBUTTONUP:
        clicked = True

# default Kamera 0 ansonsten andere Zahl waehlen >0 
# Standard Quelle (/dev/videoN)
#videocapture cameraCapture = VideoCapture(0)
cameraCapture = cv2.VideoCapture(0) 

########## Aufloesung reduzieren ################################
cameraCapture.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
cameraCapture.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
cameraCapture.set(cv2.CAP_PROP_FPS, 10)
#########

#Kamera oeffnen 
cv2.namedWindow('Kamerabild')
cv2.setMouseCallback('KameraMouse', onMouse)

# Frames kontinuerlich in einer Schleife einlesen/aktualisieren
success, frame = cameraCapture.read()
while success and not clicked:
    cv2.waitKey(1)
    success, frame = cameraCapture.read()

    

    # Eingehendes Bild in den Grauen Farbraum transferieren
    # erhoeht die Geschwindigkeit der Erkennung
    # kein Unterschied ob das Schild grau oder bunt ist
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    


    # medianBlur() -> Rauschunterdrueckung des Streams um falsche Kreiserkennung zu verhindern 
    # https://docs.opencv.org/3.1.0/d4/d13/tutorial_py_filtering.html
    img = cv2.medianBlur(gray, 37)


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
    circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT,1, 50, param1=120, param2=40)
   
    # Demo zwecke darstellung mehrer Kreise
    #circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT,1, 50, param1=120, param2=40)
    

    if not circles is None:
        circles = np.uint16(np.around(circles))

        # Problem bei einer grossen Anzahl an Schildern, das jeweils aktuelle/naechste herauszufiltern  
        # -> naechsten/groesten Kreis/Schild herausfiltern, kleinere/weiter entferntere verwerfen
        max_r, max_i = 0, 0
        for i in range(len(circles[:, :, 2][0])):
            if circles[:, :, 2][0][i] > 50 and circles[:, :, 2][0][i] > max_r:
                max_i = i
                max_r = circles[:, :, 2][0][i]
        x, y, r = circles[:, :, :][0][max_i]


        # Out of Bounds verhindern
        # Zugriff auf einen Index ausserhalb der Elipse/Kreises verhindern, da fuer die weitere
        # Verarbeitung der Ausschnitt nur benoetigt wird
        if y > r and x > r:
            square = frame[y-r:y+r, x-r:x+r]

            # dominierende Farbe erhalten
            dominant_color = get_dominant_color(square, 2)
            # unterteilung der Verkehrszeichen in 2 Farbkategorien
            #   rot -> Stopschild
            #   blau -> Richtungsweisende Schilder

            # STOP Schild ist als einizges Rot, Vergleich ob hoher Rotanteil im erkannten Kreis
            if dominant_color[2] > 100:
                # Zuordnung des Verkehrszeichens zu seiner Bedeutung
                print("STOP")                       ##### // Problem alles rote wird als STOP erkannt

            # Richtungsweisende Schilder die nur Blau sein koennen
            # Vergleich ob hoher Blauanteil im erkannten Kreis
            # python elif = elseif
            elif dominant_color[0] > 80:
                # educates guess:  erkannter Kreis/Schild in 3 Zonen aufteilen

                # linke Zone
                zone_0 = square[square.shape[0]*3//8:square.shape[0] * 5//8, square.shape[1]*1//8:square.shape[1]*3//8]
                zone_0_color = get_dominant_color(zone_0, 1)

                # mittlere Zone
                zone_1 = square[square.shape[0]*1//8:square.shape[0] * 3//8, square.shape[1]*3//8:square.shape[1]*5//8]
                zone_1_color = get_dominant_color(zone_1, 1)

                # rechte Zone
                zone_2 = square[square.shape[0]*3//8:square.shape[0] * 5//8, square.shape[1]*5//8:square.shape[1]*7//8]
                zone_2_color = get_dominant_color(zone_2, 1)


                # Zuordnung der Verkehrszeichen zu ihrer Bedeutung
                if zone_1_color[2] < 60:
                    if sum(zone_0_color) > sum(zone_2_color):
                        print("Links")
                    else:
                        print("Rechts")
                else:
                    if sum(zone_1_color) > sum(zone_0_color) and sum(zone_1_color) > sum(zone_2_color):
                        print("Gerade aus")
                    # python elif = elseif
                    elif sum(zone_0_color) > sum(zone_2_color):
                        print("Gerade aus und Links")
                    else:
                        print("Gerade aus und Rechts")
            else:
                # Unbekanntes Schild Behandlung
                print("unbekannt")

        # Ausgabe der erkannten Verkehrszeichen auf dem Bildschirm mit jeweiliger Typbezeichnung
        for i in circles[0, :]:
            cv2.circle(frame, (i[0], i[1]), i[2], (0, 255, 0), 2)
            cv2.circle(frame, (i[0], i[1]), 2, (0, 0, 255), 3)
    cv2.imshow('Kamera', frame)

    # zusaetzlicher Output
    cv2.imshow('gray', gray)
    cv2.imshow('img', img)
   


# offene Fenster schliessen
cv2.destroyAllWindows()
# Kamera Stream beenden und Ausfuehrung stoppen 
cameraCapture.release()