# Echtzeitsysteme

add this directory into your catkin_ws/src folder of the PSES Repository (https://github.com/tud-pses/pses_workspace.git)
do "catkin_make" in the catkin_ws directory to invoke build process with the rest of the repository

## Bauen von einzelnen Packages

* z.B. Package Echtzeitsysteme: führe `catkin_make --pkg echtzeitsysteme` aus

## Debugging

* Bauen mit Debug-Informationen: `catkin_make -DCMAKE_BUILD_TYPE=Debug [package-name]`
* Node mit GDB debuggen:
    * z.B. `rosrun --prefix 'gdb --args' echtzeitsysteme camera_reading_test`

# Nodes

## lane_detection
`rosrun echtzeitsysteme lane_detection`

Liest Frames (der Webcam) vom Topic `camera/frame` aus, verarbeitet diese und published das verarbeitete Bild auf
dem Topic `image_processing/fully_processed`. Erkannte Punkte der Fahrbahn werden auf den
Topics `right_line`, `left_line` und `center_line` gepublished.

## webcam_publisher
`rosrun echtzeitsysteme webcam_publisher`

Startet die Webcam und liest kontinuierlich Bilder ein, die über das Topic `camera/frame`
gepublished werden.

## image_publisher
`rosrun echtzeitsysteme image_publisher`

Liest ein Test-Bild ein und published dieses in regelmäßigen Abständen über das Topic `camera/frame`.

(Nur zu Testzwecken.)

## image_viewer
`rosrun echtzeitsysteme image_viewer`

Liest das Topic `camera/frame` aus und zeigt jedes neue Bild in einem Fenster an.

## trajectory_planning

Liest die Punkte der erkannten Linien sowie die Distanzen der Ultraschallsensoren ein. Daraus wird eine Wunschtrajektorie
berechnet. Mit Hilfe der Trajektorie wird über das Ackermannmodell der zu stellende Lenkwinkel bzw die entsprechende
Stellgröße bestimmt.
Sendet die 'trajectory' damit sie später in das Debugimage eingetragen werden kann.

## drawGridOnCamera.py
`rosrun echtzeitsysteme drawGridOnCamera.py`
(Zuerst die Datei ausführbar machen mit `chmod +x drawGridOnCamera.py`)

Liest das Topic `camera/frame` aus und zeigt das Bild mit einem eingezeichneten Grid an.
Drücken einer beliebigen Taste nimmt ein Foto auf (wird unter `captured_frames` abgespeichert).


## Starten der Test-Node

* von "catkin_ws/src" aus: `rosrun echtzeitsysteme camera_reading_test` ausführen
* alle Bilder sind im Ordner "images" im Echtzeitsysteme-Repo

# Rekonfiguration von Parametern zur Laufzeit 

## Starten des Fensters zum Ändern von Parametern

Starten der Node und Öffnen des Fensters: `rosrun rqt_reconfigure rqt_reconfigure`

## Erstellen von Konfigurationen für Nodes
# Sonstiges

* alle Test-Bilder sind im Ordner `images`

## Kommunikation zwischen ROS-Nodes über das Netzwerk

* auf einem Gerät muss der Master laufen (roscore oder uc_bridge)
    * verwaltet die Topics und Publisher und Subscriber
* auf dem anderen Gerät (welches mit dem Master verbunden werden soll):
    * Neusetzen der Master-ID (im Terminal des zweiten Geräts): `export ROS_MASTER_URI=http://[IP des ersten Geräts]:11311`
    * Hinzufügen der IP-Adresse des Master-Gerätes in der Datei `/etc/hosts` (auf dem zweiten Gerät, i.d.R. der Laptop):
        `<IP>   pses-04` (pses-04 ist der Hostname).
    * danach können Nodes auf dem zweiten Gerät regulär mit dem Master kommunizieren
    und über Topics publishen und subscriben (müssen über das gleiche Terminal ausgeführt
    werden, in welchem die Variable gesetzt wurde)
    
## Einrichtung von CLion für ROS
<https://www.jetbrains.com/help/clion/ros-setup-tutorial.html>


#### Starten mit korrekten Umgebungsvariablen

* Terminal starten, in ``catkin_ws`` wechseln → stelle sicher, dass ROS-Environment-Variables gelesen werden (zum Ausführen von ``rosrun, catkin_make,`` ...)
* von dort aus Clion ausführen
	* entweder durch vorher definiertem alias oder durch Angabe des vollen Pfads zu Clion (bei mir ``/opt/CLion/bin/clion.sh``)


#### ROS-Projekt(e) öffnen
In Clion:

* File → Open → die Datei "CMakeLists.txt" im ``src``-Ordner des Catkin-Workspace öffnen ("Open as Project")
* *alternativ:* File → Import → wähle das ``src``-Directory aus, um von dort aus das Projekt zu importieren


* wenn alles gut geht, sollte unten im CMake-Tab von Clion der Build-Prozess angezeigt werden und nicht failen


#### einmalige Konfiguration (nach dem ersten Öffnen des Projekts)

* In Clion:
	* File → Settings → Build, Execution, Deployment → CMake
	* bei "CMake options" Folgendes eintragen (Pfad durch eigenen Pfad ersetzen):
		* ``-DCATKIN_DEVEL_PREFIX:PATH= <voller Pfad zum Ordner catkin_ws>``
	* bei "Generation path" Folgendes eintragen:
		* ``<voller Pfad zum Ordner catkin_ws>/build_clion``
		* (es kann auch ein anderer Ordner für den Build angegeben werden, aber ein separater (build_clion) hat für mich am besten funktioniert)
	* Konfiguration speichern, fertig!
		* beim Starten einer Node (der entsprechenden main) wird der eigene Ordner für den Build genutzt
