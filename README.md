# Echtzeitsysteme

add this directory into your catkin_ws/src folder of the PSES Repository (https://github.com/tud-pses/pses_workspace.git)
do "catkin_make" in the catkin_ws directory to invoke build process with the rest of the repository

## Bauen von einzelnen Packages

* z.B. Package Echtzeitsysteme: führe `catkin_make --pkg echtzeitsysteme` aus

## Debugging

* Bauen mit Debug-Informationen: `catkin_make -DCMAKE_BUILD_TYPE=Debug [package-name]`
* Node mit GDB debuggen:
    * z.B. `rosrun --prefix 'gdb --args' echtzeitsysteme camera_reading_test`

# Dynamic Reconfiguration von Parametern 

## Starten des Fensters zum Ändern von Parametern

Starten der Node und Öffnen des Fensters: `rosrun rqt_reconfigure rqt_reconfigure`

## Erstellen von Konfigurationen für Nodes

# camera_reading_test

## Starten der Test-Node

* von "catkin_ws/src" aus: `rosrun echtzeitsysteme camera_reading_test` ausführen
* alle Bilder sind im Ordner "images" im Echtzeitsysteme-Repo