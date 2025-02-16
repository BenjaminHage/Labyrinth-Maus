# Labyrinth-Maus

#sudo pigpiod
sudo apt install git python3-dev python3-pip

sudo pip3 install smbus2 --break-system-packages
sudo raspi-config nonint do_i2c 0
i2cdetect -y 1

sudo apt remove python3-rpi.gpio
sudo apt update
sudo apt install python3-rpi-lgpio

sudo git clone https://github.com/pololu/motoron-python.git
cd motoron-python
sudo python3 setup.py install #--break-system-packages
cd ../

sudo git clone https://github.com/abelectronicsuk/ABElectronics_Python_Libraries.git
cd ABElectronics_Python_Libraries
sudo python3 setup.py install #--break-system-packages
cd ../

#sudo pip3 install Adafruit-Blinka --break-system-packages #vielleicht unnötig weil nächster es mit installiert 
sudo pip3 install adafruit-circuitpython-icm20x --break-system-packages

sudo pip3 install matplotlib --break-system-packages
sudo pip3 install keyboard --break-system-packages
sudo pip3 install scipy --break-system-packages
sudo pip3 install filterpy --break-system-packages
sudo pip3 install numpy --break-system-packages #nachträglich hinzugefügt müsste aber hoffentlich stimmen

#wget https://files.waveshare.com/upload/e/ea/UPS_Module_3S_Code.zip #stromversorgung


#sudo chmod +x setup_pi_ap.sh

#sudo chmod +x git-push-script.sh




###################### .env  #########################################

#sudo raspi-config nonint do_i2c 0
#sudo pigpiod

#python3 -m venv .env
#source .env/bin/activate

#pip3 install Adafruit-Blinka
#pip3 install adafruit-circuitpython-icm20x

#sudo git clone https://github.com/abelectronicsuk/ABElectronics_Python_Libraries.git
#cd ABElectronics_Python_Libraries
#python3 setup.py install
#cd ../

#git clone https://github.com/pololu/motoron-python.git
#cd motoron-python
#python3 setup.py install
#cd ../

#pip3 install numpy
#pip3 install matplotlib
#pip3 install keyboard
#pip3 install smbus2





Dokumentation des Zustandsautomaten und des Pledge-Algorithmus
1. Einleitung
Der Pledge-Algorithmus ist ein Verfahren zur Navigation in unbekannten, labyrinthartigen Umgebungen. Er ermöglicht es einem Roboter, Hindernisse zu umgehen und dabei seine ursprüngliche Bewegungsrichtung beizubehalten. Diese Dokumentation beschreibt die Implementierung des Algorithmus als Zustandsautomat, der in Python umgesetzt wurde. Der Zustandsautomat steuert das Verhalten des Roboters basierend auf Sensordaten und definierten Zuständen.

2. Der Pledge-Algorithmus
2.1 Funktionsweise
Der Pledge-Algorithmus basiert auf folgenden Schritten:

Bewegung in einer definierten Richtung:
Der Roboter bewegt sich in einer festgelegten Richtung, bis er auf ein Hindernis trifft.

Folgen der Wand:
Beim Auftreffen auf ein Hindernis beginnt der Roboter, der Wand an einer Seite (z. B. links oder rechts) zu folgen.

Drehungszählung:
Während des Folgens der Wand zählt der Roboter jede Drehung:

Linksdrehungen: +1

Rechtsdrehungen: -1

Verlassen der Wand:
Sobald die Summe der Drehungen wieder null ist, verlässt der Roboter die Wand und setzt seine ursprüngliche Bewegungsrichtung fort.

Abbruchbedingung:
Falls der Roboter keinen Ausgang findet, endet die Navigation.

Der Algorithmus garantiert, dass der Roboter einen Ausgang findet, sofern einer existiert.

3. Zustandsautomat
Der Zustandsautomat ist das Herzstück der Implementierung. Er besteht aus 22 Zuständen, die das Verhalten des Roboters in verschiedenen Situationen steuern. Jeder Zustand repräsentiert eine spezifische Aktion oder Regelung, und die Übergänge zwischen den Zuständen werden durch Sensordaten und interne Logik gesteuert.

3.1 Zustände und Zustandsübergänge
Zustand 0: Initialisierung
Beschreibung: Der Roboter überprüft die Sensordaten und entscheidet, ob eine Wand erkannt wurde. Dieser Zustand dient als Startpunkt für die Navigation.

Aktionen:

Setzt follow_sensor auf eine leere Liste.

Setzt pledge_count auf eine leere Liste.

Setzt die Geschwindigkeiten der linken und rechten Räder auf 0.

Setzt on_point auf False.

Übergänge:

Zustand 3: Eine Wand auf der rechten Seite wurde erkannt. Der Roboter bereitet sich darauf vor, eine 90°-Drehung nach rechts durchzuführen. (Steuernachricht: "detect wall at the right, set up turning to it")

Zustand 4: Eine Wand auf der linken Seite wurde erkannt. Der Roboter bereitet sich darauf vor, eine 90°-Drehung nach links durchzuführen. (Steuernachricht: "detect wall at the left, set up turning to it")

Zustand 11: Eine Wand vorne links wurde erkannt. Der Roboter bereitet sich darauf vor, eine 45°-Drehung nach links durchzuführen. (Steuernachricht: "detect wall at the front_left, set up turning to it")

Zustand 10: Eine Wand vorne rechts wurde erkannt. Der Roboter bereitet sich darauf vor, eine 45°-Drehung nach rechts durchzuführen. (Steuernachricht: "detect wall at the front_right, set up turning to it")

Zustand 12: Eine Wand direkt vorne wurde erkannt. Der Roboter bereitet sich darauf vor, eine 0°-Drehung (geradeaus ausrichten) durchzuführen. (Steuernachricht: "detect wall at the front, set up turning to it")

Zustand 15: Keine Wand wurde erkannt. Der Roboter bereitet sich darauf vor, eine 360°-Drehung durchzuführen, um die Umgebung zu scannen. (Steuernachricht: "no wall detected, set up 360° turn")

Zustand 1: Linker Wand folgen
Beschreibung: Der Roboter folgt einer linken Wand und hält dabei einen definierten Abstand ein.

Aktionen:

Regelt den Abstand zur linken Wand mithilfe eines PID-Reglers.

Passt die Geschwindigkeiten der Räder an, um den Abstand zur Wand zu halten.

Setzt follow_sensor auf self.left.

Übergänge:

Zustand 6: Der Roboter erkennt eine Kante (keine Wand mehr auf der linken Seite). Er beginnt, geradeaus zu fahren. (Steuernachricht: "detect edge, start driving forward")

Zustand 9: Der Roboter erkennt eine Wand vorne innerhalb des Regelabstands. Er beginnt, den Abstand zur vorderen Wand zu regeln. (Steuernachricht: "found front wall, start controlling the distance to it")

Zustand 2: Rechter Wand folgen
Beschreibung: Der Roboter folgt einer rechten Wand und hält dabei einen definierten Abstand ein.

Aktionen:

Regelt den Abstand zur rechten Wand mithilfe eines PID-Reglers.

Passt die Geschwindigkeiten der Räder an, um den Abstand zur Wand zu halten.

Setzt follow_sensor auf self.right.

Übergänge:

Zustand 6: Der Roboter erkennt eine Kante (keine Wand mehr auf der rechten Seite). Er beginnt, geradeaus zu fahren. (Steuernachricht: "detect edge, start driving forward")

Zustand 9: Der Roboter erkennt eine Wand vorne innerhalb des Regelabstands. Er beginnt, den Abstand zur vorderen Wand zu regeln. (Steuernachricht: "found front wall, start controlling the distance to it")

Zustand 3: Vorbereitung auf 90°-Drehung nach rechts
Beschreibung: Der Roboter bereitet eine 90°-Drehung nach rechts vor, um sich parallel zu einer rechten Wand auszurichten.

Aktionen:

Berechnet den Zielwinkel (angle_setpoint) basierend auf dem aktuellen Winkel (theta) und dem vorherigen Zustand.

Setzt follow_sensor auf self.left, wenn der Roboter zuvor einer vorderen Wand gefolgt ist.

Verringert den pledge_count um 1, falls dieser nicht leer ist.

Setzt die Geschwindigkeiten der Räder auf 0.

Übergang:

Zustand 5: Die Drehung wird gestartet. (Steuernachricht: "set up 90° right turn, start turning")

Zustand 4: Vorbereitung auf 90°-Drehung nach links
Beschreibung: Der Roboter bereitet eine 90°-Drehung nach links vor, um sich parallel zu einer linken Wand auszurichten.

Aktionen:

Berechnet den Zielwinkel (angle_setpoint) basierend auf dem aktuellen Winkel (theta) und dem vorherigen Zustand.

Setzt follow_sensor auf self.right, wenn der Roboter zuvor einer vorderen Wand gefolgt ist.

Erhöht den pledge_count um 1, falls dieser nicht leer ist.

Setzt die Geschwindigkeiten der Räder auf 0.

Übergang:

Zustand 5: Die Drehung wird gestartet. (Steuernachricht: "set up 90° left turn, start turning")

Zustand 5: Geregelte Drehung
Beschreibung: Der Roboter führt eine geregelte Drehung durch, um sich zu einer Wand auszurichten.

Aktionen:

Regelt die Drehung mithilfe eines PID-Reglers, um den Zielwinkel (angle_setpoint) zu erreichen.

Passt die Geschwindigkeiten der Räder an, um die Drehung zu steuern.

Übergänge:

Zustand 15: Der Roboter hat sich orthogonal zu einer vorderen Wand ausgerichtet. Er beginnt mit der ESC-Regelung (Emergency Stop Control). (Steuernachricht: "turned to front wall, start ESC")

Zustand 19: Der Roboter hat sich parallel zu einer rechten Wand ausgerichtet. Er bereitet die PID-Regelung für die rechte Wand vor. (Steuernachricht: "turned parallel to right wall, set up PID for right wall")

Zustand 18: Der Roboter hat sich parallel zu einer linken Wand ausgerichtet. Er bereitet die PID-Regelung für die linke Wand vor. (Steuernachricht: "turned parallel to left wall, set up PID for left wall")

Zustand 6: Nach einer orthogonalen Drehung ist der Abstand zur vorderen Wand groß. Der Roboter beginnt, geradeaus zu fahren. (Steuernachricht: "did orthogonal turn, start driving forward")

Zustand 9: Nach einer orthogonalen Drehung ist die vordere Wand nah. Der Roboter beginnt, den Abstand zur vorderen Wand zu regeln. (Steuernachricht: "did orthogonal turn, start controlling to near front wall")

Zustand 6: Der Roboter hat eine Drehung auf einem Zielpunkt abgeschlossen und beginnt, geradeaus zu fahren. (Steuernachricht: "did on-point turn, start driving forward")

Zustand 6: Geradeausfahren
Beschreibung: Der Roboter fährt geradeaus, ohne eine spezifische Regelung.

Aktionen:

Passt die Geschwindigkeiten der Räder an, um geradeaus zu fahren.

Setzt on_point auf False.

Wenn der Roboter zuvor einen Zielpunkt gesetzt hat (prev_state == 7), wird die Bewegung zum Zielpunkt geregelt.

Setzt follow_sensor auf self.front, wenn der pledge_count 0 ist.

Übergänge:

Zustand 7: Der Roboter erkennt das Ende einer Wand (Kante) und setzt einen Zielpunkt. (Steuernachricht: "over edge, set target point")

Zustand 8: Der Roboter nähert sich einem Zielpunkt und beginnt, die Bewegung zum Punkt zu regeln. (Steuernachricht: "near target point, start controlling to it")

Zustand 21: Der Roboter erkennt eine Wand vorne links und beginnt, den Abstand zu dieser Wand zu regeln. (Steuernachricht: "arrived at front-left wall, start controlling to it")

Zustand 22: Der Roboter erkennt eine Wand vorne rechts und beginnt, den Abstand zu dieser Wand zu regeln. (Steuernachricht: "arrived at front-right wall, start controlling to it")

Zustand 13: Der Roboter erkennt eine vordere Wand bei einem Pledge-Zähler von 0. Er überspringt die ESC-Regelung und fährt geradeaus. (Steuernachricht: "ausrichtung gerade wand forne für pledge wechsel aber esc ertmal übersprungen")

Zustand 18: Der Roboter erkennt eine linke Wand nach einer linken Außenecke und bereitet die PID-Regelung für die linke Wand vor. (Steuernachricht: "arrived at left wall, setup PID for left wall")

Zustand 19: Der Roboter erkennt eine rechte Wand nach einer rechten Außenecke und bereitet die PID-Regelung für die rechte Wand vor. (Steuernachricht: "arrived at right wall, setup PID for right wall")

Zustand 0: Der Roboter erkennt eine Wand in der Initialisierungsphase und startet den Initialisierungsprozess erneut. (Steuernachricht: "found a wall, restart init")

Zustand 9: Der Roboter erkennt eine vordere Wand innerhalb des Regelabstands und beginnt, den Abstand zu regeln. (Steuernachricht: "front wall is near, start to control the distance")

Zustand 0: Der Roboter erkennt eine unerwartete vordere Wand und startet den Initialisierungsprozess erneut. (Steuernachricht: "error unexpected front wall, restart init")

Zustand 7: Zielpunkt setzen
Beschreibung: Der Roboter setzt einen Zielpunkt, zu dem er sich bewegen soll.

Aktionen:

Berechnet den Zielpunkt (target_x, target_y) basierend auf der aktuellen Position und Ausrichtung des Roboters.

Übergang:

Zustand 6: Der Roboter beginnt, sich zum Zielpunkt zu bewegen. (Steuernachricht: "set up forward point, start driving forward to it")

Zustand 8: Regelung zum Zielpunkt
Beschreibung: Der Roboter regelt die Bewegung, um einen bestimmten Zielpunkt zu erreichen.

Aktionen:

Regelt die Bewegung mithilfe eines PID-Reglers, um den Zielpunkt zu erreichen.

Setzt on_point auf True.

Übergänge:

Zustand 4: Der Roboter hat den Zielpunkt erreicht und bereitet eine 90°-Drehung nach links vor. (Steuernachricht: "got to point, start set up left turn")

Zustand 3: Der Roboter hat den Zielpunkt erreicht und bereitet eine 90°-Drehung nach rechts vor. (Steuernachricht: "got to point, start set up right turn")

Zustand 9: Regelung zur vorderen Wand
Beschreibung: Der Roboter regelt den Abstand zu einer vorderen Wand.

Aktionen:

Passt den front_desired_distance_faktor an, falls keine Wand vorne links oder rechts erkannt wird.

Regelt den Abstand zur vorderen Wand mithilfe eines PID-Reglers.

Übergänge:

Zustand 3: Der Roboter hat den gewünschten Abstand zur vorderen Wand erreicht und bereitet eine 90°-Drehung nach rechts vor. (Steuernachricht: "got to wall distance, start set up right turn")

Zustand 4: Der Roboter hat den gewünschten Abstand zur vorderen Wand erreicht und bereitet eine 90°-Drehung nach links vor. (Steuernachricht: "got to wall distance, start set up left turn")

Zustand 11: Der Roboter erkennt eine Wand vorne links und bereitet eine 45°-Drehung nach links vor. (Steuernachricht: "got to wall distance, detect wall front left, start set up left turn")

Zustand 10: Vorbereitung auf 45°-Drehung nach rechts
Beschreibung: Der Roboter bereitet eine 45°-Drehung nach rechts vor.

Aktionen:

Setzt den Zielwinkel (angle_setpoint) auf den aktuellen Winkel (theta) minus 45°.

Setzt die Geschwindigkeiten der Räder auf 0.

Übergang:

Zustand 5: Die Drehung wird gestartet. (Steuernachricht: "set up 45° right turn, start turning")

Zustand 11: Vorbereitung auf 45°-Drehung nach links
Beschreibung: Der Roboter bereitet eine 45°-Drehung nach links vor.

Aktionen:

Setzt den Zielwinkel (angle_setpoint) auf den aktuellen Winkel (theta) plus 45°.

Setzt follow_sensor auf self.right, wenn der Roboter zuvor einer vorderen Wand gefolgt ist.

Setzt die Geschwindigkeiten der Räder auf 0.

Übergang:

Zustand 5: Die Drehung wird gestartet. (Steuernachricht: "set up 45° left turn, start turning")

Zustand 12: Vorbereitung auf 0°-Drehung
Beschreibung: Der Roboter bereitet eine 0°-Drehung (geradeaus ausrichten) vor.

Aktionen:

Setzt den Zielwinkel (angle_setpoint) auf den aktuellen Winkel (theta).

Setzt die Geschwindigkeiten der Räder auf 0.

Übergang:

Zustand 5: Die Drehung wird gestartet. (Steuernachricht: "set up 0° turn, start turning")

Zustand 13: ESC (Emergency Stop Control)
Beschreibung: Der Roboter überprüft, ob er sich orthogonal zu einer vorderen Wand befindet.

Aktionen:

Regelt die Ausrichtung mithilfe eines ESC-Algorithmus.

Setzt follow_sensor auf self.front.

Übergang:

Zustand 6: Der Roboter hat sich orthogonal ausgerichtet und beginnt, geradeaus zu fahren. (Steuernachricht: "orthogonal to front wall, start driving forward")

Zustand 14: Ungeregelt drehen
Beschreibung: Der Roboter führt eine unregulierte Drehung durch, um eine Wand zu finden.

Aktionen:

Setzt die Geschwindigkeiten der Räder, um eine Drehung durchzuführen.

Speichert den Winkel mit dem kleinsten Abstand zur Wand (min_distance_angle).

Übergänge:

Zustand 0: Der Roboter erkennt eine Wand und startet den Initialisierungsprozess erneut. (Steuernachricht: "found a wall, restart init")

Zustand 20: Der Roboter hat eine 360°-Drehung abgeschlossen und richtet sich auf den kleinsten Abstand zur Wand aus. (Steuernachricht: "did 360° turn, set angle_setpoint to smallest front distance")

Zustand 6: Der Roboter hat keine Wand gefunden und beginnt, geradeaus zu fahren. (Steuernachricht: "found no wall after 360° turn, start driving forward")

Zustand 15: Vorbereitung auf 360°-Drehung
Beschreibung: Der Roboter bereitet eine 360°-Drehung vor, um die Umgebung zu scannen.

Aktionen:

Setzt den Zielwinkel (angle_setpoint) auf den aktuellen Winkel (theta) plus 360°.

Setzt die Geschwindigkeiten der Räder auf 0.

Setzt follow_sensor auf self.front, wenn der vorherige Zustand 5 war.

Übergang:

Zustand 14: Die Drehung wird gestartet. (Steuernachricht: "set up 360° turn, start turning")

Zustand 16: Direktes Wechseln nach links
Beschreibung: Der Roboter wechselt direkt zur linken Wand.

Aktionen:

Setzt follow_sensor auf self.left.

Übergänge:

Keine spezifischen Übergänge.

Zustand 17: Direktes Wechseln nach rechts
Beschreibung: Der Roboter wechselt direkt zur rechten Wand.

Aktionen:

Setzt follow_sensor auf self.right.

Übergänge:

Keine spezifischen Übergänge.

Zustand 18: PID-Regelung für linke Wand
Beschreibung: Der Roboter richtet sich aus, um einer linken Wand zu folgen.

Aktionen:

Initialisiert den PID-Regler für die linke Wand.

Setzt pledge_count auf 0, falls dieser leer ist.

Übergänge:

Zustand 1: Der Roboter beginnt, der linken Wand zu folgen. (Steuernachricht: "did setup, start following left wall")

Zustand 6: Der Roboter erkennt eine Kante (keine Wand mehr links) und beginnt, geradeaus zu fahren. (Steuernachricht: "detect edge, start driving forward")

Zustand 19: PID-Regelung für rechte Wand
Beschreibung: Der Roboter richtet sich aus, um einer rechten Wand zu folgen.

Aktionen:

Initialisiert den PID-Regler für die rechte Wand.

Setzt pledge_count auf 0, falls dieser leer ist.

Übergänge:

Zustand 2: Der Roboter beginnt, der rechten Wand zu folgen. (Steuernachricht: "did setup, start following right wall")

Zustand 6: Der Roboter erkennt eine Kante (keine Wand mehr rechts) und beginnt, geradeaus zu fahren. (Steuernachricht: "detect edge, start driving forward")

Zustand 20: Winkel mit kleinstem Abstand einstellen
Beschreibung: Der Roboter stellt den Winkel ein, bei dem der Abstand zur Wand am kleinsten ist.

Aktionen:

Berechnet den Zielwinkel (angle_setpoint) basierend auf dem kleinsten Abstand zur Wand (min_distance_angle).

Setzt die Geschwindigkeiten der Räder auf 0.

Übergang:

Zustand 5: Der Roboter beginnt die geregelte Drehung, um sich auf den kleinsten Abstand zur Wand auszurichten. (Steuernachricht: "set smallest distance angle, start turning")

Zustand 21: Links diagonal folgen
Beschreibung: Der Roboter folgt einer Wand, die vorne links erkannt wurde.

Aktionen:

Regelt den Abstand zur vorne links erkannten Wand mithilfe eines PID-Reglers.

Passt die Geschwindigkeiten der Räder an, um den Abstand zur Wand zu halten.

Setzt follow_sensor auf self.left.

Übergänge:

Zustand 1: Der Roboter erkennt die linke Wand und beginnt, dieser zu folgen. (Steuernachricht: "arrived at left wall, start controlling to it")

Zustand 6: Der Roboter erkennt eine Kante (keine Wand mehr vorne links) und beginnt, geradeaus zu fahren. (Steuernachricht: "detect edge, start driving forward")

Zustand 22: Rechts diagonal folgen
Beschreibung: Der Roboter folgt einer Wand, die vorne rechts erkannt wurde.

Aktionen:

Regelt den Abstand zur vorne rechts erkannten Wand mithilfe eines PID-Reglers.

Passt die Geschwindigkeiten der Räder an, um den Abstand zur Wand zu halten.

Setzt follow_sensor auf self.right.

Übergänge:

Zustand 2: Der Roboter erkennt die rechte Wand und beginnt, dieser zu folgen. (Steuernachricht: "arrived at right wall, start controlling to it")

Zustand 6: Der Roboter erkennt eine Kante (keine Wand mehr vorne rechts) und beginnt, geradeaus zu fahren. (Steuernachricht: "detect edge, start driving forward")

4. Fazit
Der Zustandsautomat ermöglicht eine effiziente und flexible Implementierung des Pledge-Algorithmus. Durch die klare Definition der Zustände und Übergänge kann der Roboter komplexe Umgebungen navigieren und Hindernisse sicher umgehen. Die Dokumentation dient als Referenz für die Weiterentwicklung und Fehlerbehebung des Systems.




