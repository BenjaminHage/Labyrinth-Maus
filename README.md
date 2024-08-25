def relative_angle(robot_x, robot_y, robot_orientation, target_x, target_y):
    """
    Berechnet den relativen Winkel zwischen der aktuellen Orientierung des Roboters und einem Zielpunkt.
    
    :param robot_x: Float - Die x-Position des Roboters.
    :param robot_y: Float - Die y-Position des Roboters.
    :param robot_orientation: Float - Die aktuelle Orientierung des Roboters in Grad (0 = rechts, 90 = oben).
    :param target_x: Float - Die x-Position des Zielpunkts.
    :param target_y: Float - Die y-Position des Zielpunkts.
    
    :return: Float - Der relative Winkel zum Zielpunkt in Grad (positiv = links, negativ = rechts).
    """
    
    # Berechne den absoluten Winkel vom Roboter zum Zielpunkt
    delta_x = target_x - robot_x
    delta_y = target_y - robot_y
    absolute_angle = math.degrees(math.atan2(delta_y, delta_x))
    
    # Berechne den relativen Winkel (absolute_angle - robot_orientation)
    relative_angle = absolute_angle - robot_orientation
    
    # Normalisiere den Winkel auf den Bereich [-180, 180] Grad
    relative_angle = (relative_angle + 180) % 360 - 180
    
    return relative_angle


# Labyrinth-Maus

sudo pigpiod
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

sudo git clone https://github.com/abelectronicsuk/ABElectronics_Python_Libraries.git
cd ABElectronics_Python_Libraries
sudo python3 setup.py install #--break-system-packages

#sudo pip3 install Adafruit-Blinka --break-system-packages #vielleicht unnötig weil nächster es mit installiert 
sudo pip3 install adafruit-circuitpython-icm20x --break-system-packages

sudo pip3 install matplotlib --break-system-packages
sudo pip3 install keyboard --break-system-packages
sudo pip3 install scipy --break-system-packages
sudo pip3 install filterpy --break-system-packages

#wget https://files.waveshare.com/upload/e/ea/UPS_Module_3S_Code.zip


sudo chmod +x setup_pi_ap.sh

sudo chmod +x git-push-script.sh




###################### .env  #########################################

sudo raspi-config nonint do_i2c 0
sudo pigpiod

python3 -m venv .env
source .env/bin/activate

#pip3 install Adafruit-Blinka
#pip3 install adafruit-circuitpython-icm20x

#sudo git clone https://github.com/abelectronicsuk/ABElectronics_Python_Libraries.git
cd ABElectronics_Python_Libraries
python3 setup.py install
cd ../

#git clone https://github.com/pololu/motoron-python.git
cd motoron-python
python3 setup.py install
cd ../

pip3 install numpy
pip3 install matplotlib
pip3 install keyboard
pip3 install smbus2








