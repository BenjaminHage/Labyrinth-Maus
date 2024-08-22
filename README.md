# Labyrinth-Maus

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
#sudo pip3 install adafruit-circuitpython-icm20x --break-system-packages

sudo pip3 install matplotlib --break-system-packages
sudo pip3 install keyboard --break-system-packages
sudo pip3 install scipy --break-system-package


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








