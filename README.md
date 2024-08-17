# Labyrinth-Maus

sudo apt install git python3-dev python3-pip

sudo pip3 install smbus2 --break-system-packages
sudo raspi-config nonint do_i2c 0
i2cdetect -y 1

sudo apt remove python3-rpi.gpio
sudo apt update
sudo apt install python3-rpi-lgpio

git clone https://github.com/pololu/motoron-python.git
cd motoron-python
sudo python3 setup.py install #--break-system-packages

git clone https://github.com/abelectronicsuk/ABElectronics_Python_Libraries.git
cd ABElectronics_Python_Libraries
sudo python3 setup.py install #--break-system-packages

sudo pip3 install matplotlib --break-system-packages
sudo pip3 install keyboard --break-system-packages

chmod +x setup_pi_ap.sh

chmod +x git-push-script.sh


