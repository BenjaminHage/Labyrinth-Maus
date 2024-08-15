# Labyrinth-Maus

sudo raspi-config nonint do_i2c 0
i2cdetect -y 1

sudo apt install git python3-dev python3-pip
sudo pip3 install smbus2 // Brake_system_package irgendwas

git clone https://github.com/pololu/motoron-python.git
cd motoron-python
sudo python3 setup.py install

git clone https://github.com/abelectronicsuk/ABElectronics_Python_Libraries.git
cd ABElectronics_Python_Libraries
python3 -m build
sudo python3 -m installer dist/*.whl 
