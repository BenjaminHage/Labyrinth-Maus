import time
import board
import adafruit_icm20x
import pigpio

i2c = board.I2C()
icm = adafruit_icm20x.ICM20948(i2c)
pi = pigpio.pi()

def funk(gpio, level, tick):
    print("									interupt")
    
pi.callback(17, pigpio.RISING_EDGE, funk)
pi.callback(21, pigpio.RISING_EDGE, funk)


while True:
    print("test")
    print(icm.gyro)
    