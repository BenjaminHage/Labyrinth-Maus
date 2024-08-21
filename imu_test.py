import time
import board
import adafruit_icm20x

i2c = board.I2C()
icm = adafruit_icm20x.ICM20948(i2c)


while True:
    print("test")
    print(icm.gyro)