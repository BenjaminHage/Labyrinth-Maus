import RPi.GPIO as GPIO
import time

# GPIO-Pins für die Encoder-Signale
pin_a_left = 17
pin_b_left = 27
pin_a_right = 21
pin_b_right = 20

# Zähler für die Pulse
counter_left = 0
counter_right = 0

def update_count_left(channel):
    global counter_left
    b_state = GPIO.input(pin_b_left)
    if b_state:
        counter_left += 1
    else:
        counter_left -= 1
    print(f"Left Encoder Pulse Count: {counter_left}")

def update_count_right(channel):
    global counter_right
    b_state = GPIO.input(pin_b_right)
    if b_state:
        counter_right += 1
    else:
        counter_right -= 1
    print(f"Right Encoder Pulse Count: {counter_right}")

# GPIO-Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(pin_a_left, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(pin_b_left, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(pin_a_right, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(pin_b_right, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Interrupts auf den A-Pins
GPIO.add_event_detect(pin_a_left, GPIO.RISING, callback=update_count_left)
GPIO.add_event_detect(pin_a_right, GPIO.RISING, callback=update_count_right)

try:
    print("Drehe die Räder, um die Pulse zu zählen...")
    while True:
        time.sleep(1)  # Hauptschleife läuft weiter, um die Interrupts zu ermöglichen
except KeyboardInterrupt:
    print("Beende das Programm.")
finally:
    GPIO.cleanup()
