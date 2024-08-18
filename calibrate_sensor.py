import numpy as np
from scipy.optimize import curve_fit
from ADCDifferentialPi import ADCDifferentialPi
import matplotlib.pyplot as plt

class DistanceSensor:
    def __init__(self):
        self.adc = ADCDifferentialPi(0x68, 0x69, 14)
        self.data = []

    def measure_voltage(self):
        voltage = self.adc.read_voltage(1)  # Annahme: Kanal 1 wird verwendet
        return voltage

    def collect_data(self):
        while True:
            distance = input("Gib die Distanz des Objekts in cm ein (oder 'fertig' zum Beenden): ")
            if distance.lower() == 'fertig':
                break
            try:
                distance = float(distance)
            except ValueError:
                print("Bitte gib eine gültige Zahl ein.")
                continue

            repetitions = int(input("Wie oft soll die Messung wiederholt werden? "))
            for _ in range(repetitions):
                voltage = self.measure_voltage()
                self.data.append((distance, voltage))
            print(f"{repetitions} Messungen bei {distance} cm durchgeführt.")

    def fit_function(self, voltage, a, b, c, d, e):
        return (a * voltage**2 + b * voltage + c) / (d * voltage + e)

    def determine_parameters(self):
        distances, voltages = zip(*self.data)
        popt, _ = curve_fit(self.fit_function, voltages, distances)
        return popt

    def save_parameters(self, filename, parameters):
        with open(filename, 'w') as file:
            for param in parameters:
                file.write(f"{param}\n")
        print(f"Parameter in {filename} gespeichert.")

    def convert_voltage_to_distance(self, voltage, parameters):
        a, b, c, d, e = parameters
        return (a * voltage**2 + b * voltage + c) / (d * voltage + e)

    def visualize_data(self, parameters):
        distances, voltages = zip(*self.data)
        fitted_distances = [self.convert_voltage_to_distance(v, parameters) for v in voltages]

        plt.scatter(voltages, distances, label='Gemessene Daten')
        plt.plot(voltages, fitted_distances, color='red', label='Gefittete Kurve')
        plt.xlabel('Spannung (V)')
        plt.ylabel('Distanz (cm)')
        plt.legend()
        plt.title('Spannung vs. Distanz')
        plt.show()

# Beispielverwendung
sensor = DistanceSensor()

# Daten sammeln
sensor.collect_data()

# Parameter bestimmen
parameters = sensor.determine_parameters()

# Parameter speichern
sensor.save_parameters('parameters.txt', parameters)

# Visualisierung der Daten und der gefitteten Kurve
sensor.visualize_data(parameters)
