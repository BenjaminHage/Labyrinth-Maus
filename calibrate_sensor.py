import argparse
import numpy as np
from scipy.optimize import curve_fit
from ADCDifferentialPi import ADCDifferentialPi
import matplotlib.pyplot as plt

class DistanceSensor:
    def __init__(self):
        self.adc = ADCDifferentialPi(0x68, 0x69, 18)
        self.data = []
        self.default_repetitions = 5  # Standardwert für die Anzahl der Messungen, falls nicht angegeben

    def measure_voltage(self):
        voltage = self.adc.read_voltage(5)  # Annahme: Kanal 1 wird verwendet
        return voltage

    def collect_data(self, fixed_repetitions=None):
        while True:
            distance = input("Gib die Distanz des Objekts in cm ein (oder 'fertig' zum Beenden): ")
            if distance.lower() == 'fertig':
                break
            try:
                distance = float(distance)
            except ValueError:
                print("Bitte gib eine gültige Zahl ein.")
                continue

            if fixed_repetitions is None:
                repetitions = input(f"Wie oft soll die Messung wiederholt werden? (Standard: {self.default_repetitions}) ")
                repetitions = int(repetitions) if repetitions.isdigit() else self.default_repetitions
            else:
                repetitions = fixed_repetitions

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

        # Spannungswerte für die Interpolation
        min_voltage = min(voltages)
        max_voltage = max(voltages)
        interpolated_voltages = np.linspace(min_voltage, max_voltage, 500)

        # Entsprechende interpolierte Distanzen berechnen
        fitted_distances = [self.convert_voltage_to_distance(v, parameters) for v in interpolated_voltages]

        # Originale Datenpunkte plotten
        plt.scatter(voltages, distances, label='Gemessene Daten', color='blue')

        # Interpolierte gefittete Kurve plotten
        plt.plot(interpolated_voltages, fitted_distances, color='red', label='Gefittete Kurve (interpoliert)')

        plt.xlabel('Spannung (V)')
        plt.ylabel('Distanz (cm)')
        plt.legend()
        plt.title('Spannung vs. Distanz')
        plt.grid(True)
        plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Distance Sensor Calibration Script")
    parser.add_argument('-f', '--filename', type=str, default='parameters.txt', help='Name der Datei, in die die Parameter gespeichert werden.')
    parser.add_argument('-n', '--num_measurements', type=int, nargs='?', const=5, help='Feste Anzahl von Messungen für jede Distanz (Standard: 5).')

    args = parser.parse_args()

    sensor = DistanceSensor()

    # Daten sammeln
    sensor.collect_data(fixed_repetitions=args.num_measurements)

    # Parameter bestimmen
    parameters = sensor.determine_parameters()

    # Parameter speichern
    sensor.save_parameters(args.filename, parameters)

    # Visualisierung der Daten und der gefitteten Kurve
    sensor.visualize_data(parameters)
