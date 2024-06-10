from classes.hx711 import hx711
from scipy.signal import savgol_filter, iirnotch, filtfilt, butter
import time


class Scale:
    """
    The Scale object uses an hx711 chip to measure the weigth on a
    scale built with four load cells
    """

    def __init__(self, referenceUnit=22659):
        """
        :param referenceUnit: This unit is used to divide the measured value
                              from the hx711 and convert it to kg
        """
        self.hx = hx711(5, 6)
        self.hx.set_reading_format("MSB", "MSB")
        self.hx.set_reference_unit(referenceUnit)
        self.hx.reset()

        self.time = []
        self.measurements = []
        self.stop = False
        


    def tare(self):
        """
        tare measures the value from the scale and makes it the new 0 kg
        """
        self.hx.tare(times=100) #measures 100 times and uses the average
        print('Tare done!')


    def get_weight(self, average_over=1):
        """
        get_weigth returns the measured weigth from the scale
        :param average_over: (optional) indicates the number of measurements to
                             average over before returning a value
        :return weigth: measured weigth in kg
        """
        weight = self.hx.get_weight(average_over)
        return weight

    def continuous_measurement(self):
        t0 = time.monotonic()
        self.time = []
        self.measurements = []
        while not self.stop:
            temp = self.get_weight()
            t1 = time.monotonic() - t0
            self.time.append(t1)
            self.measurements.append(temp)
            time.sleep(1/150)
        
        print('done measuring with scale')
            


    def close_scale(self):
        """
        close_scale closes the connection with the hx711 to avoid issues with GPIO
        """
        self.hx.close_connection()


    def clean_values(self, timestamp, values):
        """
        clean_values applies various rounds of filters to the complete list of 
        measurements to return a cleaner result
        :param timestamp: list of times for every recorded weigth
        :param values: list of recorded values in kg
        :return timestamp_1: list of times for every filtered weigth value
        :return values_clean_4: list of filtered values in kg
        """
        thresh = 4 # Max allowed kg between a value and its adjacent measurements

        values_clean_1 = []
        timestamp_1 = []

        for i in range(1, len(values)-1): # cleaning the lists of any erronous mesurement
            if not((abs(values[i] - values[i-1]) > thresh) and (abs(values[i] - values[i+1]) > thresh)):
                values_clean_1.append(values[i])
                timestamp_1.append(timestamp[i])

        # first round of notch filtering at 50 hz
        fs = 115.0
        Q = 15
        f0 = 50.0
        b, a = iirnotch(f0, Q, fs)
        values_clean_2 = filtfilt(b, a, values_clean_1)

        # second round of notch filtering at 48 hz
        fs = 115.0
        Q = 15
        f0 = 48
        b, a = iirnotch(f0, Q, fs)
        values_clean_3 = filtfilt(b, a, values_clean_2)

        # third round of notch filtering at 52 hz
        fs = 115.0
        Q = 15
        f0 = 52
        b, a = iirnotch(f0, Q, fs)
        values_clean_4 = filtfilt(b, a, values_clean_3)

        return timestamp_1, values_clean_4
