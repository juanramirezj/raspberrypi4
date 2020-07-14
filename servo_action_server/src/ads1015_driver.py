import board
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

class ADS1015:
    def __init__(self):
        # Create the I2C bus
        self.i2c = busio.I2C(board.SCL, board.SDA)

        # Create the ADC object using the I2C bus
        self.ads = ADS.ADS1015(i2c)

        # Create single-ended input on channel 0
        self.chan0 = AnalogIn(ads, ADS.P0)
        self.chan1 = AnalogIn(ads, ADS.P1)

    def print_values(self):
        print("{:>5},{:>5.3f},{:>5},{:>5.3f}".format(self.chan0.value, self.chan0.voltage, self.chan1.value, self.chan1.voltage))
    def get_values(self):
        return {
                'value0': self.chan0.value,
                'voltage0': self.chan0.voltage,
                'value1': self.chan1.value,
                'voltage1': self.chan1.voltage
                }


