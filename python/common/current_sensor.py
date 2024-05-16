#!/usr/bin/python3
from ina219 import INA219
from ina219 import DeviceRangeError
import time

class CurrentSensor:
    
    SHUNT_OHMS = 0.1
    MAX_EXPECTED_AMPS = 3.2  
    
    def __init__(self):
        self.ina = INA219(self.SHUNT_OHMS, busnum=5, address=0x45)
        self.ina.configure(INA219.RANGE_16V, bus_adc=INA219.ADC_12BIT)

        self.refresh()
    
    def refresh(self):
        self.voltage = self.ina.voltage()
        try:
            self.power = self.ina.power()
            self.current = self.ina.current()
            self.shunt_voltage = self.ina.shunt_voltage()
            self.range_error = False
        except DeviceRangeError as e:
            self.range_error = True        

if __name__ == "__main__":
    
    sensor = CurrentSensor()

    running = True
    try:
        while running:
            sensor.refresh()

            print("Bus Voltage: %.3f V" % sensor.voltage)

            if(not sensor.range_error):
                print("Bus Current: %.3f mA" % sensor.current)
                print("Power: %.3f mW" % sensor.power)
                print("Shunt voltage: %.3f mV" % sensor.shunt_voltage)
            else:
                print("Current overflow")

            time.sleep(1)
    except KeyboardInterrupt:
        running = False
