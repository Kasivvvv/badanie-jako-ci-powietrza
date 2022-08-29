#pin 26
#repo https://github.com/fjebaker/MQ7/blob/master/examples/MQ7_Example.ino
from machine import ADC, Pin, Timer
import time

class MQ7:
    CALIBRATION_SECONDS = 15
    COEF_A0 = 100.0
    COEF_A1 = -1.513
    LOAD_RES = 10.0
    CALIBRATION_CONSTANT = 5.0
    
    
    def __init__(self, pin, v_in):
        self.pin = ADC(Pin(26))
        self.v_in = 3.3
    
    def readPpm(self):
        return self.COEF_A0*((self.readRs()/self.calibrate())**self.COEF_A1)
    
    def readRs(self):
        return self.LOAD_RES * self.readRsRl()
    
    def readRsRl(self):
        voltage = self.convertVoltage(ADC.read_u16(self.pin))
        return (self.v_in - voltage)/voltage
    
    def convertVoltage(self,voltage):
        return voltage * (self.v_in/1023.0)
    
    def calibrate(self):
        for i in range(self.CALIBRATION_SECONDS + 1):
            time.sleep(1)
            R0 = self.readRs()/self.CALIBRATION_CONSTANT
        return R0     

mq7 = MQ7(26,3.3)
timer_work = Timer()
timer_calibartion = Timer()

def do_calibartion(timer_calibartion):
    print('Calibarting')
    mq7.calibrate()
    print('Calibartion done')


def do_work(timer_work):
    print('PPM = {} '.format(mq7.readPpm()))


timer_work.init(freq=1, mode=Timer.PERIODIC, callback=do_work)
timer_calibartion.init(freq=0.2, mode=Timer.PERIODIC, callback=do_calibartion)



    