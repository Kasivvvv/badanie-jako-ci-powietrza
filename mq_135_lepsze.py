import time
from machine import ADC

R0 = 176;
R2 = 1000;

while 1:
    adc = ADC(26)
    
    volts = (adc.read_u16() *3.3)/1023
    Rs = R2 * (1-volts)
    Rs = Rs/volts
    PPM_acetone = 159.6 - 133.33*(Rs/R0)
    print(PPM_acetone)
    time.sleep(5)