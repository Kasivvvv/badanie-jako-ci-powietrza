import am2320
from machine import I2C, Pin
i2c = I2C(scl=Pin(5), sda=Pin(4))
sensor = am2320.AM2320(i2c)
sensor.measure()
print(sensor.temperature())
print(sensor.humidity())