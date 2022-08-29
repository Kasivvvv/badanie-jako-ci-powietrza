import bme280
import bmp180
import HTU21D
import mq7



bme280_i2c = I2C(1,scl=Pin(7), sda=Pin(6), freq=10000)
bme = BME280(i2c=bme280_i2c)

#bmp180_i2c =  I2C(1, scl=Pin(7), sda=Pin(6), freq=100000)
#bmp180 = BMP180(bmp180_i2c)

lectura = HTU21D(7,6)

mq7 = MQ7(26,3.3)

mq135 = MQ135(26)

while True:

    temp = bme.temperature
    hum = bme.humidity
    pres = bme.pressure
    print('bme280')
    print('Temperature: ', temp)
    print('Humidity: ', hum)
    print('Pressure: ', pres)
  
    #temp = bmp180.temperature
    #p = bmp180.pressure
    #altitude = bmp180.altitude
    #print('bmp180')
    #print('Temperature: ', temp)
    #print('Altitude: ', altitude)
    #print('Pressure: ', p)

    hum = lectura.humidity
    temp = lectura.temperature
    print('HTU21')
    print('Humedad: ', + hum)
    print('Temperatura: ', + temp)

    
    print('MQ-7')
    mq7.calibrate()
    print('PPM = {} '.format(mq7.readPpm()))
    
    print('MQ-135')
    ppm = mq135.get_ppm()
    corrected_ppm = mq135.get_corrected_ppm(temperature, humidity)
    print('PPM:'+str(ppm)+'ppm')
    print("Corrected PPM: "+str(corrected_ppm)+"ppm")
    


    time.sleep(5)






  
  