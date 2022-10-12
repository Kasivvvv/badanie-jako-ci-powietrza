import bme280
import bmp180
import HTU21D
import mq7
import mq135
import sen0321



bme280_i2c = I2C(1,scl=Pin(7), sda=Pin(6), freq=10000)
bme = BME280(i2c=bme280_i2c)

#bmp180_i2c =  I2C(1, scl=Pin(7), sda=Pin(6), freq=100000)
#bmp180 = BMP180(bmp180_i2c)

lectura = HTU21D(7,6)

mq7 = MQ7(26,3.3)

mq135 = MQ135(26)

IIC_MODE         = 0x01
OZONE_ADDRESS_3    = 0x73
MEASURE_MODE_AUTOMATIC    = 0x00

sen0321 = DFRobot_Ozone_IIC(IIC_MODE ,OZONE_ADDRESS_3)
sen0321.set_mode(MEASURE_MODE_AUTOMATIC)

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
    
    ozone_concentration = sen0321.get_ozone_data(20)*1.996
    print("Ozone: %d \u03BCg/m^3."%ozone_concentration)



    time.sleep(5)






  
  