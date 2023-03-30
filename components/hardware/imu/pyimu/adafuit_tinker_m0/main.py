import board
import busio
import time
import adafruit_bno055
import math
     
#Initialize I2C and the sensor.
i2c = busio.I2C(board.SCL, board.SDA)
#sensor = adafruit_bno055.BNO055(i2c)
sensor = adafruit_bno055.BNO055(i2c)
#sensor.set_mode(OPERATION_MODE_NDOF)

# Main loop runs forever printing acceleration and Euler angles every second.

while True:
    Temperature = sensor.temperature
    XAcc, YAcc, ZAcc = sensor.linear_acceleration
    XMag, YMag, ZMag = sensor.magnetometer
    XGyr, YGyr, ZGyr = sensor.gyroscope
    Yaw, Roll, Pich = sensor.euler
	
    #print('Quaternion: {}'.format(sensor.quaternion))
	#print('Linear acceleration (m/s^2): {}'.format(sensor.linear_acceleration))
	#print('Gravity (m/s^2): {}'.format(sensor.gravity))
    angle = (math.atan2(YMag, XMag) - 0.01989675)*180/math.pi
    print (Yaw, Roll, Pich, XAcc, YAcc, ZAcc, XGyr, YGyr, ZGyr, XMag, YMag, ZMag, "comp", angle)
    
    
    
    
#    print(Yaw, Roll, Pich)
#    print (XAcc)
#    print (YAcc)
#    print (ZAcc)
#    print (XGyr)
#    print (YGyr)
#    print (ZGyr)
#    print (XMag)
#    print (YMag)
#    print (ZMag)
#    print (Yaw)
#    print (Roll)
#    print (Pich)    
    #print (Temperature)    
    time.sleep(0.004)
