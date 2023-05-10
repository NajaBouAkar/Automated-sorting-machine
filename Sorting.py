#Connections
#MPU6050 - Raspberry pi
#VCC - 5V  (2 or 4 Board)
#GND - GND (6 - Board)
#SCL - SCL (5 - Board)
#SDA - SDA (3 - Board)
#Servo Motor 1:
#PWM - GPIO (18 - Board)
#Servo Motor 2:
#PWM - GPIO (16 - Board)
#Servo Motor 3:
#PWM - GPIO (13 - Board)
#Servo Motor 4:
#PWM - GPIO (12 - Board)
#H-Bridge:
#PWM - GPIO (15 - Board)
#In1 - GPIO (29 - Board)
#In2 - GPIO (31 - Board)

#This code depends on the library KalmanAngle that have the following URL:https://github.com/rocheparadox/Kalman-Filter-Python-for-mpu6050.git



from Kalman import KalmanAngle
import smbus                  #import SMBus module of I2C
import time
import math
import RPi.GPIO as GPIO
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import RPi.GPIO as GPIO
from time import sleep
import socket

# Set up the pins
PWM_PIN = 15
IN1_PIN = 29
IN2_PIN = 31

GPIO.setmode(GPIO.BOARD)
GPIO.setup(PWM_PIN, GPIO.OUT)
GPIO.setup(IN1_PIN, GPIO.OUT)
GPIO.setup(IN2_PIN, GPIO.OUT)

pwm = GPIO.PWM(PWM_PIN, 100)
pwm.start(0)
GPIO.setmode(GPIO.BOARD)
# GPIO for servo
GPIO.setup(13, GPIO.OUT)
GPIO.setup(16, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(18, GPIO.OUT)

servo_pwm1 = GPIO.PWM(18, 50)
servo_pwm1.start(0)
servo_pwm2 = GPIO.PWM(16, 50) # 50Hz frequency
servo_pwm2.start(0) # start PWM with 0% duty cycle
servo_pwm3 = GPIO.PWM(13, 50) # 50Hz frequency
servo_pwm3.start(0) # start PWM with 0% duty cycle
servo_pwm4 = GPIO.PWM(12, 50)
servo_pwm4.start(0)

kalmanX = KalmanAngle()
kalmanY = KalmanAngle()
kalmanX2 = KalmanAngle()
kalmanY2 = KalmanAngle()
bus = smbus.SMBus(1)
RestrictPitch = True    #Comment out to restrict roll to ±90deg instead
radToDeg = 57.2957786
kalAngleX = 0
kalAngleY = 0
kalAngleX2 = 0
kalAngleY2 = 0
#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

def set_angle1(angle):
    duty_cycle = (angle / 18.0) + 2.5 # convert angle to duty cycle
    servo_pwm1.ChangeDutyCycle(duty_cycle)
    time.sleep(.05) # wait for servo to reach the desired angle
def set_angle2(angle):
    duty_cycle = (angle / 18.0) + 2.5 # convert angle to duty cycle
    servo_pwm2.ChangeDutyCycle(duty_cycle)
    time.sleep(.05) # wait for servo to reach the desired angle

def set_angle3(angle):
    duty_cycle = (angle / 18.0) + 2.5 # convert angle to duty cycle
    servo_pwm3.ChangeDutyCycle(duty_cycle)
    time.sleep(.05) # wait for servo to reach the desired angle

def set_angle4(angle):
    duty_cycle = (angle / 18.0) + 2.5 # convert angle to duty cycle
    servo_pwm4.ChangeDutyCycle(duty_cycle)
    time.sleep(.05) # wait for servo to reach the desired angle

#Read the gyro and acceleromater values from MPU6050


def MPU_Init(Address):
      #write to sample rate register
      bus.write_byte_data(Address, SMPLRT_DIV, 7)

      #Write to power management register
      bus.write_byte_data(Address, PWR_MGMT_1, 1)

      #Write to Configuration register
      #Setting DLPF (last three bit of 0X1A to 6 i.e '110' It removes the noise due to vibration.) https://ulrichbuschbaum.wordpress.com/2015/01/18/using-the-mpu6050s-dlpf/
      bus.write_byte_data(Address, CONFIG, int('0000110',2))

      #Write to Gyro configuration register
      bus.write_byte_data(Address, GYRO_CONFIG, 24)

      #Write to interrupt enable register
      bus.write_byte_data(Address, INT_ENABLE, 1)


def read_raw_data(Address,addr):
      #Accelero and Gyro value are 16-bit
      high = bus.read_byte_data(Address, addr)
      low = bus.read_byte_data(Address, addr+1)

      #concatenate higher and lower value
      value = ((high << 8) | low)

      #to get signed value from mpu6050
      if(value > 32768):
            value = value - 65536
      return value

# Set the motor direction
def set_direction(direction):
    if direction == 'f':
        GPIO.output(IN1_PIN, GPIO.HIGH)
        GPIO.output(IN2_PIN, GPIO.LOW)
    elif direction == 'b':
        GPIO.output(IN1_PIN, GPIO.LOW)
        GPIO.output(IN2_PIN, GPIO.HIGH)
    else:
        # Stop the motor if the direction is not recognized
        GPIO.output(IN1_PIN, GPIO.LOW)
        GPIO.output(IN2_PIN, GPIO.LOW)

# Run the motor at a given speed
def run_motor(speed):
    pwm.ChangeDutyCycle(speed * 100)
   
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

server_socket.bind(('0.0.0.0', 5555))

server_socket.listen()

client_socket, client_address = server_socket.accept()


bus = smbus.SMBus(1) #for older version boards
DeviceAddress = 0x68   # MPU6050 device address

DeviceAddress2 = 0x69

MPU_Init(DeviceAddress)
MPU_Init(DeviceAddress2)
while True:
      set_angle2(180) #rotate servo to 90 degrees
      set_angle3(40)
      set_angle1(90)
      set_angle4(180)
      time.sleep(1)
      while True:
            data = client_socket.recv(1024).decode().strip()
            print(data)
            if data == 'stop_code':
                  
                  break
            elif data=='bottle':
                  Object=True
            elif data=='cup':
                  Object=False
            elif data == 'stop':
                  print(data)

                  set_direction('s')

            else:
                  print(data)
                  
                  speed = float(data)
                  set_direction('b')
                  run_motor(speed)
                  
      angle_diff = ctrl.Antecedent(np.arange(-90, 90, 0.1), 'angle_diff')
      # Define membership functions for input variable

      angle_diff['same'] = fuzz.gaussmf(angle_diff.universe, 0, 7.5)

      angle_diff['nmedium'] = fuzz.gaussmf(angle_diff.universe, -25, 7.5)

      angle_diff['nhigh'] = fuzz.zmf(angle_diff.universe, -60, -25)

      angle_diff['pmedium'] = fuzz.gaussmf(angle_diff.universe, 25, 7.5)

      angle_diff['phigh'] = fuzz.smf(angle_diff.universe, 25, 60)

      

      # Define output variables

      output = ctrl.Consequent(np.arange(-10, 10, 0.1), 'output')

      

      # Define membership functions for output variables

      output['nc'] = fuzz.gaussmf(output.universe, 0, 1)

      output['mi'] = fuzz.gaussmf(output.universe, 4.5, 1.5)

      output['hi'] = fuzz.smf(output.universe, 2.5, 10)

      output['md'] = fuzz.gaussmf(output.universe, -4.5, 1.5)

      output['hd'] = fuzz.zmf(output.universe, -10, -2.5)

      # Define rules

      rule1 = ctrl.Rule(angle_diff['same'], output['nc'])

      rule2 = ctrl.Rule(angle_diff['nmedium'], output['md'])

      rule3 = ctrl.Rule(angle_diff['nhigh'], output['hd'])

      rule4 = ctrl.Rule(angle_diff['pmedium'], output['mi'])

      rule5 = ctrl.Rule(angle_diff['phigh'], output['hi'])

      # Define fuzzy system and add rules
      fuzzy_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5])

      # Define simulation
      simulation = ctrl.ControlSystemSimulation(fuzzy_ctrl)

     

      time.sleep(1)
      #Read Accelerometer raw value
      accX = read_raw_data(DeviceAddress,ACCEL_XOUT_H)
      accY = read_raw_data(DeviceAddress,ACCEL_YOUT_H)
      accZ = read_raw_data(DeviceAddress,ACCEL_ZOUT_H)
      accX2 = read_raw_data(DeviceAddress2,ACCEL_XOUT_H)
      accY2 = read_raw_data(DeviceAddress2,ACCEL_YOUT_H)
      accZ2 = read_raw_data(DeviceAddress2,ACCEL_ZOUT_H)

      if (RestrictPitch):
            roll = math.atan2(accY,accZ) * radToDeg
            pitch = math.atan(-accX/math.sqrt((accY**2)+(accZ**2))) * radToDeg
            roll2 = math.atan2(accY2,accZ2) * radToDeg
            pitch2 = math.atan(-accX2/math.sqrt((accY2**2)+(accZ2**2))) * radToDeg
      else:
            roll = math.atan(accY/math.sqrt((accX**2)+(accZ**2))) * radToDeg
            pitch = math.atan2(-accX,accZ) * radToDeg
            roll2 = math.atan(accY2/math.sqrt((accX2**2)+(accZ2**2))) * radToDeg
            pitch2 = math.atan2(-accX2,accZ2) * radToDeg

      kalmanX.setAngle(roll)
      kalmanY.setAngle(pitch)
      kalmanX2.setAngle(roll2)
      kalmanY2.setAngle(pitch2)
      gyroXAngle = roll;
      gyroYAngle = pitch;
      gyroXAngle2 = roll2;
      gyroYAngle2 = pitch2;
      compAngleX = roll;
      compAngleY = pitch;
      compAngleX2 = roll2;
      compAngleY2 = pitch2;
      timer = time.time()
      flag = 0
      T2=180
      T3=40
      start=time.time()
      Tref2=126
      Tref3=144
      diff_angle2=100
      diff_angle3=100

      while abs(diff_angle2)>=5 or abs(diff_angle3)>=10:
            set_angle2(T2)
            set_angle3(T3)
            
            if(flag >100): #Problem with the connection
                  print("There is a problem with the connection")
                  flag=0
                  continue
            try:
                  #Read Accelerometer raw value
                  accX = read_raw_data(DeviceAddress,ACCEL_XOUT_H)
                  accY = read_raw_data(DeviceAddress,ACCEL_YOUT_H)
                  accZ = read_raw_data(DeviceAddress,ACCEL_ZOUT_H)
                  accX2 = read_raw_data(DeviceAddress2,ACCEL_XOUT_H)
                  accY2 = read_raw_data(DeviceAddress2,ACCEL_YOUT_H)
                  accZ2 = read_raw_data(DeviceAddress2,ACCEL_ZOUT_H)
                  #Read Gyroscope raw value
                  gyroX = read_raw_data(DeviceAddress,GYRO_XOUT_H)
                  gyroY = read_raw_data(DeviceAddress,GYRO_YOUT_H)
                  gyroZ = read_raw_data(DeviceAddress,GYRO_ZOUT_H)
                  gyroX2 = read_raw_data(DeviceAddress2,GYRO_XOUT_H)
                  gyroY2 = read_raw_data(DeviceAddress2,GYRO_YOUT_H)
                  gyroZ2 = read_raw_data(DeviceAddress2,GYRO_ZOUT_H)
                  dt = time.time() - timer
                  timer = time.time()


                  if (RestrictPitch):
                        roll = math.atan2(accY,accZ) * radToDeg
                        pitch = math.atan(-accX/math.sqrt((accY**2)+(accZ**2))) * radToDeg
                        roll2 = math.atan2(accY2,accZ2) * radToDeg
                        pitch2 = math.atan(-accX2/math.sqrt((accY2**2)+(accZ2**2))) * radToDeg            
                  else:
                        roll = math.atan(accY/math.sqrt((accX**2)+(accZ**2))) * radToDeg
                        pitch = math.atan2(-accX,accZ) * radToDeg
                        roll2 = math.atan(accY2/math.sqrt((accX2**2)+(accZ2**2))) * radToDeg
                        pitch2 = math.atan2(-accX2,accZ2) * radToDeg
                  gyroXRate = gyroX/131
                  gyroYRate = gyroY/131
                  gyroXRate2 = gyroX2/131
                  gyroYRate2 = gyroY2/131        

                  if (RestrictPitch):
                        set_angle2(T2)
                        set_angle3(T3)
                        if((roll < -90 and kalAngleX >90) or (roll > 90 and kalAngleX < -90)):
                              kalmanX.setAngle(roll)
                              complAngleX = roll
                              kalAngleX   = roll
                              gyroXAngle  = roll
                        else:
                              kalAngleX = kalmanX.getAngle(roll,gyroXRate,dt)

                        if(abs(kalAngleX)>90):
                              gyroYRate  = -gyroYRate
                              kalAngleY  = kalmanY.getAngle(pitch,gyroYRate,dt)
                        if((roll2 < -90 and kalAngleX2 >90) or (roll2 > 90 and kalAngleX2 < -90)):
                              kalmanX2.setAngle(roll2)
                              complAngleX2 = roll2
                              kalAngleX2   = roll2
                              gyroXAngle2  = roll2
                        else:
                              kalAngleX2 = kalmanX2.getAngle(roll2,gyroXRate2,dt)

                        if(abs(kalAngleX2)>90):
                              gyroYRate2  = -gyroYRate2
                              kalAngleY2  = kalmanY2.getAngle(pitch2,gyroYRate2,dt)                
                  else:

                        if((pitch < -90 and kalAngleY >90) or (pitch > 90 and kalAngleY < -90)):
                              kalmanY.setAngle(pitch)
                              complAngleY = pitch
                              kalAngleY   = pitch
                              gyroYAngle  = pitch
                        else:
                              kalAngleY = kalmanY.getAngle(pitch,gyroYRate,dt)

                        if(abs(kalAngleY)>90):
                              gyroXRate  = -gyroXRate
                              kalAngleX = kalmanX.getAngle(roll,gyroXRate,dt)

                        if((pitch2 < -90 and kalAngleY2 >90) or (pitch2 > 90 and kalAngleY2 < -90)):
                              kalmanY2.setAngle(pitch2)
                              complAngleY2 = pitch2
                              kalAngleY2   = pitch2
                              gyroYAngle2  = pitch2
                        else:
                              kalAngleY2 = kalmanY2.getAngle(pitch2,gyroYRate2,dt)

                        if(abs(kalAngleY2)>90):
                              gyroXRate2  = -gyroXRate2
                              kalAngleX2 = kalmanX2.getAngle(roll2,gyroXRate2,dt)                

                      #angle = (rate of change of angle) * change in time
                  gyroXAngle = gyroXRate * dt
                  gyroYAngle = gyroYAngle * dt
                  gyroXAngle2 = gyroXRate2 * dt
                  gyroYAngle2 = gyroYAngle2 * dt
                  
                      #compAngle = constant * (old_compAngle + angle_obtained_from_gyro) + constant * angle_obtained from accelerometer
                  compAngleX = 0.93 * (compAngleX + gyroXRate * dt) + 0.07 * roll
                  compAngleY = 0.93 * (compAngleY + gyroYRate * dt) + 0.07 * pitch
                  compAngleX2 = 0.93 * (compAngleX2 + gyroXRate2 * dt) + 0.07 * roll2
                  compAngleY2 = 0.93 * (compAngleY2 + gyroYRate2 * dt) + 0.07 * pitch2
                  
                  if ((gyroXAngle < -180) or (gyroXAngle > 180)):
                        gyroXAngle = kalAngleX
                  if ((gyroYAngle < -180) or (gyroYAngle > 180)):
                        gyroYAngle = kalAngleY
                  if ((gyroXAngle2 < -180) or (gyroXAngle2 > 180)):
                        gyroXAngle2 = kalAngleX2
                  if ((gyroYAngle2 < -180) or (gyroYAngle2 > 180)):
                        gyroYAngle2 = kalAngleY2
                  
                  print("Angle X: " + str(kalAngleX)+"   " +"Angle Y: " + str(kalAngleY))
                  print("Angle X2: " + str(kalAngleX2)+"   " +"Angle Y2: " + str(kalAngleY2))
                  
                  
                  diff_angle2=Tref2-kalAngleX
                  diff_angle3=Tref3-kalAngleX2

                  if abs(diff_angle2)>=5:
                        simulation.input['angle_diff'] = diff_angle2

          # Evaluate system
                        simulation.compute()

          # Get output value
                        output_value = simulation.output['output']
                        T2-=output_value
                  else:
                        if abs(diff_angle3)>=10:
                              simulation.input['angle_diff'] = diff_angle2
                              simulation.compute()
                              output_value2 = simulation.output['output']
                              T3-=output_value2
                              
                        
                  time.sleep(0.05)


            except Exception as exc:
                  flag += 1
   
   
      set_angle4(0)
      time.sleep(2)
      T3=int(T3)
      if(T3>0):       
          for i in range (T3,0,-5):
                set_angle3(i)
                time.sleep(0.05)
      if(Object):
          for i in range(90,135,5):
                set_angle1(i)
                time.sleep(0.05)
                
      else:
          for i in range(90,45,-5):
                set_angle1(i)
                time.sleep(0.05)
      T2=int(T2)
      if(T2>110):     
          for i in range (T2,110,-5):
                set_angle2(i)
                time.sleep(0.05)
      set_angle4(180)
      time.sleep(1)
      set_angle2(180)
      time.sleep(1)
      set_angle3(40)
      time.sleep(1)
      set_angle1(90)
      time.sleep(1)
      client_socket.sendall(b'done\n')
      time.sleep(1)