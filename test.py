import maestro
import time
import pygame
import time
import sys, getopt
sys.path.append('.')
import RTIMU
import os.path
import time
import math
import collections
import PID

SETTINGS_FILE = "RTIMULib"

print("Using settings file " + SETTINGS_FILE + ".ini")
if not os.path.exists(SETTINGS_FILE + ".ini"):
  print("Settings file does not exist, will be created")

s = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(s)

print("IMU Name: " + imu.IMUName())

if (not imu.IMUInit()):
    print("IMU Init Failed")
    sys.exit(1)
else:
    print("IMU Init Succeeded")


servo = maestro.Controller()
#servo.setRange(0, 4000, 8000)    #set servo 0 acceleration to 4
#servo.setSpeed(0, 0)


pid_pitch_stab = PID.PID(0.1, 0, 0)
pid_roll_stab = PID.PID(0.1, 0, 0)
pid_yaw_stab = PID.PID(10, 0, 0)


pid_pitch_rate = PID.PID(0.1, 0, 0)
pid_roll_rate = PID.PID(0.1, 0, 0)
pid_yaw_rate = PID.PID(1, 0, 0)


poll_interval = imu.IMUGetPollInterval()
print("Recommended Poll Interval: %dmS\n" % poll_interval)

pygame.init()
j = pygame.joystick.Joystick(0)
j.init()
#print 'Initialized Joystick : %s' % j.get_name()
"""
Returns a vector of the following form:
[LThumbstickX, LThumbstickY, Unknown Coupled Axis???, 
RThumbstickX, RThumbstickY, 
Button 1/X, Button 2/A, Button 3/B, Button 4/Y, 
Left Bumper, Right Bumper, Left Trigger, Right Triller,
Select, Start, Left Thumb Press, Right Thumb Press]

Note:
No D-Pad.
Triggers are switches, not variable. 
Your controller may be different
"""
MOTOR_FL = 2
MOTOR_FR = 0
MOTOR_BL = 1
MOTOR_BR = 3

servo.setTarget(MOTOR_BR, 4000)
servo.setTarget(MOTOR_FR, 4000)
servo.setTarget(MOTOR_FL, 4000)
servo.setTarget(MOTOR_BL, 4000)

time.sleep(1)

servo.setTarget(MOTOR_FR, 8000)
servo.setTarget(MOTOR_FL, 8000)
servo.setTarget(MOTOR_BR, 8000)
servo.setTarget(MOTOR_BL, 8000)

time.sleep(0.1)

servo.setTarget(MOTOR_FR, 4000)
servo.setTarget(MOTOR_FL, 4000)
servo.setTarget(MOTOR_BR, 4000)
servo.setTarget(MOTOR_BL, 4000)

def map(x):
    result = (x - 1)*(8000 - (-8000)) / (-1 - (1)) + (-8000)
    if result < 4000: return 4000
    if result > 8000: return 8000
    else: return result

def rc_map(x,in_min,in_max,out_min,out_max):
    result = (x - in_min)*(out_max - out_min) / (in_max - in_min) + out_min
    return result



def get():
    #Read input from the two joysticks
    pygame.event.pump()
    throttle = j.get_axis(2)
    roll = j.get_axis(4)
    pitch = j.get_axis(3)
    yaw = j.get_axis(5)
    rc_angles = [throttle, pitch, roll, yaw]
    #print("Pitch: {}".format(yaw))
    #print("Roll: {}".format(roll))
    #print("Throttle: {}".format(throttle))
    #print(rc_angles)
    return rc_angles

imu.setSlerpPower(0.02)
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)

def test():
    first = time.time()
    while True:
        if imu.IMURead():
          yaw_target = 0
          program_starts = time.time()
          x, y, z = imu.getFusionData()
          #print("%f %f %f" % (x,y,z))namedtuple
          data = imu.getIMUData()
          #print(math.degrees(data["gyro"][0]))
          fusionPose = data["fusionPose"]
          gyro = data["gyro"]
          #print("r: %f p: %f y: %f" % (math.degrees(fusionPose[0]), math.degrees(fusionPose[1]), math.degrees(fusionPose[2])))
          des_attitude = get()
          des_pitch = rc_map(des_attitude[1],1,-1,-45,45)
          des_roll = rc_map(des_attitude[2],-1,1,-45,45)
          des_yaw = rc_map(des_attitude[3],-1,1,-45,45)
	  #print(des_pitch)
          des_throttle = map(des_attitude[0])
          roll = math.degrees(fusionPose[0])
          pitch = math.degrees(fusionPose[1])
          yaw = math.degrees(fusionPose[2])
          pid_pitch_stab.SetPoint = des_pitch
          pid_roll_stab.SetPoint = des_roll
          pid_yaw_stab.Setpoint = 0
          feedback_stab_pitch = pitch
          feedback_stab_roll = roll
	  feedback_stab_yaw = yaw
          pid_pitch_stab.update(feedback_stab_pitch)
          pid_roll_stab.update(feedback_stab_roll)
          pid_yaw_stab.update(feedback_stab_yaw)
          if des_throttle > 5000:
              pitch_stab_out = max(min(pid_pitch_stab.output, 250),  -250)
              roll_stab_out = max(min(pid_roll_stab.output, 250),  -250)
              yaw_stab_out = max(min(pid_yaw_stab.output, 360),  -360)
              if des_yaw > 5:
                  yaw_stab_ouput = des_yaw
                  yaw_target = yaw
              pid_pitch_rate.Setpoint = pitch_stab_out
              pid_roll_rate.SetPoint = roll_stab_out
              feedback_rate_pitch = gyro[0] # I need to find these vales !
              feedback_rate_roll = gyro[1] # I need to find these values !
              feedback_rate_yaw = gyro[2]
              pid_pitch_rate.update(feedback_rate_pitch)
              pid_roll_rate.update(feedback_rate_roll)
              pid_yaw_rate.update(feedback_rate_yaw)
              pitch_out = max(min(pid_pitch_rate.output, -500), 500)
              roll_out = max(min(pid_roll_rate.output, -500), 500)
              yaw_out =  max(min(pid_yaw_rate.output, -500), 500)
              #print("r: %f p: %f y: %f" % (roll, pitch, yaw ))
              #print("It has been {0} seconds since the loop started".format(now - program_starts))
              print(int(roll_out))
              servo.setTarget(MOTOR_FL, int(des_throttle - roll_out - pitch_out - yaw_out))
              servo.setTarget(MOTOR_BL, int(des_throttle - roll_out + pitch_out + yaw_out))
              servo.setTarget(MOTOR_FR, int(des_throttle + roll_out - pitch_out + yaw_out))
              servo.setTarget(MOTOR_BR, int(des_throttle + roll_out + pitch_out - yaw_out))
              time.sleep(poll_interval*1.0/1000.0)
          else:
              #print(int(roll_out))
              servo.setTarget(MOTOR_FL, 4000)
              servo.setTarget(MOTOR_BL, 4000)
              servo.setTarget(MOTOR_FR, 4000)
              servo.setTarget(MOTOR_BR, 4000)
              yaw_target = yaw
              pid_pitch_rate.setKi(0)
              pid_roll_rate.setKi(0)
              pid_yaw_rate.setKi(0)
              time.sleep(poll_interval*1.0/1000.0)
          
          #print("Throttle: {}".format(get()))
        
test()
