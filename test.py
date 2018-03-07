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

rr_pid = PID(p=0.7, i=1, imax=50)
pr_pid = PID(p=0.7, i=1, imax=50)
yr_pid = PID(p=2.7, i=1, imax=50)
rs_pid = PID(p=4.5)
ps_pid = PID(p=4.5)
ys_pid = PID(p=10)



poll_interval = imu.IMUGetPollInterval()
print("Recommended Poll Interval: %dmS\n" % poll_interval)

pygame.init()
j = pygame.joystick.Joystick(0)
j.init()
print 'Initialized Joystick : %s' % j.get_name()
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

SERVO_0 = 0
SERVO_1 = 1
SERVO_2 = 2
SERVO_3 = 3

servo.setTarget(SERVO_0, 4000)
servo.setTarget(SERVO_1, 4000)
servo.setTarget(SERVO_2, 4000)
servo.setTarget(SERVO_3, 4000)

time.sleep(1)

servo.setTarget(SERVO_0, 8000)
servo.setTarget(SERVO_1, 8000)
servo.setTarget(SERVO_2, 8000)
servo.setTarget(SERVO_3, 8000)

time.sleep(0.1)

servo.setTarget(SERVO_1, 4000)
servo.setTarget(SERVO_0, 4000)
servo.setTarget(SERVO_2, 4000)
servo.setTarget(SERVO_3, 4000)

def map(x):
    result = (x - 1)*(8000 - (-8000)) / (-1 - (1)) + (-8000)
    if result < 4000: return 4000
    if result > 8000: return 8000
    else: return result

def rc_map(x,in_min,in_max,out_min,out_max):
    result = (x - in_min)*(out_max - out_min) / (in_max - in_min) + out_max
    return result



def get():
    #Read input from the two joysticks
    pygame.event.pump()
    throttle = map(j.get_axis(2))
    roll = j.get_axis(3)
    pitch = j.get_axis(4)
    yaw = j.get_axis(5)
    rc_angles = collections.namedtuple('rc_angles', ['throttle', 'roll', 'pitch', 'yaw'])
    #print("Pitch: {}".format(pitch))
    #print("Roll: {}".format(roll))
    #print("Throttle: {}".format(throttle))
    #Read input from button
    return rc_angles

imu.setSlerpPower(0.02)
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)

def test():
    first = time.time()
    while True:
        if imu.IMURead():
          program_starts = time.time()
          x, y, z = imu.getFusionData()
          print("%f %f %f" % (x,y,z))
          data = imu.getIMUData()
          fusionPose = data["fusionPose"]
          des_angle = get()
          des_pitch = rc_map(des_angle[0],-1,1,-45,45)
          des_roll = rc_map(des_angle[1],-1,1,-45,45)
          des_yaw = rc_map(des_angle[2],-1,1,-45,45)
          des_throttle = rc_map(des_angle[3],-1,1,4000,8000)
          roll = math.degrees(fusionPose[0])
          pitch = math.degrees(fusionPose[1])
          yaw = math.degrees(fusionPose[2])
          pid_pitch_stab_stab.SetPoint=des_pitch
          pid_roll_stab_stab_stab.SetPoint=des_roll
          feedback_pitch = pitch
          feedback_roll = roll
          pid_pitch_stab_stab.update(feedback_pitch)
          if des_throttle > 1200:
              pitch_stab_out = max(min(pid_pitch_stab_stab.output, 250),  -250)
              roll_stab_out = max(min(pid_roll_stab_stab_stab.output, 250),  -250)
              if des_yaw > 5:
                  yaw_stab_ouput = des_yaw
                  yaw_target = yaw
              pid_gyro_pitch.Setpoint = pitch_stab_out
              pid_gyro_roll.SetPoint = roll_stab_out
              feedback_gyro_pitch = gyro_pitch
              feedback_gyro_roll = gyro_roll
              pid_gyro_pitch.update(feedback_gyro_pitch)
              pid_gyro_roll.update(feedback_gyro_roll)
              pitch_out = max(min(pid_gyro_pitch.output, 500), -500)
              roll_out = max(min(pid_gyro_roll.output,500), -500)
          print("r: %f p: %f y: %f" % (roll, pitch 
          , yaw ))
          #print("It has been {0} seconds since the loop started".format(now - program_starts))
          time.sleep(poll_interval*1.0/1000.0)
          servo.setTarget(SERVO_3, get())
          servo.setTarget(SERVO_0, get())
          servo.setTarget(SERVO_2, get())
          servo.setTarget(SERVO_1, get())
          #print("r: %f p: %f y: %f" % (math.degrees(fusionPose[0]), math.degrees(fusionPose[1]), math.degrees(fusionPose[2])))
            #print("Throttle: {}".format(get()))
    
        
test()
