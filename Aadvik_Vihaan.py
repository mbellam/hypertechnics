from pybricks.hubs import PrimeHub
















from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Icon, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from MineStemLib import resetHeading
from MineStemLib import myHeading
wheel_dia = 49.5
wheel_axle_dist = 125
myPrimeHub = PrimeHub()

left_wheel_motor = Motor(Port.A)
right_wheel_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
left_attachment_motor = Motor(Port.C)
right_attachment_motor = Motor(Port.D)
left_color_sensor = ColorSensor(Port.E)
right_color_sensor = ColorSensor(Port.F)
drive_base = DriveBase(left_wheel_motor, right_wheel_motor, wheel_diameter=wheel_dia, axle_track=wheel_axle_dist)
#forgot right_wheel_motor
drive_base.use_gyro(True)
def leftAttachment(speed,angle):
    left_attachment_motor.run_angle(speed,angle,then=Stop.COAST,wait=True)

def rightAttachment(speed,angle):
    right_attachment_motor.run_angle(speed,angle,then=Stop.COAST,wait=True) 
#leftAttachment(50,  180)
#rightAttachment(150, 90)
def run1():
 #needed colon :
    leftAttachment(50, 180)
    rightAttachment(150, 90)
    leftAttachment(80, 360)
    leftAttachment(20, -90)
    rightAttachment(100, -135)

#run1()

def run2():
    drive_base.settings(straight_speed=600,straight_acceleration=300,turn_rate=400, turn_acceleration=300)
    drive_base.straight(10)
    drive_base.turn(45)
    drive_base.straight(150)
    drive_base.turn(-45)
    drive_base.straight(500)
    drive_base.turn(55)
    drive_base.straight(650)


#run2()

def run3():
     drive_base.settings(straight_speed=600,straight_acceleration=200,turn_rate=250, turn_acceleration=87.536)
     drive_base.turn(15)
     #from starting home base
     wait(200)
     drive_base.straight(-600)
     #using original turning to go to second turning spot
     wait(200)
     drive_base.turn(35)
     #turning to start pushing lever
     wait(200)
     drive_base.straight(-395)
     #pushing lever
     wait(200)
     drive_base.turn(-21)
     #former one 18 degrees negative
     #turning to push lever again
     wait(200)
     drive_base.straight(-165)
     #pushing lever all the way
     drive_base.straight(185)
     #coming back to pick up second item
     wait(200)
     drive_base.turn(18)
     #turn to get it
     wait(200)
     drive_base.straight(-310)
     #get item
     wait(200)
     drive_base.turn(-70)
     #turning to get four items
     wait(200)
     drive_base.straight(-400)
     #going to pick up two items
     wait(200)
     drive_base.turn(-15)
     #turning to pick up one item
     wait(200)
     drive_base.straight(-150)
     #picks up third item
     wait(200)
     drive_base.turn(-55)
     #picking up fourth item
     wait(200)
     drive_base.straight(-70)
     #picks it up
     
     drive_base.straight(-450)
     #going to home base
     
     drive_base.turn(60)
     drive_base.straight(-100)
     #drive_base.straight(40) 
     #drive_base.turn(45)
     #drive_base.straight(150)
     #drive_base.turn(-40)
     #drive_base.straight(500)
     #drive_base.turn(50)
     #drive_base.straight(470)
     #drive_base.turn(-30)
     #drive_base.straight(150)
     #drive_base.straight(-15)
     #drive_base.turn(59)
     #drive_base.straight(50)
     #drive_base.turn(-50)
     #drive_base.straight(410)
     #drive_base.turn(-110)
     #drive_base.straight(300)
    



run3()



