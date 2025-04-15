# Importing libraries
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Icon, Color, Direction, Port, Side, Stop
from pybricks.tools import wait, StopWatch
from umath import sqrt


# Initialize attachment motors
front_back_attachment_motor = Motor(Port.D)
left_right_attachment_motor = Motor(Port.C)


# Attachment functions  
def FB_Attachment(speed,angle,pause):
    front_back_attachment_motor.run_angle(speed,angle,then=Stop.HOLD,wait=pause)


def LR_Attachment(speed,angle,pause):
    left_right_attachment_motor.run_angle(speed,angle,then=Stop.HOLD,wait=pause)


# Initialize wheel motors
m_left = Motor(Port.A)
m_right = Motor(Port.B)


# Initialize PrimeHub
hub = PrimeHub()


# Initialize StopWatch
stop_watch = StopWatch()


# Function to setup all the gears when we start
def fit_tools():

    speed = 1000
    wait_time = 150

    m_left.run(speed)
    m_right.run(-speed)
    left_right_attachment_motor.run(speed)
    front_back_attachment_motor.run(speed)

    wait(time=wait_time)

    m_left.brake()
    m_right.brake()
    left_right_attachment_motor.brake()
    front_back_attachment_motor.brake()
    
    wait(time=wait_time)

    m_left.run(-speed)
    m_right.run(speed)
    left_right_attachment_motor.run(-speed)
    front_back_attachment_motor.run(-speed)

    wait(time=wait_time)

    m_left.brake()
    m_right.brake()
    left_right_attachment_motor.brake()
    front_back_attachment_motor.brake()


# Function to setup only the wheels when we start
def fit_tools_onlywheels():

    speed = 1000
    wait_time = 150

    m_left.run(speed)
    m_right.run(-speed)

    wait(time=wait_time)

    m_left.brake()
    m_right.brake()

    m_left.run(-speed)
    m_right.run(speed)

    wait(time=wait_time)

    m_left.brake()
    m_right.brake()


def turn_new(left_speed, right_speed, target_angle, approach_angle=15, min_speed=40):
    m_left.control.limits(1000, 500, 200)  #configures max speed, accel, torque
    m_right.control.limits(1000, 500, 200)

    # Right turn
    if hub.imu.heading() < target_angle:
        left_velocity = left_speed
        right_velocity = right_speed
        m_left.run(left_velocity)
        m_right.run(right_velocity)

        while hub.imu.heading() < (target_angle - approach_angle):
            pass
        if right_velocity == 0:
            m_left.run(min_speed*left_velocity/abs(left_velocity))
            m_right.run(0)
        elif left_velocity == 0:
            m_left.run(0)
            m_right.run(min_speed*right_velocity/abs(right_velocity))
        else:
            m_left.run(min_speed*left_velocity/abs(left_velocity))
            m_right.run(min_speed*right_velocity/abs(right_velocity))
        while hub.imu.heading() < target_angle:
            pass
    
    # Left turn
    elif hub.imu.heading() > target_angle:
        left_velocity = -left_speed
        right_velocity = -right_speed
        m_left.run(left_velocity)
        m_right.run(right_velocity)

        while hub.imu.heading() > (target_angle + approach_angle):
            pass
        if right_velocity == 0:
            m_left.run(min_speed*left_velocity/abs(left_velocity))
            m_right.run(0)
        elif left_velocity == 0:
            m_left.run(0)
            m_right.run(min_speed*right_velocity/abs(right_velocity))
        else:
            m_left.run(min_speed*left_velocity/abs(left_velocity))
            m_right.run(min_speed*right_velocity/abs(right_velocity))
        while hub.imu.heading() > target_angle:
            pass

    # Stop and hold motors at current position
    m_left.hold()
    m_right.hold()

    # Reset both wheel motor encoders
    m_left.reset_angle(0)
    m_right.reset_angle(0)


def accel_straightPID(initial_velocity, final_velocity, target, direction):
    """
    Advances forwards or backwards until it reaches the target at the given velocity
    - velocity: Velocity of the motors. this velocity will be changed by a little by the pid to correct the path and direction of the robot
    - target: Target value of the drive motors encoders mean. 
    - direction: Direction at which the gyro should be. The difference between the real value in each iteration and the direction will be the error of the PID
    """

    # Kp, Ki and Kd factors to apply. constants
    kp = 0.0
    ki = 0
    kd = 0

    integral = 0
    initial_velocity *= 10
    final_velocity *= 10

    #print(initial_velocity, final_velocity)

    # Multiply the velocity by 10 to approximatly convert from velocity percentage (-100, 100) to degrees per second.

    #last time the loop ran
    last_time = stop_watch.time()
    last_error = direction

    if initial_velocity > 0 and final_velocity < 0:
        raise Exception("Velocities must be positive or negative")

    if initial_velocity < 0 and final_velocity > 0:
        raise Exception("Velocities must be positive or negative")

    v0 = abs(initial_velocity)
    v1 = abs(final_velocity)

    x0 = (m_left.angle() + -m_right.angle()) / 2
    total_dist = abs(target - x0)

    if (initial_velocity + final_velocity) > 0: neg = 1
    else: neg = -1

    total_time = 2 * total_dist / abs(v0 + v1)
    a = (v1 - v0) / total_time

    travelled_dist = 0

    "main loop"
    # The loop will run while the mean of the motor encoders is lower than the target.
    # Due to the motors being turned 180º in the core, the encoder of the right motor has to be negative. (This may change)
    while True:
        velocity = sqrt(v0 * v0 + 2 * a * travelled_dist)
        if velocity < 5:
            velocity = 5

        #print(velocity)

        delta_time = stop_watch.time() - last_time
        last_time = stop_watch.time()

        if delta_time < 0.001: delta_time = 0.001

        wait(100)
        travelled_dist = abs((m_left.angle() + -m_right.angle()) / 2 - x0)
        if travelled_dist >= total_dist: break

        # Get the current error.
        error = hub.imu.heading() - direction

        integral += error * delta_time

        # Multiply the error times the kp factor to get the proportional correction.
        p_correction = error * kp
        i_correction = integral * ki
        d_correction = (error - last_error)/(delta_time) * kd

        last_error = error        

        k = p_correction + i_correction + d_correction

        # Gets the final motor velocities for this iteration applying the correction to the given velocity.
        # If the robot is going backwards, the correction is the opposite of going forward.
        if velocity > 0:
            v_left = (1 - k) * velocity
            v_right = (1 + k) * velocity
        else:
            v_left = (1 + k) * velocity
            v_right = (1 - k) * velocity
        
        # Print values.
        #print(f"error {error:<20} heading {hub.imu.heading():<20} p {p_correction:<10} i {i_correction:<10} d {d_correction:<10} k {k:<10} v_left {v_left:<20} v_left {v_right:<20}")
        
        # Due to the motors being turned 180º in the core, the velocity given to the motors has to be opposite to go straight. (This may change)
        m_left.run(v_left*neg)
        m_right.run(-v_right*neg)


# Straight (PID) funtion
def straightPID(velocity, target, direction):    
    
    #Advances forwards or backwards until it reaches the target at the given velocity
    #- velocity: Velocity of the motors. this velocity will be changed by a little by the pid to correct the path and direction of the robot
    #- target: Target value of the drive motors encoders mean. 
    #- direction: Direction at which the gyro should be. The difference between the real value in each iteration and the direction will be the error of the PID 

    # Kp, Ki and Kd factors to apply. constants
    kp = 0.0
    ki = 0
    kd = 0

    integral = 0

    # Multiply the velocity by 10 to approximatly convert from velocity percentage (-100, 100) to degrees per second.
    velocity *= 10

    #last time the loop ran
    last_time = stop_watch.time()
    last_error = direction

    # If the robot is going forwards, the < sign should be used, if it is going backwards, > should be used.
    if velocity > 0:
        check_done = lambda : (m_left.angle() + -m_right.angle()) / 2 < target
    else:
        check_done = lambda : (m_left.angle() + -m_right.angle()) / 2 > target

    "main loop"
    # The loop will run while the mean of the motor encoders is lower than the target.
    # Due to the motors being turned 180º in the core, the encoder of the right motor has to be negative. (This may change)
    while check_done():

        delta_time = stop_watch.time() - last_time
        last_time = stop_watch.time()

        if delta_time < 0.001: delta_time = 0.001

        wait(100)

        # Get the current error.
        error = hub.imu.heading() - direction

        integral += error * delta_time

        # Multiply the error times the kp factor to get the proportional correction.
        p_correction = error * kp
        i_correction = integral * ki
        d_correction = (error - last_error)/(delta_time) * kd

        last_error = error        

        k = p_correction + i_correction + d_correction

        # Gets the final motor velocities for this iteration applying the correction to the given velocity.
        # If the robot is going backwards, the correction is the opposite of going forward.
        if velocity > 0:
            v_left = (1 - k) * velocity
            v_right = (1 + k) * velocity
        else:
            v_left = (1 + k) * velocity
            v_right = (1 - k) * velocity
        
        # Print values.
        #print(f"error {error:<20} heading {hub.imu.heading():<20} p {p_correction:<10} i {i_correction:<10} d {d_correction:<10} k {k:<10} v_left {v_left:<20} v_left {v_right:<20}")
        
        # Due to the motors being turned 180º in the core, the velocity given to the motors has to be opposite to go straight. (This may change)
        m_left.run(v_left)
        m_right.run(-v_right)


# Turn function
def turn(left_speed, right_speed, target_angle):
    m_left.control.limits(1000, 500, 200)  #configures max speed, accel, torque
    m_right.control.limits(1000, 500, 200)

    # Right turn
    if hub.imu.heading() < target_angle:
        left_velocity = left_speed
        right_velocity = right_speed
        m_left.run(left_velocity)
        m_right.run(right_velocity)

        while hub.imu.heading() < (target_angle - 20):
            pass
        if right_velocity == 0:
            m_left.run(40*left_velocity/abs(left_velocity))
            m_right.run(0)
        elif left_velocity == 0:
            m_left.run(0)
            m_right.run(40*right_velocity/abs(right_velocity))
        else:
            m_left.run(40*left_velocity/abs(left_velocity))
            m_right.run(40*right_velocity/abs(right_velocity))
        while hub.imu.heading() < target_angle:
            pass
    
    # Left turn
    if hub.imu.heading() > target_angle:
        left_velocity = -left_speed
        right_velocity = -right_speed
        m_left.run(left_velocity)
        m_right.run(right_velocity)

        while hub.imu.heading() > (target_angle + 20):
            pass
        if right_velocity == 0:
            m_left.run(40*left_velocity/abs(left_velocity))
            m_right.run(0)
        elif left_velocity == 0:
            m_left.run(0)
            m_right.run(40*right_velocity/abs(right_velocity))
        else:
            m_left.run(40*left_velocity/abs(left_velocity))
            m_right.run(40*right_velocity/abs(right_velocity))
        while hub.imu.heading() > target_angle:
            pass

    # Stop and hold motors at current position
    m_left.hold()
    m_right.hold()

    # Reset both wheel motor encoders
    m_left.reset_angle(0)
    m_right.reset_angle(0)


# Initialize button variables
pressed = []
run_number = 1
hub.display.char(str(run_number))
last_buttons = ()




" Example Code "
# straightPID(velocity = 30, target = 500, direction = 0)
    # velocity: velocity of the wheel motors
    # target: average wheel motor encoder value you want to move to 
    # direction: the heading angle at which the robot should be driving along
# turn(left_speed = 150, right_speed = 150, target_angle = 45)
    # left_speed: left wheel motor speed 
    # right speed: right wheel motor speed
    # target_angle: angle heading you want to turn to
# FB_Attachment(speed = 200, angle = 180, pause = True)
    # speed: attachment motor speed
    # angle: how many degrees you want the attachment motor to rotate
    # pause: True or False on whether to wait until attachment motion completes before moving to next line
# LR_Attachment(speed = 100, angle = -75, pause = False)
    # speed: attachment motor speed
    # angle: how many degrees you want the attachment motor to rotate
    # pause: True or False on whether to wait until attachment motion completes before moving to next line




" Run Code "

# Run 1: Right side collection (3 krill, reef segment, plankton sample, unknown creature) + Change Shipping Lanes
# Previously Runs 1 and 2, now combined
def run1(): 
    runbegin_time = stop_watch.time()

    #Mesh all the gears when we start
    #rotate all the motors slightly so that their gears lock with the gears they are connected to
    fit_tools()

    m_left.dc(duty=-30)
    m_right.dc(duty=30)

    wait(200)

    #lock up the wheel motors so that they dont move any more
    m_left.hold()
    m_right.hold()

    #Reset gyro angle
    hub.imu.reset_heading(0)
    print(hub.imu.heading())
     
    # Reset both wheel motor encoders
    m_left.reset_angle(0)
    m_right.reset_angle(0)
    #Straight a bit
    accel_straightPID(initial_velocity=20, final_velocity=50, target=200, direction=0)
    m_left.hold()
    m_right.hold() 
    #wait(200)
    #turn(100,100,-15)

    print(hub.imu.heading())
    
    #wait(300)
    #turn_new(left_speed=350, right_speed=0, target_angle=-15)
    #Turn left to avoid shipping lanes
    turn_new(left_speed=200, right_speed=0, target_angle=-21, approach_angle=15, min_speed=50)
    #wait(200)
    print(hub.imu.heading())
    
    #LR_Attachment(80,165,False)
    #straightPID(100,340,-60)
    #turn(100,100,1)
    
    #straightPID(60,990,-15)
    #Go straight
    accel_straightPID(initial_velocity=20, final_velocity=40, target=300, direction=-21)
    straightPID(40,500,-21)
    accel_straightPID(initial_velocity=40, final_velocity=0, target=750, direction=-21)
    m_left.hold()
    m_right.hold() 


    print(hub.imu.heading())
    #wait(300)
    #Turn right
    turn_new(left_speed=200, right_speed=0, target_angle=34, approach_angle=15, min_speed=50)
    print(hub.imu.heading())
    #Straight toward whale
    #accel_straightPID(initial_velocity=20, final_velocity=60, target=25, direction=33)
    straightPID(60,75,34)
    #accel_straightPID(initial_velocity=60, final_velocity=0, target=75, direction=33)
    m_left.hold()
    m_right.hold() 
    print(hub.imu.heading())
    
    LR_Attachment(600,165,False)
    wait(300)

    print(hub.imu.heading())
    turn_new(left_speed=200, right_speed=0, target_angle=15, approach_angle=15, min_speed=50)
    print(hub.imu.heading())

    #accel_straightPID(initial_velocity=-20, final_velocity=-60, target=-100, direction=15)
    #accel_straightPID(initial_velocity=-60, final_velocity=0, target=-220, direction=15)
    straightPID(-30,-220,15)
    m_left.hold()
    m_right.hold() 
    #turn(100,100,-25) 
    
    print(hub.imu.heading())
    turn_new(left_speed=200, right_speed=0, target_angle=-15, approach_angle=15, min_speed=50)
    print(hub.imu.heading())

    accel_straightPID(initial_velocity=-20, final_velocity=-70, target=-200, direction=-15)
    accel_straightPID(initial_velocity=-70, final_velocity=0, target=-500, direction=-15)
    m_left.hold()
    m_right.hold()

    print(hub.imu.heading())
    turn_new(left_speed=300, right_speed=0, target_angle=-40, approach_angle=15, min_speed=50)
    print(hub.imu.heading()) 
    #wait(200)
    
    #this is running into octopus
    #accel_straightPID(initial_velocity=0, final_velocity=60, target=575, direction=-40)
    m_left.dc(duty=70)
    m_right.dc(duty=-70)
    wait(1300)
    m_left.hold()
    m_right.hold()
    
    m_left.reset_angle(0)
    m_right.reset_angle(0)

    """
    m_left.dc(duty=30)
    m_right.dc(duty=-30)

    #wait for enough time for the robot to hit and stick to the reference model or wall
    wait(400)
    m_left.hold()
    m_right.hold()
    """
    accel_straightPID(initial_velocity=-10, final_velocity=-30, target=95, direction=-40)
    m_left.hold()
    m_right.hold()
    wait(200)
    print(hub.imu.heading())
    FB_Attachment(200,250,False)
    turn_new(left_speed=150, right_speed=0, target_angle=-132.5, approach_angle=15, min_speed=50) #133.5
    print(hub.imu.heading()) 
    wait(200)
    
    accel_straightPID(initial_velocity=-20, final_velocity=-40, target=-270, direction=-133.5) #255
    #accel_straightPID(initial_velocity=-50, final_velocity=0, target=-250, direction=-133)
    m_left.hold()
    m_right.hold()
    wait(200)
    
    #front_back_attachment_motor.dc(-50)
    #wait(200)
    #front_back_attachment_motor.hold()
    
    FB_Attachment(500,-325,False)
    wait(1200)
    
    accel_straightPID(initial_velocity=30, final_velocity=70, target=-170, direction=-160)
    m_left.hold()
    m_right.hold()

    #LR_Attachment(600,-165,False)

    turn_new(left_speed=200, right_speed=200, target_angle=-220, approach_angle=10, min_speed=50)
    print(hub.imu.heading()) 

    LR_Attachment(600,-165,False)

    straightPID(velocity=100, target=500, direction=-200) #235
    m_left.hold()
    m_right.hold()

    runend_time = stop_watch.time()
    print("Run 1: ",(runend_time - runbegin_time)/1000)
    

    

# Sonar Discovery, collect seabed sample, Left side collection (water sample, krill, 2 reef segments)
# Previously Run 3
def run2(): 
    runbegin_time = stop_watch.time()

    #rotate all the motors slightly so that their gears lock with the gears they are connected to
    fit_tools()

    #move your robot back so that it is tightly in its starting point
    #first set the motor torque to 30 for each wheel motor so that when the wheel moves
    #back it will not have enough power to keep running once it touches the wall or reference model
    #A reference model is a set of lego pieces we use to set the starting point. A reference model
    #should be built such that when the robot moves back into it, the reference model should be stable
    #and should not move when the robot hits it
    #The torques have opposite values for each motor as one wheel's motor is points in a different direction
    #and we need a negative value for it to go in the same direction as the other wheel
    #m_left.dc(duty=-30)
    #m_right.dc(duty=30)

    #wait for enough time for the robot to hit and stick to the reference model or wall
    wait(200)

    #lock up the wheel motors so that they dont move any more
    m_left.hold()
    m_right.hold() 

    #Reset gyro angle
    hub.imu.reset_heading(0)
    print(hub.imu.heading())
     
    # Reset both wheel motor encoders
    m_left.reset_angle(0)
    m_right.reset_angle(0)

    accel_straightPID(initial_velocity=-20,final_velocity=-30,target=-100, direction=0)   #accel a bit toward change ship
    accel_straightPID(initial_velocity=-30,final_velocity=0,target=-210, direction=0)    #decel toward change ship
    m_left.hold()                       #Hold Left Motor
    m_right.hold()                      #Hold Right Motor
    print("After straight1=0: " + str(hub.imu.heading()))

    #wait(200)
    #Turn to get out of way of shipping lanes         
    turn_new(left_speed=200, right_speed=0, target_angle=-30, approach_angle=15, min_speed=50) 
    print("After turn 1=-30: " + str(hub.imu.heading()))
    
    FB_Attachment(200,100,False) 

    #wait(500)
    accel_straightPID(initial_velocity=-20,final_velocity=-50,target=-100, direction=-30)   
    accel_straightPID(initial_velocity=-50,final_velocity=0,target=-445, direction=-30)   
    m_left.hold()                       #Hold Left Motor
    m_right.hold()                      #Hold Right Motor
    #wait(200)
    print("After straight2=-30: " + str(hub.imu.heading()))
    
    turn_new(left_speed=0, right_speed=200, target_angle=0, approach_angle=15, min_speed=50)
    #wait(200)
 
    print("After turn 2=0: " + str(hub.imu.heading()))
    accel_straightPID(initial_velocity=-20,final_velocity=-40,target=-250, direction=0) #Accelerate straight toward wall  
    accel_straightPID(initial_velocity=-40,final_velocity=0,target=-510, direction=0)   #525 Decelerate straight toward wall  
    m_left.hold()                       #Hold Left Motor
    m_right.hold()                      #Hold Right Motor
    #wait(200)
    print("After straight 3=0: " + str(hub.imu.heading()))

    turn_new(left_speed=200, right_speed=0, target_angle=90, approach_angle=15, min_speed=50) #Turn to 90 to get in parallel with Sonar Discoveries
    print("After turn before Sonar=90: " + str(hub.imu.heading()))
    wait(200)
    accel_straightPID(initial_velocity=20,final_velocity=70,target=300, direction=90)  #Accelerate toward Sonar Discoveries 
    straightPID(70,1200,90)  #Accelerate toward Sonar Discoveries 
    accel_straightPID(initial_velocity=70,final_velocity=0,target=1535, direction=90)  #Accelerate toward Sonar Discoveries 
    m_left.hold()                       #Hold Left Motor
    m_right.hold()                      #Hold Right Motor
    print("Reaced End of big run=90: " + str(hub.imu.heading()))  
    
    turn_new(left_speed=200, right_speed=0, target_angle=85, approach_angle=15, min_speed=50)

    LR_Attachment(200,135,False)        #Drop the pushing thingie
    wait(500)
    
    accel_straightPID(initial_velocity=20,final_velocity=30,target=50, direction=80) #Accelerate toward Sonar Discoveries  
    accel_straightPID(initial_velocity=30,final_velocity=0,target=370, direction=80) #Accelerate toward Sonar Discoveries  
    
    #accel_straightPID(5,45,150,75)      #Drive away with anchor!!!
    #accel_straightPID(45,0,430,75)      #Decel and hope we don't crash into the coral reef...
    m_left.hold()                       #Hold Left Motor
    m_right.hold()  
    

    turn_new(left_speed=0, right_speed=200, target_angle=42.5, approach_angle=25, min_speed=50)
    #FB_Attachment(200,110,True)          #Extend Sonar Discoveries arm
    accel_straightPID(initial_velocity=20,final_velocity=80,target=155, direction=42.5) #Accelerate toward Sonar Discoveries 
    LR_Attachment(40,-135,False)        #Undrop the pushing thingie 
    accel_straightPID(initial_velocity=80,final_velocity=100,target=1250, direction=42.5) #Accelerate toward Sonar Discoveries 
    print("After turning towards home=45: " + str(hub.imu.heading()))   
    m_left.hold()                       #Hold Left Motor
    m_right.hold()  
    runend_time = stop_watch.time()
    print("Run 2: ",(runend_time - runbegin_time)/1000) 

# Coral Nursery, collect scuba diver, deliver reef segments
# Previously Run 4
def run3(): 
    runbegin_time = stop_watch.time()

    #Mesh all the gears when we start
    fit_tools()
    m_left.dc(duty=-30)
    m_right.dc(duty=30)

    #wait for enough time for the robot to hit and stick to the reference model or wall
    wait(300)

    #lock up the wheel motors so that they dont move any more
    m_left.hold()
    m_right.hold() 

    #Reset gyro angle
    hub.imu.reset_heading(0)
    print(hub.imu.heading())
     
    # Reset both wheel motor encoders
    m_left.reset_angle(0)
    m_right.reset_angle(0)

    FB_Attachment(speed=100, angle=120, pause=False)

    #straightPID(velocity=50, target=450, direction=0)
    accel_straightPID(initial_velocity=0, final_velocity=42, target=200, direction=0)
    #straightPID(velocity=50, target=350, direction=0)
    accel_straightPID(initial_velocity=42, final_velocity=0, target=500, direction=0)
    m_left.hold()
    m_right.hold() 
   
   #picking up coral and putting it on hook

    #OLD WORKING CODE - INCONSISTENT
    #LR_Attachment(speed=2000, angle=-1500, pause=False)
    #accel_straightPID(initial_velocity=10, final_velocity=20, target=730, direction=0) #700
    #accel_straightPID(initial_velocity=20, final_velocity=0, target=800, direction=0)
    #m_left.hold()
    #m_right.hold() 

    LR_Attachment(speed=1000, angle=-1500, pause=False)
    accel_straightPID(initial_velocity=10, final_velocity=16, target=700, direction=0) #700
    accel_straightPID(initial_velocity=16, final_velocity=0, target=800, direction=0)
    m_left.hold()
    m_right.hold()


  #letting go of coral
    LR_Attachment(speed=2500, angle=500, pause=False)
    wait(300)

  #putting down right attachment and picking up diver
    FB_Attachment(speed=500, angle=-120, pause=False)
    wait(500)
    
    m_left.dc(duty=-100)
    m_right.dc(duty=-100)

    #wait for enough time for the robot to hit and stick to the reference model or wall
    wait(1000)

    #lock up the wheel motors so that they dont move any more
    m_left.hold()
    m_right.hold() 
    #turn_new(left_speed=300, right_speed=300, target_angle=-5) #-6.3
    
    # Reset both wheel motor encoders
    m_left.reset_angle(0)
    m_right.reset_angle(0)
    #straightPID(velocity=10, target=10, direction=0)
    #m_left.hold()
    #m_right.hold() 
    FB_Attachment(speed=300, angle=200, pause=True)
    #front_back_attachment_motor.dc(60)
    #wait(250)
    #front_back_attachment_motor.hold()
  #going to left home
    #wait(300)
    
    straightPID(velocity=-85, target=-730, direction=0)
    m_left.hold()
    m_right.hold()

    runend_time = stop_watch.time()
    print("Run 3: ",(runend_time - runbegin_time)/1000)


def run4():  
    runbegin_time = stop_watch.time()

    #Mesh all the gears when we start
    #fit_tools()
    m_left.dc(duty=-30)
    m_right.dc(duty=30)

    #wait for enough time for the robot to hit and stick to the reference model or wall
    wait(200)

    #lock up the wheel motors so that they dont move any more
    m_left.hold()
    m_right.hold() 

    #Reset gyro angle
    hub.imu.reset_heading(0)
    print(hub.imu.heading())
     
    # Reset both wheel motor encoders
    m_left.reset_angle(0)
    m_right.reset_angle(0)

    # Extend diver slider
    FB_Attachment(speed=500, angle=945, pause=False)

    # Drive toward wall, start retracting diver arm at end
    accel_straightPID(initial_velocity=20, final_velocity=70, target=300, direction=0)
    straightPID(velocity=70, target=600, direction=0)
    accel_straightPID(initial_velocity=70, final_velocity=0, target=1400, direction=0)
    m_left.hold()
    m_right.hold()
    wait(200)
    FB_Attachment(speed=800, angle=-800, pause=False)
    
    # Set up dc to run into wall
    m_left.dc(duty=40)
    m_right.dc(duty=-40)
    #wait for enough time for the robot to hit and stick to the reference model or wall
    wait(1000)
    #lock up the wheel motors so that they dont move any more
    m_left.hold()
    m_right.hold()

    # Reset both wheel motor encoders
    m_left.reset_angle(0)
    m_right.reset_angle(0)

    # Hit Shark and Coral Reef levers
    LR_Attachment(speed=300, angle=-155, pause=False) #-145
    wait(600)
    LR_Attachment(speed=500, angle=140, pause=False)
    wait(10)

    # Drive back away from wall
    accel_straightPID(initial_velocity=-20, final_velocity=-60, target=-250, direction=0)
    #straightPID(velocity=-60, target=-250, direction=0)
    accel_straightPID(initial_velocity=-60, final_velocity=0, target=-570, direction=0) #-575   
    m_left.hold()
    m_right.hold()

    # Turn toward Raise the Mast, drive into mission to raise mast and capture treasure chest
    turn_new(left_speed=250, right_speed=0, target_angle=-89, approach_angle=15, min_speed=50)
    """
    accel_straightPID(initial_velocity=-20, final_velocity=-45, target=-200, direction=-89)
    straightPID(velocity=-45, target=-400, direction=-89)
    accel_straightPID(initial_velocity=-45, final_velocity=0, target=-610, direction=-89)
    m_left.hold()
    m_right.hold()
    """
    accel_straightPID(initial_velocity=-20, final_velocity=-60, target=-150, direction=-89)
    accel_straightPID(initial_velocity=-60, final_velocity=0, target=-300, direction=-89)
    m_left.dc(-60)
    m_right.dc(60)
    wait(500)
    m_left.hold()
    m_right.hold()
    # Reset both wheel motor encoders
    m_left.reset_angle(0)
    m_right.reset_angle(0)

    # Pause briefly 
    wait(300)

    # Drive away from mission with treasure chest
    accel_straightPID(initial_velocity=20, final_velocity=30, target=200, direction=-89) #-450
    m_left.hold()
    m_right.hold()

    # Turn toward and drive into home area
    turn_new(left_speed=0, right_speed=350, target_angle=-135, approach_angle=15, min_speed=50)
    #accel_straightPID(initial_velocity=0, final_velocity=100, target=225, direction=-165)
    straightPID(velocity=100, target=850, direction=-135)
    #accel_straightPID(initial_velocity=100, final_velocity=0, target=775, direction=-165)
    m_left.hold()
    m_right.hold()

    runend_time = stop_watch.time()
    print("Run 4: ",(runend_time - runbegin_time)/1000)




# Deliver 3 samples into Research Vessel, deliver shark, push Research Vessel across board
# Previously Run 6
def run5():  
    runbegin_time = stop_watch.time()

    #Reset gyro angle
    hub.imu.reset_heading(0)
    print(hub.imu.heading())
     
    m_left.reset_angle(0)
    m_right.reset_angle(0)
    accel_straightPID(initial_velocity=-20, final_velocity=-70, target=-300, direction=2)
    straightPID(-70,-1390,2)
    accel_straightPID(initial_velocity=-70, final_velocity=0, target=-1690, direction=2)
    m_left.hold()
    m_right.hold() 
    #straightPID(-70,-1690,1)
    wait(500)
    LR_Attachment(200,120,False)
    FB_Attachment(600,150,True)
    #wait(500)
    #LR_Attachment(200,-90,False)
    straightPID(15,-1500,2)
    m_left.hold()
    m_right.hold() 
    
    #FB_Attachment(600,700,True)
    
    m_right.hold()
    """turn(200,200,-40)
    straightPID(-30,-70,-26)
    straightPID(-100,-1800,2)
    m_left.hold()
    m_right.hold() """

    runend_time = stop_watch.time()
    print("Run 5: ",(runend_time - runbegin_time)/1000)




# Feed the Whale (4 krill), Send over the Submersible, collect trident handle, Angler Fish, deliver unknown creature
# Previously Run 7
def run6():  
    #front_back_attachment_motor.dc(90)
    #wait(400)
    #front_back_attachment_motor.hold()
    #return

    runbegin_time = stop_watch.time()

    #Mesh all the gears when we start
    fit_tools_onlywheels()

    m_left.dc(duty=30)
    m_right.dc(duty=-30)

    #wait for enough time for the robot to hit and stick to the reference model or wall
    wait(100)

    #lock up the wheel motors so that they dont move any more
    m_left.hold()
    m_right.hold() 
    
    #Reset gyro angle
    hub.imu.reset_heading(0)
    print(hub.imu.heading())
     
    # Reset both wheel motor encoders
    m_left.reset_angle(0)
    m_right.reset_angle(0)
    
    #LR_Attachment(200,-10,True)

    print("At Start: " + str(hub.imu.heading()))
    accel_straightPID(-20,-80,-300,0)
    print("After first acceleration: " + str(hub.imu.heading()))
    straightPID(-80, -1025, 0)
    print("After constant speed run: " + str(hub.imu.heading()))
    accel_straightPID(-80,0,-1330,0)
    #accel_straightPID(-70,0,-1350,0)
    m_left.hold()
    m_right.hold()
    ##wait(200)
    print("After deceleration, before Turn of 45: " + str(hub.imu.heading()))
    
    turn_new(left_speed=200, right_speed=0, target_angle=45, approach_angle=15, min_speed=50)

    ##wait(200)
    print("After 45 turn: " + str(hub.imu.heading()))
    
    m_left.dc(-70)
    m_right.dc(70)
    wait(750)
    m_left.hold()
    m_right.hold()
    print("After aligning with Whale: " + str(hub.imu.heading()))

    LR_Attachment(200,80,True)
    wait(400)
    #Feed Whale
    LR_Attachment(200,-80,True)
    ##wait(200)
    # Reset both wheel motor encoders
    m_left.reset_angle(0)
    m_right.reset_angle(0)
    straightPID(30,300,45)
    m_left.hold()
    m_right.hold()
    print("After coming out of Whale: " + str(hub.imu.heading()))

    turn_new(left_speed=200, right_speed=0, target_angle=90, approach_angle=15, min_speed=50)
    ##wait(200)
    print("After turn: " + str(hub.imu.heading()))

   #Toward Sub Mission
    accel_straightPID(20,60,250,0)
    print("After first acceleration: " + str(hub.imu.heading()))
    accel_straightPID(60,0,540,0)
    m_left.hold()
    m_right.hold()
    print("After deceleration: " + str(hub.imu.heading()))

    #front_back_attachment_motor.dc(-90)
    #wait(250)
    #front_back_attachment_motor.hold()
    FB_Attachment(180,-400,False) #300
    turn_new(left_speed=200, right_speed=0, target_angle=135, approach_angle=15, min_speed=50)
    print("After turn: " + str(hub.imu.heading()))
    wait(200)

    m_left.dc(70)
    m_right.dc(-70)
    wait(1000)
    m_left.hold()
    m_right.hold()
    print("After chanelling sub: " + str(hub.imu.heading()))

    #Send Over The Sub
    #FB_Attachment(180,320,True)
    front_back_attachment_motor.dc(75)
    wait(850)
    front_back_attachment_motor.hold()
    
    #Back out of sub
    # Reset both wheel motor encoders
    m_left.reset_angle(0)
    m_right.reset_angle(0)
    #straightPID(-30,-200,135)
    accel_straightPID(-10,-30,-90,135)
    print("After first acceleration: " + str(hub.imu.heading()))
    m_left.hold()
    m_right.hold()
    print("After backing out of sub: " + str(hub.imu.heading()))
    #wait(200)
    #turn towards the angler fish

    turn_new(left_speed=0, right_speed=200, target_angle=228, approach_angle=15, min_speed=50)
    ##wait(200)
    FB_Attachment(350,-320,False)
    accel_straightPID(-20,-40,-195,228) #-10, -30, Go forward towards the Trident
    m_left.hold()
    m_right.hold()
    #wait(200)
    #Reached the angler fish - pick up the trident

    #wait(300)
    turn_new(left_speed=0, right_speed=200, target_angle=245) #, approach_angle=15, min_speed=50

    front_back_attachment_motor.dc(60)
    wait(500)
    front_back_attachment_motor.hold()

    turn_new(left_speed=400, right_speed=0, target_angle=264) #, approach_angle=15, min_speed=50
 
    #front_back_attachment_motor.dc(75)
    #wait(200)
    #front_back_attachment_motor.hold()

    straightPID(-80,-180,265)
    m_left.hold()
    m_right.hold()
 
    runend_time = stop_watch.time()
    print("Run 6: ",(runend_time - runbegin_time)/1000)


""" Permanent loop to process any button that is pressed """
while True:
    buttons = hub.buttons.pressed()
    released_buttons = set(last_buttons) - set(buttons)

    """ Process the button that was pressed """
    if (Button.LEFT in released_buttons):
        run_number = run_number + 1
        if run_number > 6:
            run_number = 1
        hub.display.char(str(run_number))
    
    if (Button.RIGHT in released_buttons):
        if run_number == 1:
                run1()
                run_number = run_number + 1
                hub.display.char(str(run_number))
        elif run_number == 2:
                run2()
                run_number = run_number + 1
                hub.display.char(str(run_number))
        elif run_number == 3:
                run3()
                run_number = run_number + 1
                hub.display.char(str(run_number))
        elif run_number == 4:
                run4()
                run_number = run_number + 1
                hub.display.char(str(run_number))
        elif run_number == 5:
                run5()
                run_number = run_number + 1
                hub.display.char(str(run_number))
        elif run_number == 6:
                run6()
                run_number = 1
                hub.display.char(str(run_number))

    last_buttons = buttons
