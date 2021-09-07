from controller import Robot, DistanceSensor, Motor

# time in [ms] of a simulation step
TIME_STEP = 16

MAX_SPEED = 6.28

STEPS_180 = 84

rotated = False 
rotated2 = False 
# create the Robot instance.
robot = Robot()

# initialize devices
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

    
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

def sleep(duration):
    global robot
    end_time = robot.getTime() + duration
    while robot.step(TIME_STEP) != -1 and robot.getTime() < end_time:
        pass
        
def step():
    if robot.step(TIME_STEP) == -1:
        return
        
def passive_wait(secs):
    start_time = robot.getTime()
    while start_time + secs > robot.getTime():
        step()
    return
    
def forward():
    print("forward")
    #if psValues[4] > 80 or psValues[3] > 80:
        #pass
    #else:    
    leftMotor.setVelocity(0.5 * MAX_SPEED)
    rightMotor.setVelocity(0.5 * MAX_SPEED)
            
def rotate():
    print("rotate")
    leftMotor.setVelocity(0.5 * MAX_SPEED)
    rightMotor.setVelocity(-0.5 * MAX_SPEED)
    passive_wait(1.4)
 
def clockwise():
    print("clockwise")
    while ps[5].getValue() < 85:
        leftMotor.setVelocity(0.5 * MAX_SPEED)
        rightMotor.setVelocity(-0.5 * MAX_SPEED)
        step()
            
def leftWall():
    print("leftwall")
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0) 
    if ps[5].getValue() > 75:
        leftMotor.setVelocity(0.5 * MAX_SPEED)
        rightMotor.setVelocity(0.5 * MAX_SPEED)

def case_set(i):
    cases = {
    1: forward,
    2: rotate,
    3: clockwise,
    4: leftWall
    }
    func = cases.get(i)
    return func()

  
# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    # read sensors outputs
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
        
    
    
    # detect obstacles
    # right_obstacle = psValues[0] > 80.0 or psValues[1] > 80.0 or psValues[2] > 80.0
    #left_obstacle = psValues[5] > 80.0 or psValues[6] > 80.0 or psValues[7] > 80.0
    #84-90 times steps for 180 deg
    front_obstacle = psValues[0] > 80 or psValues[7] > 80 or psValues[6] > 80 or psValues[1] > 80
    #full_turn = psValues[3] >= 80 and psValues [4] >= 80

    # initialize motor speeds at 50% of MAX_SPEED.
    MAX_SPEED = 6.28
    #leftSpeed  = 0.5 * MAX_SPEED
    #rightSpeed = 0.5 * MAX_SPEED
    

    
    #set case   
    if front_obstacle and not rotated:
        case_set(2) #rotate
        rotated = True
    elif front_obstacle and rotated and not rotated2:
        case_set(3) #clockwise
        rotated2 = True
    elif rotated2:
        case_set(4) #look for leftwall
    else:
        case_set(1) #forward

        
           
    # write actuators inputs

    #leftMotor.setVelocity(leftSpeed)
    #rightMotor.setVelocity(rightSpeed)