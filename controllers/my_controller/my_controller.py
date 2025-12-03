from controller import Robot, DistanceSensor, Motor

# time in [ms] of a simulation step
TIME_STEP = 64

MAX_SPEED = 6.28

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

camera = robot.getDevice("camera")
camera.enable(TIME_STEP)

red_found = False

def checkForRed(): 
    global red_found

    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()
    
    x = width // 2   # center pixel
    y = height // 2

    red   = camera.imageGetRed(image, width, x, y)
    green = camera.imageGetGreen(image, width, x, y)
    blue  = camera.imageGetBlue(image, width, x, y)
    
    if red > 150 and green < 100 and blue < 100:
        print("Red detected - stopping")
        red_found = True



# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:

    checkForRed()
    
    if red_found:
        leftMotor.setVelocity(0.0)
        rightMotor.setVelocity(0.0)
        break
        
    # read sensors outputs
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())

    # detect obstacles
    right_obstacle = psValues[0] > 80.0 or psValues[1] > 80.0 or psValues[2] > 80.0
    left_obstacle = psValues[5] > 80.0 or psValues[6] > 80.0 or psValues[7] > 80.0

    # initialize motor speeds at 50% of MAX_SPEED.
    leftSpeed  = 0.5 * MAX_SPEED
    rightSpeed = 0.5 * MAX_SPEED
    # modify speeds according to obstacles
    if left_obstacle:
        # turn right
        leftSpeed  = 0.5 * MAX_SPEED
        rightSpeed = -0.5 * MAX_SPEED
        
    elif right_obstacle:
        # turn left
        leftSpeed  = -0.5 * MAX_SPEED
        rightSpeed = 0.5 * MAX_SPEED
        
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)