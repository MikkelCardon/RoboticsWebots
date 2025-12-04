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
    y = height // 2 # center pixel
    
    red_count = 0
    
    for yDiff in range(-10, 10):
        for xDiff in range(-10, 10):
            r = camera.imageGetRed(image, width, x + xDiff, y + yDiff)
            g = camera.imageGetGreen(image, width, x + xDiff, y + yDiff)
            b = camera.imageGetBlue(image, width, x + xDiff, y + yDiff)

            h, s, v = rgb_to_hsv(r, g, b)

            # Red detection in HSV
            # Hue around 0° (or 360°) ±20°, saturation and value thresholds
            if ((h <= 15 or h >= 345) and s > 0.6 and v > 0.3):
                red_count += 1

    if red_count > 5:
        print("Red count: ", red_count)

    if red_count > 10:     
        print("Red detected - stopping")
        red_found = True

def rgb_to_hsv(r, g, b):
    """Convert RGB (0-255) to HSV (0-360°, 0-1, 0-1)."""
    r_, g_, b_ = r / 255.0, g / 255.0, b / 255.0
    cmax = max(r_, g_, b_)
    cmin = min(r_, g_, b_)
    delta = cmax - cmin

    # Hue calculation
    if delta == 0:
        h = 0
    elif cmax == r_:
        h = 60 * (((g_ - b_) / delta) % 6)
    elif cmax == g_:
        h = 60 * (((b_ - r_) / delta) + 2)
    else:
        h = 60 * (((r_ - g_) / delta) + 4)

    # Saturation
    s = 0 if cmax == 0 else delta / cmax

    # Value
    v = cmax

    return h, s, v

def driveForward():
    print("Driving towards target")
    leftMotor.setVelocity(MAX_SPEED * 0.5)
    rightMotor.setVelocity(MAX_SPEED * 0.5)

    while robot.step(TIME_STEP) != -1:
        if(ps[0].getValue() > 80.0 or ps[1].getValue() > 80.0 or ps[7].getValue() > 80.0 or ps[6].getValue() > 80.0):
            print("Arrived at target destination")
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break



# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:

    checkForRed()
    
    if red_found:
        leftMotor.setVelocity(0.0)
        rightMotor.setVelocity(0.0)
        driveForward()
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