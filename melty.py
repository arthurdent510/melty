import time
import board
import pulseio
from digitalio import DigitalInOut, Direction, Pull
from adafruit_motor import servo, motor


in1 = DigitalInOut(board.D13)
in1.direction = Direction.OUTPUT

in2 = DigitalInOut(board.D12)
in2.direction = Direction.OUTPUT

pwm = pulseio.PWMOut(board.A1, frequency=50)
 
# Create a servo object, my_servo.
my_servo = servo.ContinuousServo(pwm)
# foo = motor.DCMotor(pwm)


leftStop = DigitalInOut(board.D2)
leftStop.direction = Direction.INPUT
leftStop.pull = Pull.UP

rightStop = DigitalInOut(board.D3)
rightStop.direction = Direction.INPUT
rightStop.pull = Pull.UP

# in1 and in2 to 0 is break
# in1 and in2 to 1 is coast
# in1 to 1 and in2 to 0, forward
# in1 to 0 and in2 to 1, reverse

my_servo.throttle = 0

time.sleep(5)

# my_servo.throttle = .5



in1.value = True
in2.value = False

direction = False

while True:
    if rightStop.value and direction:
        in1.value = False
        in2.value = False
        time.sleep(0.5)

        in1.value = True
        in2.value = False

        direction = False

        

    if leftStop.value and not direction:
        in1.value = False
        in2.value = False
        time.sleep(0.5)

        in1.value = False
        in2.value = True

        direction = True

       
    time.sleep(0.01)