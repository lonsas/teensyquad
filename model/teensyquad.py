from ctypes import *
import os
from enum import Enum
import numpy as np
import math

class TeensyQuadState(Enum):
    STARTUP = 0
    READY_WAIT = 1
    ARMED = 2
    USB_CONNECTED = 3

class QuadControl(Enum):
    ROLL =  0
    PITCH =  1
    THROTTLE = 2
    YAW = 3
    AUX1 = 4
    AUX2 = 5

class TeensyQuad:
    def __init__(self):
        dll_name = "teensyquad.so"
        dll_path = os.path.dirname(__file__) + "/../build/model/"
        self.quad = CDLL(dll_path + dll_name)
        self.quad.stateInit()
        self.setDefaultSticks()

    def setCommands(self, commands):
        if(len(commands) != 6):
            print("setCommand wrong usage")
            return;
        # Pulse width is between 1000 and 2000 micro seconds, scale
        commands = [int(x * 1000 + 1000) for x in commands]
        pulse_width = (c_int * len(commands))(*commands)
        self.quad.receiverSetAllManualPW(pulse_width)

    def setCommand(self, signal, command):
        pulse_width = command * 1000 + 1000
        self.quad.receiverSetManualPW(signal.value, int(pulse_width))

    def setMotion(self, acceleration, rotational_velocity):
        ACCELERATION_MAX = 2.0*9.82 #m/s^2
        ROTATIONAL_VELOCITY_MAX = math.radians(500.0) #rad/s
        DATA_WIDTH = 15 #bits
        RAW_DATA_MAX = 2**15-1

        a_scale = (2**DATA_WIDTH)/ACCELERATION_MAX
        r_scale = (2**DATA_WIDTH)/ROTATIONAL_VELOCITY_MAX
        a = [a_scale * x for x in acceleration]
        r = [r_scale * x for x in rotational_velocity]

        r = np.clip(r, -RAW_DATA_MAX, RAW_DATA_MAX)
        tmp = a[0]
        a[0] = -a[1]
        a[1] = tmp

        # The accelerometer will read the force opposing gravity
        a[0] = -a[0]
        a[1] = -a[1]
        a[2] = -a[2]

        self.quad.setMotion6(int(a[0]), int(a[1]), int(a[2]), int(r[0]), int(r[1]), int(r[2]))

    def getMotors(self):
        motor_analog_values = (c_int * 4)(*[0, 0, 0, 0])
        self.quad.getMotorOutput(motor_analog_values)
        PWM_RES = 12
        PWM_RATE = 400

        to_micro_scaling = 1/(PWM_RATE*(2**PWM_RES)/1000000.0)
        motors = [(x * to_micro_scaling - 1000)/1000 for x in motor_analog_values]
        return motors

    def doIteration(self):
        self.quad.stateUpdate()
        self.quad.stateDo()

    def arm(self, arm):
        if(arm):
            self.setCommand(QuadControl.AUX2, 1)
        else:
            self.setCommand(QuadControl.AUX2, 0)

    def setThrottle(self, throttle):
        self.setCommand(QuadControl.THROTTLE, throttle)

    def setYawStick(self, yaw):
        self.setCommand(QuadControl.YAW, yaw / 2.0 + 0.5)

    def setRollStick(self, roll):
        self.setCommand(QuadControl.ROLL, roll / 2.0 + 0.5)

    def setPitchStick(self, pitch):
        self.setCommand(QuadControl.PITCH, pitch / 2.0 + 0.5)

    def setDefaultSticks(self):
        self.setCommands([0.5, 0.5, 0, 0.5, 0, 0])

    def getState(self):
        return TeensyQuadState(int(self.quad.getCurrState()))

    def setAUX1(self, value):
        self.setCommand(QuadControl.AUX1, value)

    def getSensorAngle(self):
        roll = c_double()
        pitch = c_double()
        yaw = c_double()
        self.quad.SensorGetAngle(byref(roll), byref(pitch), byref(yaw))
        return [roll.value, pitch.value, yaw.value]

    def getSensorOmega(self):
        roll = c_double()
        pitch = c_double()
        yaw = c_double()
        self.quad.SensorGetOmega(byref(roll), byref(pitch), byref(yaw))
        return [roll.value, pitch.value, yaw.value]

    def update(self, acceleration, rotational_velocity):
       self.setMotion(acceleration, rotational_velocity)
       self.quad.stateUpdate()
       self.quad.stateDo()
       return self.getMotors()

    def gotoArmed(self):
       self.doIteration()
       self.arm(True)
       self.doIteration()

    def printState(self):
        print("State: {0}, motors: {1} Sensor: a:{2} o:{3}".format(self.getState(), self.getMotors(), self.getSensorAngle(), self.getSensorOmega()))

if __name__ == "__main__":
    print("Starting...")
    quad = TeensyQuad()
    for _ in range(10):
        quad.doIteration()
        quad.printState()

    print("Arming...")
    quad.arm(True)
    for _ in range(10):
        quad.doIteration()
        quad.printState()

    print("Throttle...")
    for _ in range(10):
        quad.setThrottle(1)
        quad.update([0,0,0], [0,0,0])
        quad.printState()

    print("Disturbance...")
    for _ in range(10):
        quad.update([0,0,1], [0,0.1,0])
        quad.printState()
