from ctypes import *
import os

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
        self.quad.receiverSetManualPW(signal, int(pulse_width))

    def setMotion(self, acceleration, rotational_velocity):
        ACCELERATION_MAX = 2.0*9.82 #m/s^2
        ROTATIONAL_VELOCITY_MAX = 250.0 #rad/s
        DATA_WIDTH = 15 #bits
      
        a_scale = (2**DATA_WIDTH)/ACCELERATION_MAX
        r_scale = (2**DATA_WIDTH)/ROTATIONAL_VELOCITY_MAX
        
        a = [a_scale * min(x, ACCELERATION_MAX) for x in acceleration]
        r = [r_scale * min(x, ROTATIONAL_VELOCITY_MAX) for x in rotational_velocity]

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
            self.setCommand(4, 1)
        else:
            self.setCommand(4, 0)

    def setThrottle(self, throttle):
        self.setCommand(2, throttle)

    def setYawStick(self, yaw):
        self.setCommand(3, yaw + 0.5)

    def setRollStick(self, roll):
        self.setCommand(3, roll + 0.5)

    def setPitchStick(self, pitch):
        self.setCommand(3, pitch + 0.5)

    def setDefaultSticks(self):
        self.setCommands([0.5, 0.5, 0, 0.5, 0, 0])

    def getState(self):
        return int(self.quad.getCurrState())

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

    def update(self, angle, rotation):
       self.setMotion(angle, rotation)
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
