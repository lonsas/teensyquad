from teensyquad import TeensyQuad
from teensyquad import TeensyQuadState
import params
import numpy as np
from math import *
import matplotlib.pyplot as plt
T = 1
dt = 0.001
sim_dt = dt/1

signal_disturbance = 0.1

def motor_to_phys(motors):
    """ Calculates Forces and torque from motor intensities """
    F = [params.MotorF * x for x in motors] # Scale
    #print(params.A.dot(F))
    #print(F)
    return params.A.dot(F)

class QuadModel:
    def __init__(self):
        self.state = np.array([0,0,0,0,0,0,0,0,0],dtype=float)

    def stateUpdate(self, F, M, dt):
        angle = self.state[0:3]
        omega = self.state[3:6]
        omegaDot = params.invI.dot(M)
        angle = angle + omega * dt
        omega = omega + omegaDot * dt

        self.state[0:3] = angle
        self.state[3:6] = omega
        self.state[6:9] = omegaDot
        #print(omegaDot)
        #print(self.state)
        print(self.state[[0,3,6]])
        #print(self.state[6:])

    def gravity(self):
        v = np.array([sin(self.state[0]),
             sin(self.state[1]),
             max(cos(self.state[0]), cos(self.state[1]))])
        v = 9.82*v/np.linalg.norm(v)

        return v
    def omega(self):
        return self.state[3:6]

def teensyquad_update_loop(teensyquad, simquad):
    omega_log = np.empty(shape=[0,3])
    FM_log = np.empty(shape=[0,4])
    for i in range(0, int(T/dt)):
        v = simquad.gravity()
        omega = simquad.omega()
        print(omega)
        motor = teensyquad.update(v, omega)
        print(motor)
        FM = motor_to_phys(motor)
        FM_log = np.vstack((FM_log, FM))
        for _ in range(0,int(dt/sim_dt)):
            simquad.stateUpdate(FM[0], FM[1:4], sim_dt)
            omega_log = np.vstack((omega_log, simquad.omega()))
    return (omega_log, FM_log)

def run():

    simquad = QuadModel()
    teensyquad = TeensyQuad()
    teensyquad.gotoArmed()
    if(teensyquad.getState() != TeensyQuadState.ARMED):
        print("Failed to arm, state is: {0}".format(teensyquad.getState()))
        return
    teensyquad.setThrottle(0.5)
    teensyquad.setRollStick(1)
    teensyquad.setPitchStick(0.7)
    teensyquad.setYawStick(-0.5)

    omega_log, FM_log = teensyquad_update_loop(teensyquad, simquad)
    sim_time = np.arange(0,T,sim_dt)
    control_time = np.arange(0,T,dt)
    plt.figure(1)
    plt.title('Simulation')
    plt.xlabel('t')
    plt.ylabel('rad/s')
    plt.plot(sim_time,omega_log[:,0], label='roll')
    plt.plot(sim_time,omega_log[:,1], label='pitch')
    plt.plot(sim_time,omega_log[:,2], label='yaw')
    plt.legend()

    plt.figure(2)
    plt.title('Control')
    plt.xlabel('t')
    plt.ylabel('Nm')
    plt.plot(control_time,FM_log[:,1], label='roll')
    plt.plot(control_time,FM_log[:,2], label='pitch')
    plt.plot(control_time,FM_log[:,3], label='yaw')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    run()
