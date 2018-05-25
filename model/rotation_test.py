from teensyquad import TeensyQuad
from teensyquad import TeensyQuadState
import params
import numpy as np
from math import *
import matplotlib.pyplot as plt
T = 2
dt = 0.001
sim_dt = dt/1

signal_disturbance_std = 0.0 #rad/s
signal_disturbance_mean = 0.2
load_disturbance = np.array([0, 0.1, 0.1, 0.1]) #Nm
load_disturbance_t = 1 #s

def motor_to_phys(motors):
    """ Calculates Forces and torque from motor intensities """
    F = [params.MotorF * x for x in motors] # Scale

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

    def gravity(self):
        v = np.array([sin(self.state[0]),
             sin(self.state[1]),
             min(cos(self.state[0]), cos(self.state[1]))])
        v = -9.82*v/np.linalg.norm(v)

        return v

    def omega(self):
        return self.state[3:6].copy()

    def angle(self):
        return self.state[0:3].copy()

def add_load_disturbance(FM, time):
    if(time > load_disturbance_t):
        return FM + load_disturbance
    return FM

def teensyquad_update_loop(teensyquad, simquad):
    omega_log = np.empty(shape=[0,3])
    angle_log = np.empty(shape=[0,3])
    FM_log = np.empty(shape=[0,4])
    for time in np.arange(0, T, dt):

        v = simquad.gravity()
        omega = simquad.omega()

        #omega += np.random.normal(signal_disturbance_mean, signal_disturbance_std)

        motor = teensyquad.update(v, omega)
        FM = motor_to_phys(motor)

        #FM = add_load_disturbance(FM, time)

        for _ in range(0,int(dt/sim_dt)):
            simquad.stateUpdate(FM[0], FM[1:4], sim_dt)
            omega_log = np.vstack((omega_log, simquad.omega()))
            FM_log = np.vstack((FM_log, FM))
            angle_log = np.vstack((angle_log, simquad.angle()))
    return (omega_log, FM_log, angle_log)

def run():
    simquad = QuadModel()
    teensyquad = TeensyQuad()
    teensyquad.gotoArmed()
    if(teensyquad.getState() != TeensyQuadState.ARMED):
        print("Failed to arm, state is: {0}".format(teensyquad.getState()))
        return
    teensyquad.setThrottle(0.4)
    teensyquad.setRollStick(0.1)
    teensyquad.setPitchStick(0.5)
    #teensyquad.setYawStick(-0.5)

    omega_log, FM_log, angle_log = teensyquad_update_loop(teensyquad, simquad)
    sim_time = np.arange(0,T,sim_dt)
    control_time = np.arange(0,T,dt)
    plt.figure(1)
    plt.subplot(221)
    plt.title('Simulation omega')
    plt.xlabel('t')
    plt.ylabel('rad/s')
    plt.plot(sim_time,omega_log[:,0], label='roll')
    plt.plot(sim_time,omega_log[:,1], label='pitch')
    plt.plot(sim_time,omega_log[:,2], label='yaw')
    plt.legend()

    plt.subplot(222)
    plt.title('Simulation Angle')
    plt.xlabel('t')
    plt.ylabel('rad')
    plt.plot(sim_time,angle_log[:,0], label='roll')
    plt.plot(sim_time,angle_log[:,1], label='pitch')
    plt.plot(sim_time,angle_log[:,2], label='yaw')
    plt.legend()

    plt.subplot(212)
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
