from teensyquad import TeensyQuad
from teensyquad import TeensyQuadState
import params
import numpy as np
from math import *
import matplotlib.pyplot as plt
T = 2
dt = 0.001
sim_dt = dt/1
control_lag = int(0.00/dt)
signal_lag = int(0.000/dt)

omega_signal_disturbance_std = 0.05 #rad/s
omega_signal_disturbance_mean = 0.0
acc_signal_disturbance_std = 0.5 #m/s^2
acc_signal_disturbance_mean = 0.0
load_disturbance_std = np.array([0, 0.01, 0.01, 0.01])
load_disturbance_mean0 = np.array([0, 0.0, 0, 0]) #Nm
load_disturbance_mean1 = np.array([0, 0.0, 0, 0]) #Nm
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
        load_disturbance_mean = load_disturbance_mean1
    else:
        load_disturbance_mean = load_disturbance_mean0
        load_disturbance_mean[1] = -FM[1] - 0.0005
    FM += np.random.normal(load_disturbance_mean, load_disturbance_std)
    return FM

def teensyquad_update_loop(teensyquad, simquad):
    omega_log = np.zeros(shape=[signal_lag+1,3])
    angle_log = np.zeros(shape=[signal_lag+1,3])
    v_log = np.zeros(shape=[signal_lag+1,3])
    FM_log = np.zeros(shape=[control_lag,4])

    for time in np.arange(0, T, dt):

        v = v_log[-(signal_lag),:]
        omega = omega_log[-(signal_lag+1),:].copy()

        #omega += np.random.normal(omega_signal_disturbance_mean, omega_signal_disturbance_std)
        #v += np.random.normal(acc_signal_disturbance_mean, acc_signal_disturbance_std)

        motor = teensyquad.update(v, omega)
        FM = motor_to_phys(motor)
        FM_log = np.vstack((FM_log, FM))
        FM = FM_log[-(control_lag+1),:].copy() # Lag

        FM = add_load_disturbance(FM, time)

        for _ in range(0,int(dt/sim_dt)):
            simquad.stateUpdate(FM[0], FM[1:4], sim_dt)
        omega_log = np.vstack((omega_log, simquad.omega()))
        angle_log = np.vstack((angle_log, simquad.angle()))
        v_log = np.vstack((v_log, simquad.gravity()))
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
    #teensyquad.setPitchStick(0.5)
    #teensyquad.setYawStick(-0.5)

    omega_log, FM_log, angle_log = teensyquad_update_loop(teensyquad, simquad)
    sim_time = np.linspace(0,len(omega_log)*dt,len(omega_log))
    control_time = np.linspace(0,len(FM_log)*dt,len(FM_log))
    plt.figure(1)
    plt.subplot(221)
    plt.title('Simulation omega')
    plt.xlabel('t')
    plt.ylabel('rad/s')
    plt.plot(sim_time,omega_log[:,0], label='roll')
    plt.plot(sim_time,omega_log[:,1], label='pitch')
    plt.plot(sim_time,omega_log[:,2], label='yaw')
    plt.grid()
    plt.legend()

    plt.subplot(222)
    plt.title('Simulation Angle')
    plt.xlabel('t')
    plt.ylabel('rad')
    plt.plot(sim_time,angle_log[:,0], label='roll')
    plt.plot(sim_time,angle_log[:,1], label='pitch')
    plt.plot(sim_time,angle_log[:,2], label='yaw')
    plt.grid()
    plt.legend()

    plt.subplot(212)
    plt.title('Control')
    plt.xlabel('t')
    plt.ylabel('Nm')
    plt.plot(control_time,FM_log[:,1], label='roll')
    plt.plot(control_time,FM_log[:,2], label='pitch')
    plt.plot(control_time,FM_log[:,3], label='yaw')
    plt.legend()
    plt.grid()
    plt.show()



if __name__ == "__main__":
    run()
