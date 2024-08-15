import numpy as np
import math

class PID:
    def __init__(self,steer_kp = 1.,steer_ki=0.0045,steer_kd=0.0017,vel_kp = 1.,vel_ki=0.0045,vel_kd=0.0017,dt = .01):
        self.steer_kp = steer_kp
        self.steer_ki = steer_ki
        self.steer_kd = steer_kd
        self.vel_kp = vel_kp
        self.vel_ki = vel_ki
        self.vel_kd = vel_kd
        self.dt = dt


    def steerControl(self,x,x_d,prev_e,prev_int):
        unit_theta = np.array([
            [math.cos(x[2,0])],
            [math.sin(x[2,0])]
        ])
        ref = x_d[:2]-x[:2]
        e = np.cross(unit_theta.T,ref.T)[0]
        e_int = prev_int+e*self.dt
        e_int = max(min(e_int,0.5),-0.5)
        e_der = (e-prev_e)/self.dt

        u_steer = self.steer_kp*e+self.steer_ki*e_int+self.steer_kd*e_der
        u_steer = max(min(u_steer,0.7),-0.7)
        return u_steer, e, e_int

    def speedControl(self,x,x_d,prev_e,prev_int):
        unit_theta = np.array([
            [math.cos(x[2,0])],
            [math.sin(x[2,0])]
        ])
        ref = x_d[:2]-x[:2]
        e = np.dot(unit_theta.T,ref)[0,0]
        e_int = prev_int+e*self.dt
        e_int = max(min(e_int,0.5),-0.5)
        e_der = (e-prev_e)/self.dt

        u_vel = self.vel_kp*e+self.vel_ki*e_int+self.vel_kd*e_der
        u_vel = max(min(u_vel,0.7),-0.7)
        return u_vel, e, e_int

        
