#!/usr/bin/env python3

"""
This file is used to control the AgileX Limos in Rastic to follow a series of Waypoints.
It uses the ROS setup from LIMO_ROS_SETUP.py and the linear quadratic regulator from LIMO_LQR.py.
To use this, create a tracker object and pass the name of the Limo you are using,
then pass a numpy array containing the sequence of waypoints to the trackTrajectory function in the following format:
waypoints = np.array([[x1,x2],[y1,y2],[theta1,theta2],[v1,v2]])
for waypoints waypoint1 = np.array([[x1],[y1],[theta1],[v1]]), waypoint2 = np.array([[x2],[y2],[theta2],[v2]]),
You can find an example of this in the "if __name__ == '__main__':" part of the file.
For best results, try to ensure that trajectories were generated based on the Limo's dynamic model.
This code was adapted by Carter Berlind with significant changes.
Original code received from Sienna Chien, original authors not listed.

Description: Waypoint tracking for limos in Rastic
"""
import matplotlib.pyplot as plt
import numpy as np
import math
import time
# from geometry_msgs.msg import PoseStamped
# from ackermann_msgs.msg import AckermannDrive
# import rospy
# from LIMO_ROS_SETUP import LIMO
import LIMO_LQR_1
import LIMO_PID_1
from nonLinModel import *

class Tracker:
    def __init__(
            self,
            limo_name:str
            ) -> None:
        """
        Tracker object used to follow a sequence of waypoints in a trajectory.
        Using this sends all the infrastructure necessary to receive pose
        information form the motion capture and send controls to the robot to track a trajectory

        :param limo_name: ROS node and topic name for the Limo you are using.
            This should match the name on the motion capture software.
            -> str
        """

        # Create ROS node to communicate with the limo robot
        # self.LIMO1 = LIMO(limo_name)

        # Frequency at which commands are sent to limo in Hz
        self.transmissionRate = 10
        self.dt = 1/self.transmissionRate
        # self.rate = rospy.Rate(self.transmissionRate)

        # Define agent state
        self.speed = 0
        self.steering_angle = 0
        self.speed2 = 0
        self.steering_angle2 = 0
        # self.x=np.array([
        #             [self.LIMO1.position_z],
        #             [self.LIMO1.position_x],
        #             [self.LIMO1.position_yaw],
        #             [self.speed]
        #         ])
        self.x=np.array([
                    [0.],
                    [0.],
                    [0.],
                    [0.]
                ])
        self.x_simp =np.array([
                    [0.],
                    [0.],
                    [0.],
                    [0.]
                ])

        # Create LQR controller object
        self.lqr = LIMO_LQR_1.LQR(dt=self.dt)
        self.pid = LIMO_PID_1.PID(dt=self.dt)
        self.theta_dot = 0
        # Get initial linearization
        [self.A,self.B] = self.lqr.getAB(self.x[2,0])
        [self.A2,self.B2] = self.lqr.getAB(self.x_simp[2,0])

    def trackTrajectoryLQR(
            self,
            trajectory:np.ndarray,
            stab_time:int = 27,
            relin_steps:int = 1
            ) -> None:
        """
        This function receives a trajectory of states and, using the motion capture
        to localize, sends control commands to the limo to track the trajectory.
        Controls are found using a linear quadratic regulator.

        :param trajectory: array of N waypoints that you want the robot to follow
            -> 4xN NumPy array
        :param stab_time: approximate number of time steps it takes to rach each point
            -> int
        :param relin_steps: number of time steps between each relinearization
            -> int
        """
        self.x_simp = self.x
        plot_x = []
        plot_y = []
        plot_x2 = []
        plot_y2 = []

        # plt.ion()
        plt.plot(trajectory[0],trajectory[1],'-g')
        # iterate through sequence of waypoints
        for i in range(0,trajectory.shape[1],4):
            #isolate current waypoint
            xd = trajectory[:,i:i+1]

            # Approach the next waypoint for stab_time time steps
            for count in range(stab_time,1,-1):

                # Update position and speed from motion capture
                # The speed is the previous velocity command
                # x=np.array([
                #     [self.LIMO1.position_z],
                #     [self.LIMO1.position_x],
                #     [self.LIMO1.position_yaw],
                #     [self.speed]
                # ])
        
                # print(x,xd)
                # Relinearize and find new gain matrix if relin_steps time steps have passed
                if count%relin_steps == 0:
                    [self.A,self.B] = self.lqr.getAB(self.x[2,0])
                    [self.A2,self.B2] = self.lqr.getAB(self.x_simp[2,0])
                    K = self.lqr.getK(count,self.A,self.B)
                    K2= self.lqr.getK(count,self.A2,self.B2)


                # Find the optimal control
                # The control is the change in angular and linear velocities repectively

                u1 = self.lqr.getControl(self.x,xd,K)
                u2 = self.lqr.getControl(self.x_simp,xd,K2)

                #Find change in steering steering angle based on desire
                ang = math.atan2((u1[0,0])*self.lqr.L/2,u1[1,0])
                
                # Update speed and steering angle
                self.speed = u1[1,0]
                self.speed2 = u2[1,0]

                #Ensure that inputs are within acceptable range
                #This is an added redundancy to ensure the viability of control inputs

                self.steering_angle = 0.7*np.clip(ang,-1.,1.)
            
                self.speed = np.clip(self.speed,-1.,1.)
                self.speed2 = np.clip(self.speed2,-1.,1.)

                
                self.x = nonlinearModelStep(self.x,np.array([[self.steering_angle],[self.speed]]),self.dt)
                self.x_simp = unicycleModelStep(self.x_simp,u1,self.dt)

                plot_x.append(self.x[0,0])
                plot_y.append(self.x[1,0])
                plot_x2.append(self.x_simp[0,0])
                plot_y2.append(self.x_simp[1,0])
                plt.plot(plot_x,plot_y,'-r')
                dy = xd[3,0]*0.2*math.sin(xd[2,0])
                dx = xd[3,0]*0.2*math.cos(xd[2,0])
                dy1 = 0.2*math.sin(self.x[2,0])
                dx1 = 0.2*math.cos(self.x[2,0])
                dy2 = 0.2*math.sin(self.x_simp[2,0])
                dx2 = 0.2*math.cos(self.x_simp[2,0])
                plt.arrow(xd[0,0],xd[1,0],dx,dy,head_width =0.03)
                # plt.arrow(self.x[0,0],self.x[1,0],dx1,dy1,color='red')
                # plt.arrow(self.x_simp[0,0],self.x_simp[1,0],dx2,dy2,color='blue')
                # plt.plot(plot_x2,plot_y2,'-b')
                plt.draw()
                plt.pause(0.1)
                # drive_msg_LIMO1 = self.LIMO1.control(self.speed,self.steering_angle)
                # self.LIMO1.pub.publish(drive_msg_LIMO1)
                # time.sleep(self.dt)
                # rospy.spin()

        # After following the trajectory, stop
        # drive_msg_LIMO1 = self.LIMO1.control(0,0)
        # self.LIMO1.pub.publish(drive_msg_LIMO1)
        plt.ioff()

        plt.plot(plot_x,plot_y,'-r')
        plt.plot(trajectory[0],trajectory[1],'-g')
        plt.show()
    def trackTrajectoryPID(
            self,
            trajectory:np.ndarray,
            ex,
            stab_time:int = 27,
            relin_steps:int = 1,
            fig = None,
            ax = None,
            
            ) -> None:
        """
        This function receives a trajectory of states and, using the motion capture
        to localize, sends control commands to the limo to track the trajectory.
        Controls are found using a linear quadratic regulator.

        :param trajectory: array of N waypoints that you want the robot to follow
            -> 4xN NumPy array
        :param stab_time: approximate number of time steps it takes to rach each point
            -> int
        :param relin_steps: number of time steps between each relinearization
            -> int
        """
        self.x_simp = self.x
        plot_x = []
        plot_y = []
        plot_x2 = []
        plot_y2 = []

        # plt.ion()
        # plt.plot(trajectory[0],trajectory[1],'-g')
        fig = plt.figure(1)
        # iterate through sequence of waypoints
        for i in range(0,trajectory.shape[1],4):
            #isolate current waypoint
            xd = trajectory[:,i:i+1]
            unit_theta = np.array([
                [math.cos(self.x[2,0])],
                [math.sin(self.x[2,0])]
            ])
            ref = xd[:2]-self.x[:2]
            e_steer_prev = np.cross(unit_theta.T,ref.T)[0]
            e_steer_int = 0
            e_vel_prev= np.dot(unit_theta.T,ref)[0,0]
            e_vel_int = 0

            # Approach the next waypoint for stab_time time steps
            for count in range(stab_time,1,-1):


                #Find change in steering steering angle based on desire
                u_steer,e_steer_prev,e_steer_int = self.pid.steerControl(self.x,xd,e_steer_prev,e_steer_int)
                u_vel,e_vel_prev,e_vel_int = self.pid.speedControl(self.x,xd,e_vel_prev,e_vel_int)

                self.x = nonlinearModelStep(self.x,np.array([[u_steer],[u_vel]]),self.dt)
                # self.x_simp = unicycleModelStep(self.x_simp,u1,self.dt)

                plot_x.append(self.x[0,0]/5.)
                plot_y.append(self.x[1,0]/5.)
                ex.agent()._sensor._p = self.x[:2]/5.
                ex.agent().plotSensorQuality()
                # plot_x2.append(self.x_simp[0,0])
                # plot_y2.append(self.x_simp[1,0])pl
                plt.plot(plot_x,plot_y,'-r')
                dy = xd[3,0]*0.2*math.sin(xd[2,0])
                dx = xd[3,0]*0.2*math.cos(xd[2,0])
                dy1 = 0.2*math.sin(self.x[2,0])
                dx1 = 0.2*math.cos(self.x[2,0])
                # dy2 = 0.2*math.sin(self.x_simp[2,0])
                # dx2 = 0.2*math.cos(self.x_simp[2,0])
                # plt.arrow(xd[0,0],xd[1,0],dx,dy,head_width =0.03)
                # plt.arrow(self.x[0,0],self.x[1,0],dx1,dy1,color='red')
                # plt.arrow(self.x_simp[0,0],self.x_simp[1,0],dx2,dy2,color='blue')
                # plt.plot(plot_x2,plot_y2,'-b')
                fig.canvas.draw()
                fig.canvas.flush_events()
                # drive_msg_LIMO1 = self.LIMO1.control(self.speed,self.steering_angle)
                # self.LIMO1.pub.publish(drive_msg_LIMO1)
                # time.sleep(self.dt)
                # rospy.spin()

        # After following the trajectory, stop
        # drive_msg_LIMO1 = self.LIMO1.control(0,0)
        # self.LIMO1.pub.publish(drive_msg_LIMO1)
        plt.ioff()

        plt.plot(plot_x,plot_y,'-r')
        plt.plot(trajectory[0],trajectory[1],'-g')
        plt.show()

if __name__ == '__main__':
    """
    Example of how to use trajectory tracking
    If this file is run on its own, this is the code that will run
    """

    # Create a Tracker object

    tracker = Tracker("limo770")
    
    # Define waypoints, ideally your planning algorithm will output waypoints
    waypoint1  = np.array([
        [.5],
        [1],
        [np.pi/4],
        [0]
    ])
    waypoint2 = np.array([
        [1.0],
        [2.0],
        [np.pi/2],
        [0]
    ])

    # Format waypoints to be as function expects
    waypoints = np.hstack((waypoint1,waypoint2))

    #Call the tracker to follow waypoints
    tracker.trackTrajectory(waypoints)
