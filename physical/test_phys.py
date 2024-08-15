"""
This file is used to generate trajectories.
"""
import matplotlib.pyplot as plt
from hytoperm import *
from pprint import pprint
import pickle
from RUN_LIMO_1 import *
import json
import os


def expandSwitchingSegment(ptraj,utraj,num_points):
    #this method is meant for the switching segments. The original code has saves only the 
    #first and last points contained in teach segment, this expands it to a tunamble number of points
    #using linear interpolation. here ptraj is the points and utraj are the controls
    p1 = ptraj[:,:1]
    p2 = ptraj[:,-1:]
    u = utraj[:,-1:]
    multipliers = np.linspace(0,1,num_points)
    out_ptraj = p1
    out_utraj = u
    for num in multipliers[1:]:
        point = num*p1+(1-num)*p2
        out_ptraj=np.hstack((out_ptraj,point))
        out_utraj=np.hstack((out_utraj,u))
    return out_ptraj, out_utraj

def getDynamics(world,traj):
    # If you want to account for or find the hybrid dynamics of a region containing
    # segment, this returns them.

    #the first point in a segment may lie on a border and return the wrong dynamics,
    #choosing the second point helps ensure that we are in the correct regions
    p = traj[:,1:2]
    dynamics = world.getRegion(p).dynamics().v()
    return dynamics
    
def zeroRegions(world):
    #this eliminates the hybrid dynamics, if you want trajectories planned without them
    for region in world._regions:
        region.setV(np.zeros((2)))

class World:
    """
    This class will generate and save trajectories. I chose to place this in the class 
    to ease retrieving information in larger programs.
    """
    def __init__(self) -> None:
        #set up the running animation
        plt.ion()
        #minimum trial number, if the trial already exists this, we automatically iterate up so there is no reason to change this
        num = 1
        # generate experiment world, n_sets is the number of regions, and fraction is the percentage of regions containing targets
        ex = Experiment.generate(n_sets=15,fraction=0.2)
        #eliminate hybrid dynamics
        zeroRegions(ex.world())
        fig, ax = ex.plotWorld()
        ex.agent().plotSensorQuality()
        #use the rbbt to find a visiting sequence
        ex.agent().computeVisitingSequence()
        #optimize the monitoring segments
        ex.agent().optimizeCycle()

        #create a new directory to store trajectories
        fin = False
        while not fin:
            if os.path.isdir(f'trial{num}'):
                num += 1
            else:
                os.mkdir(f'trial{num}')
                fin = True

        count = 0
        # In this we store the points and controls in the cycle as a whole.
        # Storing both the points found as a whole and from stitching together individual segments gives additional trouble shooting options
        with open(f'trial{num}/cycleInfo{num}_total_points.json', "w") as final:
            json.dump(ex.agent()._cycle.pTrajectory.x.tolist(), final)
        with open(f'trial{num}/cycleInfo{num}_total_cntrls.json', "w") as final:
            json.dump(ex.agent()._cycle.uTrajectory.x.tolist(), final)
        #store trajectory information segment by segment        
        for ts in ex.agent()._cycle._trajectorySegments:
            ptraj = ts.pTrajectory.x
            utraj = ts.uTrajectory.x
            v = getDynamics(ex.world(),ptraj)
            with open(f'trial{num}/cycleInfo{num}_{count}_points.json', "w") as final:
                json.dump(ptraj.tolist(), final)
            with open(f'trial{num}/cycleInfo{num}_{count}_cntrls.json', "w") as final:
                json.dump(utraj.tolist(), final)
            with open(f'trial{num}/cycleInfo{num}_{count}_dynams.json', "w") as final:
                json.dump(v.tolist(), final)
            count += 1
        # plot the optimal cycle
        plt.ioff()
        fig, ax = ex.plotWorld()
        ex.agent().plotSensorQuality()
        ex.agent().plotCycle()
        #save the final trajectory figure
        pickle.dump(fig,open(f'trial{num}/Worldplot{num}.pickle','wb'))
        plt.show()

#The following functions are documented in the run_cycle_sim.py file. For more information check there.
def angleCorrection(thetas):
    out = np.zeros(thetas.shape)
    out[0,0] = thetas[0,0]
    for i in range(thetas.shape[1]-1):
        signal = 0
        out[0,i+1] = thetas[0,i+1]
        while signal != 1:
            if abs(out[0,i+1]-2*np.pi-out[0,i]) < abs(out[0,i+1]-out[0,i]):
                out[0,i+1] -= 2*np.pi
            elif abs(out[0,i+1]+2*np.pi-out[0,i]) < abs(out[0,i+1]-out[0,i]):
                out[0,i+1] += 2*np.pi
            else:
                signal = 1
    return out
    
def getThetas(points):
    thetas = np.zeros((1,points.shape[1]))
    for i in range(points.shape[1]):
        if i==0:
            thetas[0,i] = math.atan2(points[1,1]-points[1,0],points[0,1]-points[0,0])
        elif i != points.shape[1]-1:
            thetas[0,i] = math.atan2(points[1,i+1]-points[1,i-1],points[0,i+1]-points[0,i-1]) 
        else:
            thetas[0,i] = math.atan2(points[1,i]-points[1,i-1],points[0,i]-points[0,i-1])
    return thetas
def getVels(controls, hybrid_dynamics):
    vels = np.zeros((1,controls.shape[1]))
    for i in range(vels.shape[1]):
        vel = controls[:,i:i+1] + hybrid_dynamics
        vels[0,i] = np.linalg.norm(vel)
    return vels
def loadFig(num):
    fig = pickle.load(open(f'trial{num}/Worldplot{num}.pickle','rb'))
    return fig
def loadPoints(num,tot):
    points_dict = {}
    uts_dict = {}
    hds_dict = {}
    vels_dict = {}
    for count in range(tot):
        f=open(f'trial{num}/cycleInfo{num}_{count}_points.json','r')
        points_dict[count] = json.loads(f.readline())
        f=open(f'trial{num}/cycleInfo{num}_{count}_cntrls.json','r')
        uts_dict[count] = json.loads(f.readline())
        f=open(f'trial{num}/cycleInfo{num}_{count}_dynams.json','r')
        hds_dict[count] = np.array(json.loads(f.readline()))
        hds_dict[count].reshape((2,1))
        vels_dict[count] = getVels(np.array(uts_dict[count]),hds_dict[count])

    pts = np.array(points_dict[0])
    vels = np.array(vels_dict[0])
    for count in range(tot-1):
        pts = np.hstack((pts,np.array(points_dict[count+1])))
        vels = np.hstack((vels,np.array(vels_dict[count+1])))
    thetas = getThetas(pts)
    thetas = angleCorrection(thetas)
    points = np.vstack((5*pts,thetas))
    if points.shape[1] != vels.shape[1]:
        min_shape = min(points.shape[1],vels.shape[1])
        points = points[:,:min_shape]
        vels = vels[:,:min_shape]
    points = np.vstack((points,5*vels))

    hd = np.array([[0.0],[0.0]])
    f = open(f'trial{num}/cycleInfo{num}_total_points.json','r')
    pts_2 = np.array(json.loads(f.readline()))
    f = open(f'trial{num}/cycleInfo{num}_total_cntrls.json','r')
    uts_2 = np.array(json.loads(f.readline()))
    thetas_2 = getThetas(pts_2)
    thetas_2 = angleCorrection(thetas_2)
    vels_2 = getVels(uts_2,hd)
    points_2 = np.vstack((5*pts_2,thetas_2))
    if points_2.shape[1] != vels_2.shape[1]:
        min_shape = min(points_2.shape[1],vels_2.shape[1])
        points_2 = points_2[:,:min_shape]
        vels_2 = vels_2[:,:min_shape]
    points_2 = np.vstack((points_2,5*vels_2))
    
    return points,points_2
if __name__ == "__main__":
    wrld = World()
    exit()
