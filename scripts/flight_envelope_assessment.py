#!/usr/bin/env python3

from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped


from helper_functions import quaternionToEuler, eulerToQuaternion
from data_logger import DataLogger


# xml parser will need to be updated and maybe take in user input vs reading only gazebo files
A0 = 0.05984281113
CLA = 4.752798721
CDA = 0.6417112299
ALPHA_STALL = 0.3391428111
WINGAREA = 0.6
AIR_DENSITY = 1.2041
MASS = 1.5
G = 9.81

# alpha_stallDeg = np.rad2deg(0.3391428111)
# cla_stall = -3.85
# cda_stall = -0.9233984055


class Visualiser:
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.true_roll = 0.0
        self.true_pitch = 0.0
        self.true_yaw = 0.0
        
        self.fig, (self.ax1,self.ax2) = plt.subplots(2,1)
        self.colors = ['orange','blue','red']
        
        self.labels = ['roll','pitch','yaw']
        self.labels2 = ['true_roll', 'true_pitch', 'true_pitch']
        
        self.lines = [self.ax1.plot([], [])[0] for _ in range(3)]
        self.lines2 = [self.ax2.plot([], [])[0] for _ in range(3)]
        
        for i, line in enumerate(self.lines):
            line._color = self.colors[i]
            line.set_label(self.labels[i])
        
        for i, line in enumerate(self.lines2):
            line._color = self.colors[i]
            line.set_label(self.labels2[i])
        
        self.time = []
        self.roll_list, self.true_roll_list = [], []
        self.pitch_list, self.true_pitch_list = [], []
        self.yaw_list, self.true_yaw_list = [], []
        
        self.repeat_length = 100
        
        sub1 = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.position_cb)

        self.ax1.legend(handles = self.lines)
        self.ax2.legend(handles = self.lines2)
    
    def plot_init(self):
        for ax in [self.ax1,self.ax2]:
            ax.set_xlim(left=0, right=self.repeat_length)
        return self.lines, self.lines2

    def position_cb(self, msg):
        time = rospy.get_time() - time_zero
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        self.roll, self.pitch, self.yaw = quaternionToEuler(qx, qy, qz, qw)
        
        time_index = len(self.time)
        self.time.append(time_index+1)

        self.roll_list.append(self.roll)
        self.pitch_list.append(self.pitch)
        self.yaw_list.append(self.yaw)
        
        self.true_roll_list.append(self.true_roll)
        self.true_pitch_list.append(self.true_pitch)
        self.true_yaw_list.append(self.true_yaw)

        for ax in [self.ax1,self.ax2]:
            if time_index > self.repeat_length:
                ax.set_xlim(time_index-self.repeat_length, time_index)
            else:
                ax.set_xlim(0, self.repeat_length)

    def update_plot(self, frame):
        self.lines[0].set_data(self.time, self.roll_list)
        self.lines[0].set_label(self.labels[0])        
        
        self.lines[1].set_data(self.time, self.pitch_list)
        self.lines[1].set_label(self.labels[1])
        
        self.lines[2].set_data(self.time, self.yaw_list)
        self.lines[2].set_label(self.labels[2])
            
        self.lines2[0].set_data(self.time, self.true_roll_list)
        self.lines2[0].set_label(self.labels2[0])
    
        self.lines2[1].set_data(self.time, self.true_pitch_list)
        self.lines2[1].set_label(self.labels2[1])
        
        self.lines2[2].set_data(self.time, self.true_yaw_list)
        self.lines2[2].set_label(self.labels2[2])
        
        return self.lines, self.lines2


class FlightEnvelopeAssessment():
    '''
    the flight envelopeassessment class takes in a models data and it will define the bounds/limits
    of the flight envelope which will then be fed into the supervisor which will set the bounds 
    of the aircraft
    '''
    def __init__(self, A0, CLA, CDA, ALPHA_STALL, WINGAREA, AIR_DENSITY, MASS, G):

        self.alpha0 = A0
        self.cla = CLA
        self.cda = CDA
        self.alpha_stall = ALPHA_STALL
        self.area = WINGAREA
        self.rho = AIR_DENSITY
        self.mass = MASS
        self.g = G

        self.coefficientLift = 0.0
        self.dynamicPressure = 0.0
        self.lift = 0.0
        self.velocity = 0.0
        self.loadFactor = 0.0
        self.stallSpeed = 0.0
        self.weight = 0.0

        self.vStall = 0.0
        self.clMaxWeights = [.2, .4, .6, .8]

        self.coefficientLiftList = []
        self.angleList = np.arange(0, ALPHA_STALL, 0.01 * np.pi/180)

        for angle in self.angleList:
            self.coefficientLift = self.calc_cl(angle)
            self.coefficientLiftList.append(self.coefficientLift)

        self.clMax = max(self.coefficientLiftList)
        self.angleListDegrees = np.rad2deg(self.angleList)

    def calc_cl(self, angle):
        return self.cla * (angle - self.alpha0)


if __name__ == "__main__":

    rospy.init_node('visualize_tracking')
    plt.close("all")
    rate = rospy.Rate(20)
    time_zero = rospy.get_time()
    vis = Visualiser()

    while not rospy.is_shutdown():
        ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init, 
                            frames=10,blit=False)
        plt.show(block=True)
        rate.sleep()

    assessment = FlightEnvelopeAssessment(A0, CLA, CDA, ALPHA_STALL, WINGAREA, AIR_DENSITY, MASS, G)
    print(assessment.clMax)