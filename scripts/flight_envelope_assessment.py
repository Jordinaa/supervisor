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
        
        self.fig, self.ax = plt.subplots()
        self.colors = ['orange','blue','red']
        self.labels = ['roll','pitch','yaw']
        self.lines = [self.ax.plot([], [], label=label, color=color)[0] for label, color in zip(self.labels, self.colors)]
        # equivalent code but same thing 
        # self.lines = []
        # for label, color in zip(self.labels, self.colors):
        #     line_object = self.ax.plot([], [], label=label, color=color)[0]
        #     self.lines.append(line_object)

        self.time = []
        self.roll_list = []
        self.pitch_list = []
        self.yaw_list = []

        self.repeat_length = 100
        subPosition = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.position_cb)
    
        # static data 
        self.static_data = [30, 60, 90, 120, 150]
        self.static_x = list(range(len(self.static_data)))
        self.static_line, = self.ax.plot(self.static_x, self.static_data, color='green', label='static_data')
        self.static_x_updated = self.static_x

        self.ax.legend()

    def plot_init(self):
        self.ax.set_xlim(left=0, right=self.repeat_length)
        self.ax.set_ylim([-180, 180])
        return self.lines

    def position_cb(self, msg):
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        self.roll, self.pitch, self.yaw = quaternionToEuler(qx, qy, qz, qw)
        self.roll = np.rad2deg(self.roll)
        self.pitch = np.rad2deg(self.pitch)
        self.yaw = np.rad2deg(self.yaw)
        
        time_index = len(self.time)
        self.time.append(time_index+1)
        self.roll_list.append(self.roll)
        self.pitch_list.append(self.pitch)
        self.yaw_list.append(self.yaw)

        if time_index > self.repeat_length:
            self.ax.set_xlim(time_index-self.repeat_length, time_index)
        else:
            self.ax.set_xlim(0, self.repeat_length)

        # static data
        left_x, right_x = self.ax.get_xlim()
        self.static_x_updated = [x + left_x for x in self.static_x]
        self.static_line.set_data(self.static_x_updated, self.static_data)

        rospy.loginfo(f"{str(self.roll)} {str(self.pitch)} {str(self.yaw)}")

    def update_plot(self, frame):
        self.lines[0].set_data(self.time, self.roll_list)    
        self.lines[1].set_data(self.time, self.pitch_list)
        self.lines[2].set_data(self.time, self.yaw_list)

        return self.lines


class FlightEnvelopeAssessment():
    '''
    The Flight Envelope Assessment class takes in a models data and it will define the bounds/limits
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

    rospy.init_node('flight_envelope_assessment_node')
    plt.close("all")
    rate = rospy.Rate(20)
    time_zero = rospy.get_time()
    vis = Visualiser()

    try:
        while not rospy.is_shutdown():
            ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init, frames=10, blit=False)
            plt.show(block=True)
            rate.sleep()

    except KeyboardInterrupt:
        print("Exiting Flight Envelope Assessment")

    assessment = FlightEnvelopeAssessment(A0, CLA, CDA, ALPHA_STALL, WINGAREA, AIR_DENSITY, MASS, G)
    print(assessment.clMax)