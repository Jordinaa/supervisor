#!/usr/bin/env python3

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import rospy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import VFR_HUD
from sensor_msgs.msg import Imu
from helper_functions import quaternionToEuler

A0 = 0.05984281113 
CLA = 4.752798721
CLA_STALL = -3.85 
CDA = 0.6417112299
CDA_STALL = -0.9233984055
ALPHA_STALL = 0.3391428111
WINGAREA = 0.6
AIR_DENSITY = 1.2041
MASS = 1.5
G = 9.81

class Visualiser:
    '''
    The Visualizer class plots the rolly, pitch, yaw data live and overlays a static plot to compare real-time
    data vs. bounds
    '''
    def __init__(self):
        bounds = FlightEnvelopeAssessment(A0, CLA, CDA, ALPHA_STALL, WINGAREA, AIR_DENSITY, MASS, G, CLA_STALL, CDA_STALL)

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.velocity = 0.0
        self.vertical_acceleration = 0.0
        self.load_factor = 0.0

        self.time_list = []
        self.roll_list = []
        self.pitch_list = []
        self.yaw_list = []
        self.velocity_list = []
        self.velocity_euler_list = []
        self.load_factor_list = []
        self.load_factor_euler_list = []

        subPosition = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.position_cb)
        subvfr_hud = rospy.Subscriber('mavros/vfr_hud', VFR_HUD, self.velocity_cb)
        subAcceleration = rospy.Subscriber('mavros/imu/data', Imu, self.cl_callback)

        # live data
        self.fig, self.ax = plt.subplots()
        self.colors = ['black', 'red']
        self.labels = ['Load Factor', 'euler']
        # self.colors = ['black']
        # self.labels = ['Load Factor']
        self.lines = [self.ax.plot([], [], label=label, color=color, marker='o', linestyle='', markersize=3)[0] for label, color in zip(self.labels, self.colors)]

        # static data
        static_velocity, static_load_factor = bounds.calc_load_factor_vs_velocity_static()
        self.static_plot(static_velocity, static_load_factor)

    def plot_init(self):
        self.ax.set_xlim(left=0, right=25)
        self.ax.set_ylim([-5, 5])
        return self.lines

    def velocity_cb(self, msg):
        self.velocity = msg.airspeed
        self.velocity_list.append(self.velocity)

    def position_cb(self, msg):
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        self.roll, self.pitch, self.yaw = quaternionToEuler(qx, qy, qz, qw)
        self.roll_list.append(self.roll)
        self.pitch_list.append(self.pitch)
        self.yaw_list.append(self.yaw)

    def cl_callback(self, msg):
        az = msg.linear_acceleration.z
        self.vertical_acceleration = az

    def calc_load_factor(self, velocity, roll):
        lift_val = bounds.calc_lift(velocity, roll)
        # load_factor = (lift_val/(bounds.mass * bounds.g))
        load_factor = (lift_val/(bounds.mass * self.vertical_acceleration))
        print(f"actual Az: {self.vertical_acceleration}")
        return load_factor

    def calc_load_factor_euler(self, velocity, roll, acceleration):
        lift_val = bounds.calc_lift(velocity, roll)
        # load_factor = (lift_val/(bounds.mass * bounds.g))
        load_factor = (lift_val/(bounds.mass * acceleration))
        return load_factor

    def predict_eulers(self):
        step_size = .1
        start = 0+self.velocity_list[-1]
        end = 1 + step_size
        dvdt = (self.velocity_list[-1] - self.velocity_list[-2]) / step_size
        print(f"dvdt: {dvdt}")
        euler_list = np.arange(0, end, step_size)
        load_factor_euler_list = np.zeros(len(euler_list))
        load_factor_euler_list[0] = start
        for i in range(1, len(euler_list)):
            load_factor_euler_list[i] = load_factor_euler_list[i-1] + step_size * dvdt
            print(f"euler: {euler_list[i]} | load_factor: {load_factor_euler_list[i]}")
        
        return euler_list, load_factor_euler_list

    def update_plot(self, frame):
        thinned_velocity_list = self.velocity_list[-10:]
        thinned_pitch_angle_list = self.pitch_list[-10:]
        thinned_roll_angle_list = self.roll_list[-10:]
        thinned_load_factor_list = self.load_factor_list[-10:]
        # print(f"velocity: {self.velocity:.4} | pitch: {np.rad2deg(self.pitch):.4} | roll: {np.rad2deg(self.roll):.4}")

        for vel, roll in zip(thinned_velocity_list, thinned_roll_angle_list):
            load_factor = self.calc_load_factor(vel, roll)
            thinned_load_factor_list.append(load_factor)
            print(f"actual velocity: {vel:.4} | actual roll: {roll:.4} | actual n: {load_factor:.4}")

        euler_vel, euler_load_factor = self.predict_eulers()

        self.lines[0].set_data(thinned_velocity_list, thinned_load_factor_list)
        self.lines[1].set_data(euler_load_factor, euler_vel)
        return self.lines
 
    def static_plot(self, static_velocity, static_load_factor):
        self.bottom_static = [-x for x in static_load_factor]
        self.ax.plot(static_velocity, static_load_factor, color='red', alpha=0.5, label='Limit of N')
        self.ax.plot(static_velocity, self.bottom_static, color='red', alpha=0.5, label='Limit of -N')
        self.ax.axhline(y=1, color='green', linestyle='--')
        self.ax.axhline(y=-1, color='green', linestyle='--')
        ticks = len(static_velocity)
        self.ax.set_xticks(range(0, ticks, 5))
        self.ax.set_yticks(range(-ticks, ticks, 1))
        self.ax.grid(visible=True)
        self.ax.legend()


class FlightEnvelopeAssessment():
    '''
        The Flight Envelope Assessment class takes in a models data and it will define the bounds/limits
        of the flight envelope which will then be fed into the supervisor which will set the bounds 
        of the aircraft
    '''
    def __init__(self, A0, CLA, CDA, ALPHA_STALL, WINGAREA, AIR_DENSITY, MASS, G, CLA_STALL, CDA_STALL):
        self.alpha0 = A0
        self.cla = CLA
        self.cla_stall = CLA_STALL
        self.cda = CDA
        self.cda_stall = CDA_STALL
        self.alpha_stall = ALPHA_STALL
        self.area = WINGAREA
        self.rho = AIR_DENSITY
        self.mass = MASS
        self.g = G

        # calculated
        self.coefficient_lift = 0.0
        self.dynamic_pressure = 0.0
        self.lift = 0.0
        self.velocity = 0.0
        self.load_factor = 0.0
        self.vStall = 0.0
        self.clMaxWeights = [.2, .4, .6, .8]

        self.coefficient_lift_list = []
        self.angleList = np.arange(0, ALPHA_STALL, 0.01 * np.pi/180)
        for angle in self.angleList:
            self.coefficient_lift = self.calc_cl(angle)
            self.coefficient_lift_list.append(self.coefficient_lift)

        self.clMax = max(self.coefficient_lift_list)
        self.calc_v_stall()
        self.angleListDegrees = np.rad2deg(self.angleList)

    def calc_cl(self, angle_of_attack):
        clift = self.cla * (angle_of_attack - self.alpha0)
        return clift

    def calc_lift(self, pitch_angle, velocity):
        # not dividing by 2 here took it out 
        lift = (self.calc_cl(pitch_angle) * self.rho * velocity**2 * self.area)
        return lift

    def calc_v_stall(self):
        self.vStall = np.sqrt((2 * self.mass * self.g) / (self.rho * self.area * self.clMax))

    def calc_load_factor_vs_velocity_static(self):
        velocities = np.linspace(0, 100, 250)
        calc_load_factor_list = []
        for v in velocities:
            self.dynamic_pressure = 0.5 * self.rho * v ** 2
            self.lift = self.clMax * self.area * self.dynamic_pressure
            self.load_factor = self.lift / (self.mass * self.g)
            calc_load_factor_list.append(self.load_factor)
        return velocities, calc_load_factor_list


if __name__ == "__main__":

    rospy.init_node('flight_envelope_assessment_node')
    plt.close("all")
    rate = rospy.Rate(20)
    time_zero = rospy.get_time()
    bounds = FlightEnvelopeAssessment(A0, CLA, CDA, ALPHA_STALL, WINGAREA, AIR_DENSITY, MASS, G, CLA_STALL, CDA_STALL)
    vis = Visualiser()

    try:
        while not rospy.is_shutdown():
            ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init, frames=1, blit=False)
            plt.show(block=True)
            rate.sleep()
        plt.close('all')
    except KeyboardInterrupt:
        print("Exiting Flight Envelope Assessment")

