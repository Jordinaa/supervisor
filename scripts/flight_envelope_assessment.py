#!/usr/bin/env python3

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import VFR_HUD
from sensor_msgs.msg import Imu
from mavros_msgs.msg import AttitudeTarget
from geometry_msgs.msg import Vector3

import config
from helper_functions import quaternionToEuler

class Visualiser:
    '''
    The Visualizer class plots the rolly, pitch, yaw data live and overlays a static plot to compare real-time
    data vs. bounds
    '''
    def __init__(self):
        bounds = FlightEnvelopeAssessment(config.A0, config.CLA, config.CDA, config.ALPHA_STALL, config.WINGAREA, config.AIR_DENSITY, config.MASS, config.G, config.CLA_STALL, config.CDA_STALL)
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
        self.roll_rate_list = []
        self.pitch_rate_list = []
        self.yaw_rate_list = []
        
        self.velocity_list = []
        self.load_factor_list = []

        self.load_factor_list2 = []
        self.load_factor_list3 = []
        self.load_factor_list4 = []

        self.load_factor_euler_list = []
        self.velocity_euler_list = []

        subPosition = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.position_cb)
        subvfr_hud = rospy.Subscriber('mavros/vfr_hud', VFR_HUD, self.velocity_cb)
        subAcceleration = rospy.Subscriber('mavros/imu/data', Imu, self.az_callback)
        subRates = rospy.Subscriber('mavros/imu/data', Imu, self.rates_cb)

        self.fig, self.ax = plt.subplots()
        self.colors = ['black', 'red', 'green', 'purple', 'orange']
        self.labels = ['n1', 'n2', 'n3', 'n4', 'n5']

        self.lines = [self.ax.plot([], [], label=label, color=color, marker='o', linestyle='', markersize=6)[0] for label, color in zip(self.labels, self.colors)]
    
        static_velocity, static_load_factors = bounds.calc_load_factor_vs_velocity_static()
        self.static_plot(static_velocity, static_load_factors)



    def plot_init(self):
        self.ax.set_xlim(left=0, right=25)
        self.ax.set_ylim([-5, 10])
        return self.lines

    def velocity_cb(self, msg):
        self.velocity = msg.groundspeed
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

    def rates_cb(self, msg):
        self.roll_rate = msg.angular_velocity.x
        self.pitch_rate = msg.angular_velocity.y
        self.yaw_rate = msg.angular_velocity.z
        self.roll_rate_list.append(self.roll_rate)
        self.pitch_rate_list.append(self.pitch_rate)
        self.yaw_rate_list.append(self.yaw_rate)
        # print(f"roll_rate: {self.roll_rate:.4} | pitch_rate: {self.pitch_rate:.4} | yaw_rate: {self.yaw_rate:.4}")

    def az_callback(self, msg):
        az = msg.linear_acceleration.z
        self.vertical_acceleration = az

    def calc_load_factor1(self, velocity, pitch):
        lift_val = bounds.calc_lift(velocity, pitch)
        load_factor = (lift_val/(bounds.mass * bounds.g))
        print(f"Load Factor1: {load_factor:.4}")
        return load_factor

    def calc_load_factor2(self, velocity, pitch):
        lift_val = bounds.calc_lift(velocity, pitch)
        load_factor = (lift_val/(self.vertical_acceleration))
        print(f"Load Factor2: {load_factor:.4}")
        return load_factor

    def calc_load_factor3(self, roll, pitch):
        n_total = (np.cos(roll) / np.cos(pitch))
        print(f"Load Factor3: {n_total:.4}")
        return n_total

    def calc_load_factor4(self, roll, pitch):
        n_bank = 1/np.cos(roll)
        n_pitch = 1/np.cos(pitch)
        n_total = np.sqrt(n_bank**2 + n_pitch**2)
        print(f"Load factor4: {n_total}")
        return n_total
    
    # i think this one works the others i have no clue. i do not have the intuition for this yet
    def calc_load_factor5(self, roll):
        n_bank = 1/np.cos(roll)
        print(f"Load factor5: {n_bank}")
        return n_bank

    def calc_load_factor_euler(self, velocity, roll, acceleration):
        lift_val = bounds.calc_lift(velocity, roll)
        # load_factor = (lift_val/(bounds.mass * bounds.g))
        load_factor = (lift_val/(bounds.mass * acceleration))
        return load_factor

    def predict_eulers(self):
        step_size = .1
        start = 0 + self.velocity_list[-1]
        end = 1 + step_size
        dvdt = (self.velocity_list[-1] - self.velocity_list[-2]) / step_size
        print(f"dvdt: {dvdt}")
        euler_list = np.arange(0, end, step_size)
        load_factor_euler_list = np.zeros(len(euler_list))
        load_factor_euler_list[0] = start
        for i in range(1, len(euler_list)):
            load_factor_euler_list[i] = load_factor_euler_list[i-1] + step_size * dvdt
            print(f"euler: {euler_list[i]:.4} | load_factor: {load_factor_euler_list[i]:.4}")
        
        return euler_list, load_factor_euler_list

    def update_plot(self, frame):
        self.thinned_velocity_list = self.velocity_list[-10:]
        self.thinned_roll_angle_list = self.roll_list[-10:]
        self.thinned_pitch_angle_list = self.pitch_list[-10:]

        self.thinned_load_factor_list = self.load_factor_list[-10:]
        self.thinned_load_factor_list2 = self.load_factor_list2[-10:]
        self.thinned_load_factor_list3 = self.load_factor_list3[-10:]
        self.thinned_load_factor_list4 = self.load_factor_list4[-10:]

        # for vel, roll in zip(self.thinned_velocity_list, self.thinned_pitch_angle_list):
        #     load_factor = self.calc_load_factor(vel, roll)
        #     self.thinned_load_factor_list.append(load_factor)
        #     # print(f"velocity1: {vel:.4} | roll1: {roll:.4} | n1: {load_factor:.4}")
        #     print(f"load_factor1: {load_factor:.4}")

        # for vel2, roll2, pitch2 in zip(self.thinned_velocity_list, self.thinned_roll_angle_list, self.thinned_pitch_angle_list):
        #     load_factor2 = self.calc_load_factor2(vel2, roll2, pitch2)
        #     self.thinned_load_factor_list2.append(load_factor2)
        #     # print(f"velocity2: {vel2:.4} | roll2: {roll2:.4} | pitch2: {pitch2} | load factor2: {load_factor2:.4}")
        #     print(f"load_factor2: {load_factor2:.4}")

        load_factor1 = self.calc_load_factor1(self.velocity_list[-1], self.thinned_pitch_angle_list[-1])
        load_factor2 = self.calc_load_factor2(self.velocity_list[-1], self.thinned_pitch_angle_list[-1])
        load_factor3 = self.calc_load_factor3(self.thinned_roll_angle_list[-1], self.thinned_pitch_angle_list[-1])
        load_factor4 = self.calc_load_factor4(self.thinned_roll_angle_list[-1], self.thinned_pitch_angle_list[-1])
        load_factor5 = self.calc_load_factor5(self.thinned_roll_angle_list[-1])

        self.lines[0].set_data(self.thinned_velocity_list[-1], load_factor1)
        self.lines[1].set_data(self.thinned_velocity_list[-1], load_factor2)
        self.lines[2].set_data(self.thinned_velocity_list[-1], load_factor3)
        self.lines[3].set_data(self.thinned_velocity_list[-1], load_factor4)
        self.lines[4].set_data(self.thinned_velocity_list[-1], load_factor5)

        # euler_vel, euler_load_factor = self.predict_eulers()
        # self.lines[0].set_data(self.thinned_velocity_list, self.thinned_load_factor_list)
        # self.lines[1].set_data(self.thinned_velocity_list, self.thinned_load_factor_list2)

        return self.lines
 
    def static_plot(self, static_velocity, static_load_factors):        
        line_styles = ['-', '-.', '--', '--', ':']  # Define different line styles for each weight
        line_labels = ['clMax', 'Notify', 'Alert', 'Caution', 'Warning', 'uh']  # Define the labels for each line
        line_colors = ['red', 'orange', 'yellow', 'green', 'blue', 'purple']  # Define the colors for each line
        for load_factor, line_style, label, line_color in zip(static_load_factors, line_styles, line_labels, line_colors):
            self.ax.plot(static_velocity, load_factor, color=line_color, alpha=0.3, linestyle=line_style, label=label)

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

        self.coefficient_lift_list = []
        self.angleList = np.arange(0, ALPHA_STALL, 0.01 * np.pi/180)

        for angle in self.angleList:
            self.coefficient_lift = self.calc_cl(angle)
            self.coefficient_lift_list.append(self.coefficient_lift)

        self.clMax = max(self.coefficient_lift_list)
        self.clMaxWeights = [.5, .6, .7, .8,]

        self.calc_v_stall()
        self.angleListDegrees = np.rad2deg(self.angleList)

    def calc_cl(self, angle_of_attack):
        clift = self.cla * (angle_of_attack - self.alpha0)
        # print(f"Coeff. Lift: {clift}")
        return clift

    def calc_lift(self, velocity, AoA):
        lift = (self.calc_cl(AoA) * (self.rho * (velocity**2) * .5) * self.area)
        return lift
    
    def calc_v_stall(self):
        self.vStall = np.sqrt((2 * self.mass * self.g) / (self.rho * self.area * self.clMax))

    def calc_load_factor_vs_velocity_static(self):
        velocities = np.linspace(0, 100, 250)
        calc_load_factor_lists = []

        # Calculate load factors for CLmax without any weight adjustments
        calc_load_factor_list = []
        for v in velocities:
            self.dynamic_pressure = 0.5 * self.rho * v ** 2
            self.lift = self.clMax * self.area * self.dynamic_pressure
            self.load_factor = self.lift / (self.mass * self.g)
            calc_load_factor_list.append(self.load_factor)
        calc_load_factor_lists.append(calc_load_factor_list)  # Add CLmax list to the list of load factor lists

        # Calculate load factors for CLmax with weight adjustments
        for weight in self.clMaxWeights:
            calc_load_factor_list = []
            for v in velocities:
                self.dynamic_pressure = 0.5 * self.rho * v ** 2
                self.lift = (self.clMax * weight) * self.area * self.dynamic_pressure
                self.load_factor = self.lift / (self.mass * self.g)
                calc_load_factor_list.append(self.load_factor)
            calc_load_factor_lists.append(calc_load_factor_list)  # Add weighted CLmax list to the list of load factor lists

        return velocities, calc_load_factor_lists



if __name__ == "__main__":

    rospy.init_node('flight_envelope_assessment_node')
    plt.close("all")
    rate = rospy.Rate(20)
    time_zero = rospy.get_time()
    bounds = FlightEnvelopeAssessment(config.A0, config.CLA, config.CDA, config.ALPHA_STALL, config.WINGAREA, config.AIR_DENSITY, config.MASS, config.G, config.CLA_STALL, config.CDA_STALL)
    vis = Visualiser()

    try:
        while not rospy.is_shutdown():
            ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init, frames=1, blit=False)
            plt.show(block=True)
            rate.sleep()
        plt.close('all')
    except KeyboardInterrupt:
        print("Exiting Flight Envelope Assessment")

