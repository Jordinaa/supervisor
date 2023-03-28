#!/usr/bin/env python3

import time
import csv
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import seaborn as sns
import numpy as np
from scipy import signal

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import VFR_HUD, State
from sensor_msgs.msg import Imu
from offboard_py.msg import FourFloats

import config
from helper_functions import quaternionToEuler
from data_logger import DataLogger

class Visualiser:
    '''
    The Visualizer class plots the rolly, pitch, yaw data live and overlays a static plot to compare real-time
    data vs. bounds
    '''
    def __init__(self):
        self.bounds = FlightEnvelopeAssessment(config.A0, config.CLA, config.CDA, config.ALPHA_STALL, config.WINGAREA, config.AIR_DENSITY, config.MASS, config.G, config.CLA_STALL, config.CDA_STALL)
        self.start_time = time.time()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.velocity = 0.0
        self.horizontal_acceleration = 0.0
        self.vertical_acceleration = 0.0
        self.load_factor = 0.0

        self.load_factor_predict = 0.0

        self.previous_filtered_load_factor = None
        self.weight = .1

        self.time_list = []
        self.roll_list = []
        self.pitch_list = []
        self.yaw_list = []
        self.roll_rate_list = []
        self.pitch_rate_list = []
        self.yaw_rate_list = []
        self.velocity_list = []
        self.vertical_acceleration_list = [0, 0, 0, 0]
        self.horizontal_acceleration_list = []
        self.cl_list = []
        self.load_factor_list = []

        self.velocity_prediction_list = []
        self.load_factor_prediction_list = []
        self.vertical_acceleration_list_prediction = [0, 0]
        self.horizontal_acceleration_list_prediction = []

        self.filteredAz_list = []
        self.filteredAz = 0.0

        self.csv_path = "/home/taranto/catkin_ws/src/offboard_py/data/drone_data.csv"
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['time', 'load_factor', 'load_factor_predict', 'accelerationZ'])

        subPosition = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.position_cb)
        subvfr_hud = rospy.Subscriber('mavros/vfr_hud', VFR_HUD, self.velocity_cb)
        subAcceleration = rospy.Subscriber('mavros/imu/data', Imu, self.acceleration_callback)
        subRates = rospy.Subscriber('mavros/imu/data', Imu, self.rates_cb)
        subState = rospy.Subscriber('mavros/state', State, callback=self.state_cb)
        SubVNData = rospy.Subscriber('vn_data_sub', FourFloats, callback=self.vn_data_callback)
        self.pub = rospy.Publisher('vn_data_pub', FourFloats, queue_size=10)

        sns.set_style('whitegrid')
        sns.set_palette('colorblind')
        self.fig, (self.ax) = plt.subplots()

        self.colors = ['blue', 'red']
        self.labels = ['$n_{t}$', '$n_{p}$']
        self.lines = [self.ax.plot([], [], label=label, color=color, marker='o', linestyle='', markersize=4)[0] for label, color in zip(self.labels, self.colors)]

        static_velocity, static_load_factors = bounds.calc_load_factor_vs_velocity_static()
        self.static_plot(static_velocity, static_load_factors)

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

    def acceleration_callback(self, msg):
        ax = msg.linear_acceleration.x
        self.horizontal_acceleration = ax
        self.horizontal_acceleration_list.append(ax)

        az = msg.linear_acceleration.z
        self.vertical_acceleration = az
        self.vertical_acceleration_list.append(az)

        self.time_list.append(time.time() - self.start_time)
    
    def state_cb(self, msg):
        self.current_state = msg

    def vn_data_callback(self, msg):
        self.m1 = msg.value1
        self.m2 = msg.value2
        self.m3 = msg.value3
        self.m4 = msg.value4

    def vn_data_publisher(self):
        start_time = time.time()
        four_floats = FourFloats()
        four_floats.value1 = self.time_list[-1] 
        four_floats.value2 = self.load_factor_list[-1]
        four_floats.value3 = self.load_factor_prediction_list[-1]
        # four_floats.value4 = self.vertical_acceleration_list[-1]
        four_floats.value4 = self.filteredAz
        # print(f'filteredAz: {four_floats.value4}')
        self.pub.publish(four_floats)

    def calc_load_factor(self):
        load_factor = self.vertical_acceleration / bounds.g
        self.load_factor_list.append(load_factor)
        self.load_factor = load_factor
        return load_factor

    def predict_load_factor(self):
        load_factor_predict = self.predict_next_load_factor(self.vertical_acceleration_list, self.time_list)
        self.load_factor_prediction_list.append(load_factor_predict)
        self.load_factor_predict = load_factor_predict
        return load_factor_predict

    def predict_next_load_factor(self, velocity_list, time_list):
        v_final = velocity_list[-1]
        v_initial = velocity_list[-2]
        dt = time_list[-1] - time_list[-2]
        delta_v = (v_final - v_initial) / dt
        prediction = delta_v / bounds.g
        return prediction

    def first_order_filter(self, raw_value, previous_filtered_value, weight):
        if previous_filtered_value is None:
            filtered_value = raw_value
        else:
            filtered_value = weight * raw_value + (1 - weight) * previous_filtered_value
        return filtered_value

    # butter worth difference equation
    # def butter_worth_filter(self, acceleration_list):
    def filter_signals(self, a, b):
        # print(f'{b[0]} | {self.vertical_acceleration_list[-1]} | {b[1]} | {self.vertical_acceleration_list[-2]} | {b[2]} | {self.vertical_acceleration_list[-3]} | {a[1]} | {self.vertical_acceleration_list_prediction[-1]} | {a[2]} | {self.vertical_acceleration_list_prediction[-2]}')
        # z = b[0] * self.vertical_acceleration_list[-1] + b[1] * self.vertical_acceleration_list[-2] + b[2] * self.vertical_acceleration_list[-3] - (a[1] * self.vertical_acceleration_list_prediction[-1] + a[2] * self.vertical_acceleration_list_prediction[-2])
        print(f'{b[0]} | {self.vertical_acceleration_list[-1]} | {b[1]} | {self.vertical_acceleration_list[-2]} | {a[1]} | {self.vertical_acceleration_list_prediction[-1]}')
        z = b[0] * self.vertical_acceleration_list[-1] + b[1] * self.vertical_acceleration_list[-2] - (a[1] * self.vertical_acceleration_list_prediction[-1])
        print(f'this is {z}')
        self.vertical_acceleration_list_prediction.append(z)
        print(f'this is verti(0) {self.vertical_acceleration_list_prediction[0]}')
        self.vertical_acceleration_list_prediction.pop(0)
        return z

    def butter_worth_filter(self):
        fs = 50
        cutoff = 2.5
        nyt = .5 * fs
        order = 1
        normalized_cutoff = cutoff / nyt
        # b = [0.204, 0.204]
        # a = [1, -0.591]
        b, a = signal.butter(order, normalized_cutoff, btype='low', analog=False)
        print(f'b: {b}')
        print(f'a: {a}')

        self.filteredAz = self.filter_signals(a, b)
        print(f'filteredAz inside that butter: {self.filteredAz}')
        # filtered_signal = signal.filtfilt(b, a, self.vertical_acceleration_list)
        # self.filteredAz_list.append(filtered_signal)
        # print(len(self.vertical_acceleration_list))
        # print(len(filtered_signal))
        # for i in range(0, len(filtered_signal)):
        #     self.filteredAz = filtered_signal[i]

        # return filtered_signal


    def plot_init_vn(self):
        self.ax.set_xlim(left=0, right=25)
        self.ax.set_ylim([-5, 10])
        self.ax.axhline(y=1, color='green', linestyle='--', alpha=0.2, linewidth=2)
        self.ax.axhline(y=-1, color='green', linestyle='--', alpha=0.2, linewidth=2)
        self.ax.set_xlabel('Velocity')
        # self.ax.set_xticks(range(0, int(ticks), 5))
        self.ax.set_ylabel('Load Factor')
        # self.ax.set_yticks(range(-ticks, ticks, 1))
        self.ax.set_title('Load Factor vs. Velocity')
        self.ax.grid(visible=True)
        self.ax.legend(fontsize='small')
        return self.lines


    def update_plot(self, frame):
        for lol in range(0, 10):
            load_factor = self.calc_load_factor()
            self.load_factor_list.append(load_factor)

            next_load_factor = self.predict_next_load_factor(self.velocity_list, self.time_list)
            
            filtered_load_factor = self.first_order_filter(next_load_factor, self.previous_filtered_load_factor, self.weight)
            self.previous_filtered_load_factor = filtered_load_factor
            
            self.load_factor_prediction_list.append(filtered_load_factor + self.load_factor)
            self.vn_data_publisher()

        self.butter_worth_filter()


        # self.butter_worth_filter(self.filteredAz)

        self.thinned_velocity_list = self.velocity_list[-10:]
        self.thinned_vertical_acceleration_list = self.vertical_acceleration_list[-10:]
        self.thinned_load_factor_list = self.load_factor_list[-10:]
        self.thinned_load_factor_prediction_list = self.load_factor_prediction_list[-10:]
        self.lines[0].set_data(self.thinned_velocity_list, self.thinned_load_factor_list)
        # self.lines[0].set_data(self.velocity_list[-10:], self.load_factor_list[-10:])
        self.lines[1].set_data(self.thinned_velocity_list[-10:], self.load_factor_prediction_list[-10:])

        print(f'true n:            {load_factor:.4}')
        print(f'predicted n:       {next_load_factor:.4}')
        print(f'filtered n:        {filtered_load_factor:.4}')
        print(f'filtered n + true: {filtered_load_factor + self.load_factor:.4}')
        return self.lines
 
    def static_plot(self, static_velocity, static_load_factors):        
        line_styles = ['-', '-', '-', '-', '-']  # Define different line styles for each weight
        line_labels = ['$Cl_{Max}$', '$Cl_{Max}$ ⋅ 0.9', '$Cl_{Max}$ ⋅ 0.8', '$Cl_{Max}$ ⋅ 0.7', '$Cl_{Max}$ ⋅ 0.6']  # Define the labels for each line   
        line_colors = [plt.cm.Reds(x) for x in range(256, 128, -(256-128)//(6-1))]
        for load_factor, line_style, label, line_color in zip(static_load_factors, line_styles, line_labels, line_colors):
            self.ax.plot(static_velocity, load_factor, color=line_color, linestyle=line_style, label=label, alpha=1, linewidth=2)

    # def __del__(self):
    #     self.csv_file.close()



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

        self.coefficient_lift = 0.0
        self.dynamic_pressure = 0.01
        self.lift = 0.0
        self.velocity = 0.0
        self.load_factor = 0.0
        self.vStall = 0.0

        self.coefficient_lift_list = []
        
        self.angleList = np.arange(0, ALPHA_STALL, 0.01 * np.pi/180)
        self.coefficient_lift_list = [self.calc_cl(angle) for angle in self.angleList]
        self.clMax = max(self.coefficient_lift_list)
        self.clMaxWeights = [.9, .8, .7, .6]
        self.vStall = self.calc_v_stall
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
        return self.vStall

    def calc_load_factor_vs_velocity_static(self):
        velocities = np.linspace(0, 100, 250)
        calc_load_factor_lists = []
        calc_load_factor_list = []

        for v in velocities:
            self.dynamic_pressure = 0.5 * self.rho * v ** 2
            self.lift = self.clMax * self.area * self.dynamic_pressure
            self.load_factor = self.lift / (self.mass * self.g)
            calc_load_factor_list.append(self.load_factor)
        calc_load_factor_lists.append(calc_load_factor_list)
        
        for weight in self.clMaxWeights:
            calc_load_factor_list = []
            for v in velocities:
                self.dynamic_pressure = 0.5 * self.rho * v ** 2
                self.lift = self.clMax * weight * self.area * self.dynamic_pressure
                self.load_factor = self.lift / (self.mass * self.g)
                calc_load_factor_list.append(self.load_factor)
            calc_load_factor_lists.append(calc_load_factor_list)

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
            ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init_vn, frames=1, blit=False)
            plt.show(block=True)
            rate.sleep()
        plt.close('all')
        # vis.__del__()

    except KeyboardInterrupt:
        print("Exiting Flight Envelope Assessment")

