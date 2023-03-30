#!/usr/bin/env python3

import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import seaborn as sns
import numpy as np
from scipy import signal

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import VFR_HUD, State
from sensor_msgs.msg import Imu
from supervisor.msg import DataLogger

import config
from helper import quaternionToEuler

class FlightEnvelopeAssessment():
    '''
        The Flight Envelope Assessment class takes in a models data and it will define the bounds/limits
        of the flight envelope which will then be fed into the supervisor which will set the bounds 
        of the aircraft
    '''
    def __init__(self):
        self.alpha0 = config.A0
        self.cla = config.CLA
        self.cla_stall = config.CLA_STALL
        self.cda = config.CDA
        self.cda_stall = config.CDA_STALL
        self.alpha_stall = config.ALPHA_STALL
        self.area = config.WINGAREA
        self.rho = config.AIR_DENSITY
        self.mass = config.MASS
        self.g = config.G

        self.coefficient_lift = 0.0
        self.dynamic_pressure = 0.01
        self.lift = 0.0
        self.velocity = 0.0
        self.load_factor = 0.0
        self.vStall = 0.0

        subState = rospy.Subscriber('mavros/state', State, callback=self.state_cb)

        self.coefficient_lift_list = []
        
        self.angleList = np.arange(0, self.alpha_stall, 0.01 * np.pi/180)
        self.coefficient_lift_list = [self.calc_cl(angle) for angle in self.angleList]
        self.clMax = max(self.coefficient_lift_list)
        self.clMaxWeights = [.9, .8, .7, .6]
        self.vStall = self.calc_v_stall
        self.angleListDegrees = np.rad2deg(self.angleList)

    def state_cb(self, msg):
        self.current_state = msg

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
        velocities = np.linspace(0, 30, 250)
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


class Visualiser(FlightEnvelopeAssessment):
    '''
    The Visualizer class plots the rolly, pitch, yaw data live and overlays a static plot to compare real-time
    data vs. bounds
    '''
    def __init__(self):
        super().__init__()
        self.start_time = time.time()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.velocity = 0.0
        self.horizontal_acceleration = 0.0
        self.vertical_acceleration = 0.0
        self.load_factor = 0.0
        self.current_state = None
        self.mode = None

        self.cb_time = 0.0
        self.cb_true_n = 0.0
        self.cb_true_v = 0.0
        self.cb_predict_n = 0.0
        self.cb_predict_v = 0.0
        self.cb_key_event = 0.0
        self.cb_key_event_time = 0.0

        self.time_list = []
        self.roll_list = []
        self.pitch_list = []
        self.yaw_list = []
        self.roll_rate_list = []
        self.pitch_rate_list = []
        self.yaw_rate_list = []
        self.velocity_list = []
        self.vertical_acceleration_list = []
        self.horizontal_acceleration_list = []

        self.cl_list = []
        self.load_factor_list = []

        # PLOTS predictions
        self.velocity_prediction_list = [0]
        self.load_factor_prediction_list = [0]

        self.vertical_acceleration_list_prediction = [0, 0]
        self.horizontal_acceleration_list_prediction = [0, 0]
        self.filteredAz_list = []
        self.filteredAx_list = []

        self.filteredAz = 0.0
        self.filteredAx = 0.0
        self.load_factor_predict = 0.0
        self.previous_filtered_load_factor = None
        self.weight = .1
        self.velocity_weight = 1.2

        subPosition = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.position_cb)
        subvfr_hud = rospy.Subscriber('mavros/vfr_hud', VFR_HUD, self.velocity_cb)
        subAcceleration = rospy.Subscriber('mavros/imu/data', Imu, self.acceleration_callback)
        subRates = rospy.Subscriber('mavros/imu/data', Imu, self.rates_cb)
        subState = rospy.Subscriber('mavros/state', State, callback=self.state_cb)
        subDataLogger = rospy.Subscriber('DataLogger', DataLogger, callback=self.data_logger_callback)
        self.pub = rospy.Publisher('DataLogger', DataLogger, queue_size=10)


        sns.set_style('whitegrid')
        sns.set_palette('colorblind')
        self.fig, (self.ax) = plt.subplots()

        self.colors = ['blue', 'red']
        self.labels = ['$n_{t}$', '$n_{p}$']
        self.lines = [self.ax.plot([], [], label=label, color=color, marker='o', linestyle='', markersize=4)[0] for label, color in zip(self.labels, self.colors)]

        static_velocity, static_load_factors = self.calc_load_factor_vs_velocity_static()
        self.static_plot(static_velocity, static_load_factors)

    def velocity_cb(self, msg):
        self.velocity = msg.airspeed
        self.cb_true_v = msg.airspeed
        self.velocity_list.append(msg.airspeed)

    def position_cb(self, msg):
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        self.roll, self.pitch, self.yaw = quaternionToEuler(qx, qy, qz, qw)   
        self.roll_list.append(self.roll)
        self.pitch_list.append(self.pitch)
        self.yaw_list.append(self.yaw)
        # print(f'roll: {np.rad2deg(self.roll):.4}, pitch: {np.rad2deg(self.pitch):.4}, yaw: {np.rad2deg(self.yaw):.4}')

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

    def data_logger_callback(self, msg):
        self.cb_time = msg.time
        self.cb_true_n = msg.true_n
        self.cb_true_v = msg.true_velocity
        self.cb_predict_n = msg.predicted_n
        self.cb_predict_v = msg.predicted_velocity
        self.cb_key_event = msg.key_event
        self.cb_key_event_time = msg.key_event_time

    def data_logger_publisher(self):
        data_logger = DataLogger()
        data_logger.time = self.time_list[-1]
        data_logger.true_n = self.load_factor_list[-1]
        data_logger.true_velocity = self.velocity_list[-1]
        a, b = self.butter_worth_filter()
        self.filteredAz, self.filteredAx = self.filter_signals(a, b)
        data_logger.predicted_n = self.load_factor_prediction_list[-1]
        data_logger.predicted_velocity = self.velocity_prediction_list[-1]
        
        self.pub.publish(data_logger)
        self.vertical_acceleration_list_prediction.append(self.filteredAz)
        self.horizontal_acceleration_list_prediction.append(self.filteredAx)

    def calc_load_factor(self):
        load_factor = self.filteredAz / self.g
        self.cb_true_n = load_factor
        self.load_factor_list.append(load_factor)
        self.load_factor = load_factor
        return load_factor

    def poly_predict(self, xlist, ylist, samplesize, x):
        coeffs = np.polyfit(xlist, ylist, samplesize)
        y_new = np.polyval(coeffs, x)
        return y_new

    def predict_next_load_factor(self):
        next_time_step = len(self.load_factor_list)
        predicted_load_factor = self.poly_predict(xlist=list(range(next_time_step)), ylist=self.load_factor_list, samplesize=min(3, len(self.load_factor_list) - 1), x=next_time_step)
        self.cb_predict_n = predicted_load_factor
        self.load_factor_prediction_list.append(predicted_load_factor)
        return predicted_load_factor

    def predict_velocity(self):
        v_pred = self.velocity + self.filteredAx * self.velocity_weight
        self.cb_predict_v = v_pred
        self.velocity_prediction_list.append(v_pred)
        # v_pred = self.velocity + self.horizontal_acceleration * (self.time_list[-1] - self.time_list[-2])
        return v_pred

    # butter worth difference equation
    def filter_signals(self, a, b):
        z_pred = b[0] * self.vertical_acceleration_list[-1] + b[1] * self.vertical_acceleration_list[-2] - (a[1] * self.vertical_acceleration_list_prediction[-1])
        x_pred = b[0] * self.horizontal_acceleration_list[-1] + b[1] * self.horizontal_acceleration_list[-2] - (a[1] * self.horizontal_acceleration_list_prediction[-1])
        # print(f"z_pred: {z_pred}")
        # print(f"x_pred: {x_pred}")
        self.filteredAz = z_pred
        self.filteredAx = x_pred
        self.vertical_acceleration_list_prediction.append(z_pred)
        self.horizontal_acceleration_list_prediction.append(x_pred)
        return z_pred, x_pred

    def butter_worth_filter(self):
        fs = 50
        cutoff = 2.5
        nyt = .5 * fs
        order = 1
        normalized_cutoff = cutoff / nyt
        b, a = signal.butter(order, normalized_cutoff, btype='low', analog=False)
        # filtered_signals = self.filter_signals(a, b)
        return a, b

    def plot_init_vn(self):
        self.ax.set_xlim(left=0, right=25)
        self.ax.set_ylim([-5, 5])
        self.ax.axhline(y=1, color='green', linestyle='--', alpha=0.2, linewidth=1)
        self.ax.axhline(y=-1, color='green', linestyle='--', alpha=0.2, linewidth=1)
        self.ax.set_xlabel('Velocity')
        # self.ax.set_xticks(range(0, int(ticks), 5))
        self.ax.set_ylabel('Load Factor')
        # self.ax.set_yticks(range(-ticks, ticks, 1))
        self.ax.set_title('Load Factor vs. Velocity')
        self.ax.grid(visible=True)
        self.ax.legend(fontsize='small')
        return self.lines

    def update_plot(self, frame):
        self.thinned_velocity_list = self.velocity_list[-10:]
        self.thinned_load_factor_list = self.load_factor_list[-10:]
        self.thinned_velocity_prediction_list = self.velocity_prediction_list[-10:]
        self.thinned_load_factor_prediction_list = self.load_factor_prediction_list[-10:]

        load_factor = self.calc_load_factor()
        self.load_factor_list.append(load_factor)
        self.predict_next_load_factor()
        self.predict_velocity()
        self.data_logger_publisher()

        if len(self.thinned_velocity_list) == len(self.thinned_velocity_prediction_list):
            self.lines[0].set_data(self.thinned_velocity_list, self.thinned_load_factor_list)
            self.lines[1].set_data(self.thinned_velocity_prediction_list, self.thinned_load_factor_prediction_list)
            return self.lines
        else:
            print("Getting current data")
            return self.lines
 
    def static_plot(self, static_velocity, static_load_factors):
        bounds = []
        line_styles = ['-', '-', '-', '-', '-']  # Define different line styles for each weight
        line_labels = ['$Cl_{Max}$', '$Cl_{Max}$ ⋅ 0.9', '$Cl_{Max}$ ⋅ 0.8', '$Cl_{Max}$ ⋅ 0.7', '$Cl_{Max}$ ⋅ 0.6']  # Define the labels for each line   
        line_colors = [plt.cm.Reds(x) for x in range(256, 128, -(256-128)//(6-1))]
        for load_factor, line_style, label, line_color in zip(static_load_factors, line_styles, line_labels, line_colors):
            self.ax.plot(static_velocity, load_factor, color=line_color, linestyle=line_style, label=label, alpha=1, linewidth=1)
            bounds.append((static_velocity, load_factor))
        return bounds

if __name__ == "__main__":

    rospy.init_node('flight_envelope_assessment_node')
    plt.close("all")
    rate = rospy.Rate(20)
    time_zero = rospy.get_time()
    bounds = FlightEnvelopeAssessment()
    vis = Visualiser()

    while not rospy.is_shutdown():
        ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init_vn, frames=1, interval=50, blit=False)
        plt.show(block=True)
        rate.sleep()
    plt.close('all')
