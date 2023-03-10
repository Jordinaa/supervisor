#!/usr/bin/env python3

import math
import matplotlib.pyplot as plt
import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from std_msgs.msg import Header

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
    assessment = FlightEnvelopeAssessment(A0, CLA, CDA, ALPHA_STALL, WINGAREA, AIR_DENSITY, MASS, G)
    print(assessment.clMax)