__author__ = 'Usama Siraj'
''' Kinematics Solver for felis Robot '''
import numpy as np
import math


class FelisRobotv1Joints():
    def __init__(self, linklegth=0.035):
        self.setLinkLength(linklegth)
        self.jointA = 0 # Degrees   -negative
        self.jointB = 0 # Degrees   -negative
        self.jointC = 0 # Degrees   +positive
        self.jointD = 0 # Degrees   -negative
        self.jointE = 0 # Degrees   -negative
        self.jointF = 0 # Degrees   +positive

        self.controlParamA = 0
        self.controlParamB = 0

        pass

    def updateControlParamA(self, angle_in_degrees):
        if angle_in_degrees > 0 or angle_in_degrees < -180:
            raise ValueError('Control Param Angle out of valid range')
        self.controlParamA = angle_in_degrees
        self._forwardsolve()

    def updateControlParamB(self, length_in_meters):
        if length_in_meters < 0 or length_in_meters >= (2*self.linklength):
            raise ValueError('Control Param Length out of valid range')
        self.controlParamB = length_in_meters
        self._forwardsolve()

    def updateControlParams(self, angle_in_degress, length_in_meters):
        if angle_in_degress > 0 or angle_in_degress < -180:
            raise ValueError('Control Param Angle out of valid range')
        if length_in_meters < 0 or length_in_meters >= (2*self.linklength):
            raise ValueError('Control Param Length out of valid range')
        self.controlParamA = angle_in_degress
        self.controlParamB = length_in_meters
        self._forwardsolve()

    def setLinkLength(self, linklength):
        if linklength < 0:
            raise ValueError('Link Length must be a positive number')
        else:
            self.linklength = linklength

    def printinfo(self):
        print 'Felis Robot Configuration [%f,%f] --> [%f,%f,%f,%f,%f,%f]' % (self.controlParamA, self.controlParamB,
                                                                             self.jointA, self.jointB, self.jointC,
                                                                             self.jointD, self.jointE, self.jointF)

    def _forwardsolve(self):
        self.jointA = self.controlParamA
        _thtemp = math.degrees(math.acos(self.controlParamB / (2.0*self.linklength)))
        self.jointE = 2 * _thtemp
        self.jointB = 2 * _thtemp
        self.jointC = -2 * _thtemp
        self.jointD = 2 * _thtemp
        self.jointF = -2 * _thtemp


if __name__ == '__main__':
    fr = FelisRobotv1Joints()
    fr.printinfo()
    fr.updateControlParamB(0.005)
    fr.updateControlParamA(-90)
    fr.printinfo()
