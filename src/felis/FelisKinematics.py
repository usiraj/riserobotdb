__author__ = 'Usama Siraj'
''' Kinematics Solver for felis Robot '''
import numpy as np
import math


class FelisRobotv1Joints():
    def __init__(self, linklegth=0.035):
        self.jointAName = 'None'
        self.jointBName = 'None'
        self.jointCName = 'None'
        self.jointDName = 'None'
        self.jointEName = 'None'
        self.jointFName = 'None'
        self.setLinkLength(linklegth)
        self.jointA = 0 # Degrees   +positive
        self.jointB = 0 # Degrees   -negative
        self.jointC = 0 # Degrees   +positive
        self.jointD = 0 # Degrees   -negative
        self.jointE = 0 # Degrees   -negative
        self.jointF = 0 # Degrees   +positive
        self.controlParamA = 0
        self.controlParamB = 0

    def set_jointNames(self, jA, jB, jC, jD, jE, jF):
        self.jointAName = jA
        self.jointBName = jB
        self.jointCName = jC
        self.jointDName = jD
        self.jointEName = jE
        self.jointFName = jF

    def updateControlParamA(self, angle_in_degrees):
        if angle_in_degrees < -45 or angle_in_degrees > 180:
            raise ValueError('Control Param Angle out of valid range')
        self.controlParamA = angle_in_degrees
        self._forwardsolve()

    def updateControlParamB(self, length_in_meters):
        if length_in_meters < 0 or length_in_meters >= (2*self.linklength):
            raise ValueError('Control Param Length out of valid range')
        self.controlParamB = length_in_meters
        self._forwardsolve()

    def updateControlParams(self, angle_in_degrees, length_in_meters):
        if angle_in_degrees < -45 or angle_in_degrees > 180:
            raise ValueError('Control Param Angle out of valid range')
        if length_in_meters < 0 or length_in_meters >= (2*self.linklength):
            raise ValueError('Control Param Length out of valid range')
        self.controlParamA = angle_in_degrees
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
        self.jointE = -2 * _thtemp
        self.jointB = -2 * _thtemp
        self.jointC = 2 * _thtemp
        self.jointD = -2 * _thtemp
        self.jointF = 2 * _thtemp

    def _getTx33Mat(self, phi,l):
        _sinphi = np.sin(math.radians(phi))
        _cosphi = np.cos(math.radians(phi))
        _tmat = np.ndarray((3,3), dtype='float32')
        _tmat[:, :] = 0.0
        _tmat[2, 2] = 1.0
        _tmat[0, 2] = l
        _tmat[0, 0] = _cosphi
        _tmat[1, 1] = _cosphi
        _tmat[0, 1] = -_sinphi
        _tmat[1, 0] = _sinphi
        return _tmat

    def getLegXZ(self):
        _t0a = self._getTx33Mat(self.jointA, 0.0)
        _tab = self._getTx33Mat(self.jointB, 0.0)
        _tbc = self._getTx33Mat(self.jointC, self.linklength)
        _tcd = self._getTx33Mat(self.jointD, 2.0*self.linklength)
        _tv = np.zeros((3, 1), dtype='float32')
        _tv[0, 0] = 2.0 * self.linklength
        _tv[2, 0] = 1.0
        _r = np.matmul(_tcd, _tv)
        _r = np.matmul(_tbc, _r)
        _r = np.matmul(_tab, _r)
        _r = np.matmul(_t0a, _r)
        _r[2] = -_r[1]
        _r[1] = 0.0
        return _r

    def syncJointInfo(self, jdict):
        jdict[self.jointAName]['value'] = self.jointA
        jdict[self.jointBName]['value'] = self.jointB
        jdict[self.jointCName]['value'] = self.jointC
        jdict[self.jointDName]['value'] = self.jointD
        jdict[self.jointEName]['value'] = self.jointE
        jdict[self.jointFName]['value'] = self.jointF


class FelisLegNumericSolverKinematics():
    def __init__(self, amin=75, amax=165, bmin=17.5, bmax=67, linklength=0.035):
        self.JSolver = FelisRobotv1Joints(linklength)
        self.ARange = [amin, amax]
        self.BRange = [bmin, bmax]






if __name__ == '__main__':
    fr = FelisRobotv1Joints()
    fr.updateControlParamB(0.025)
    fr.updateControlParamA(150)
    fr.printinfo()
    print fr.getLegXZ()
