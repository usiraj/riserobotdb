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
        if angle_in_degrees < 30 or angle_in_degrees >= 180:
            raise ValueError('Control Param Angle out of valid range : %f'%angle_in_degrees)
        self.controlParamA = angle_in_degrees
        self._forwardsolve()

    def updateControlParamB(self, length_in_meters):
        if length_in_meters < 0 or length_in_meters >= (2*self.linklength):
            raise ValueError('Control Param Length out of valid range : %f'%length_in_meters)
        self.controlParamB = length_in_meters
        self._forwardsolve()

    def updateControlParams(self, angle_in_degrees, length_in_meters):
        if angle_in_degrees < 30 or angle_in_degrees >= 180:
            raise ValueError('Control Param Angle out of valid range : %f'%angle_in_degrees)
        if length_in_meters < 0 or length_in_meters >= (2*self.linklength):
            raise ValueError('Control Param Length out of valid range : %f'%length_in_meters)
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

    def ShowLegArcsA(self, stepsize=10):
        legarcs={}
        _val = self.ARange[0]
        _i = 1
        while _val <= self.ARange[1]:
            legarcs['arc_%03d'%_i] = {'param': _val, 'x': [], 'z':[], 'B':[]}
            _val += stepsize
            _i += 1
        ############ Generate Data ##########
        for _arc in legarcs:
            _pA = legarcs[_arc]['param']
            self.JSolver.updateControlParamA(_pA)
            _val = self.BRange[0]
            while _val <= self.BRange[1]:
                self.JSolver.updateControlParamB(_val/float(1000))
                _xyz = self.JSolver.getLegXZ()
                legarcs[_arc]['B'].append(_val)
                legarcs[_arc]['x'].append(_xyz[0, 0])
                legarcs[_arc]['z'].append(_xyz[2, 0])
                _val += 0.1
        ############ Plot Data ###############
        import matplotlib.pyplot as plt
        for _arc in legarcs:
            plt.plot(legarcs[_arc]['x'], legarcs[_arc]['z'])
        plt.axis('equal')
        plt.xlabel('X axis')
        plt.ylabel('Z axis')
        plt.title('XZ foot arcs for different fixed control param A:[%f,%f] with B:[%f,%f]'%(self.ARange[0],
                                                                                             self.ARange[1],
                                                                                             self.BRange[0],
                                                                                             self.BRange[1]))
        plt.grid()
        plt.gca().invert_xaxis()
        plt.show()

    def ShowLegArcsB(self, stepsize=1):
        legarcs = {}
        _val = self.BRange[0]
        _i = 1
        while _val <= self.BRange[1]:
            legarcs['arc_%03d' % _i] = {'param': _val, 'x': [], 'z': [], 'A': []}
            _val += stepsize
            _i += 1
        ############ Generate Data ##########
        for _arc in legarcs:
            _pB = legarcs[_arc]['param']
            self.JSolver.updateControlParamB(_pB / float(1000))
            _val = self.ARange[0]
            while _val <= self.ARange[1]:
                self.JSolver.updateControlParamA(_val)
                _xyz = self.JSolver.getLegXZ()
                legarcs[_arc]['A'].append(_val)
                legarcs[_arc]['x'].append(_xyz[0, 0])
                legarcs[_arc]['z'].append(_xyz[2, 0])
                _val += 0.2
        ############ Plot Data ###############
        import matplotlib.pyplot as plt
        for _arc in legarcs:
            plt.plot(legarcs[_arc]['x'], legarcs[_arc]['z'])
        plt.axis('equal')
        plt.xlabel('X axis')
        plt.ylabel('Z axis')
        plt.title('XZ foot arcs for different fixed control param B:[%f,%f] with A:[%f,%f]' % (self.BRange[0],
                                                                                               self.BRange[1],
                                                                                               self.ARange[0],
                                                                                               self.ARange[1]))
        plt.grid()
        plt.gca().invert_xaxis()
        plt.show()

    def JacobianAt(self, aparam, bparam):
        _deltaang = 0.1
        _delta = 0.01
        try:
            self.JSolver.updateControlParams(aparam + _deltaang, bparam/float(1000))
            _fahplus =self.JSolver.getLegXZ()
            self.JSolver.updateControlParams(aparam - _deltaang, bparam / float(1000))
            _fahminus = self.JSolver.getLegXZ()
            self.JSolver.updateControlParams(aparam, (bparam + _delta) / float(1000))
            _fbhplus = self.JSolver.getLegXZ()
            self.JSolver.updateControlParams(aparam, (bparam - _delta) / float(1000))
            _fbhminus = self.JSolver.getLegXZ()
        except ValueError:
            return None
        _JacobMat = np.zeros((2,2),dtype='float32')
        _JacobMat[0, 0] = (_fahplus[0] - _fahminus[0])/(2.0*_deltaang)
        _JacobMat[0, 1] = (_fbhplus[0] - _fbhminus[0]) / (2.0 * _delta)
        _JacobMat[1, 0] = (_fahplus[2] - _fahminus[2])/(2.0*_deltaang)
        _JacobMat[1, 1] = (_fbhplus[2] - _fbhminus[2])/(2.0*_delta)
        return _JacobMat

    def Jacobian(self, params):
        return self.JacobianAt(params[0], params[1])

    def SolveIK(self, xz, starting_ab, tolerance=1e-9):
        import scipy.optimize as sp
        __x0 = np.array(starting_ab)

        def _func(_params):
            try:
                self.JSolver.updateControlParams(_params[0], _params[1]/float(1000))
            except ValueError as r:
                raise r
            _tmp = self.JSolver.getLegXZ()
            _rval = np.zeros(2, dtype='float32')
            _rval[0] = (xz[0] - _tmp[0])*1000.0
            _rval[1] = (xz[1] - _tmp[2])*1000.0
            return _rval
        try:
            sol = sp.least_squares(_func, __x0, loss='soft_l1', diff_step=[0.1, 0.05],
                                   bounds=([self.ARange[0], self.BRange[0]], [self.ARange[1], self.BRange[1]]))
        except ValueError as p:
            return None
        if sol['cost'] > 0.01:
            return None
        if sol['success']:
            return sol.x
        else:
            return None


if __name__ == '__main__':
    fr = FelisRobotv1Joints()
    fr.updateControlParamB(0.025)
    fr.updateControlParamA(150)
    fr.printinfo()
    ks = FelisLegNumericSolverKinematics()
    #ks.ShowLegArcsA(stepsize=1)
    #ks.ShowLegArcsB(stepsize=0.5)
    _val = ks.SolveIK([0.04, -0.1], [90, 31])
    print _val
    if _val is not None:
        fr.updateControlParams(_val[0], _val[1]/float(1000))
        print fr.getLegXZ()*1000






