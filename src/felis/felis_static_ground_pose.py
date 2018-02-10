#!/usr/bin/env python
__author__ = 'Usama Siraj'
'''Felis Static Ground Pose based on three contacts'''

import rospy, sys, math
import numpy as np
import xml.dom.minidom
from geometry_msgs.msg import PointStamped
import tf
from tf import transformations

def _get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        print 'has param'
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value


class FelisStaticGroundPoseEstimator():
    def __init__(self):
        description = _get_param('robot_description')
        if description is None:
            raise EnvironmentError('No robot description found on parameter server, check param remapping before'
                                   'starting the application')
            sys.exit(-1)
        self.tfprefix = _get_param('tf_prefix')
        if self.tfprefix is None:
            raise EnvironmentError('tf_prefix must be set in parameter server')
            sys.exit(-1)
        self.robot = xml.dom.minidom.parseString(description).getElementsByTagName('robot')[0]
        self.PreviousSolution = np.array([0, 0, 0],dtype='float64')
        rospy.loginfo('Loaded URDF for felis')
        self.leg_length = self.get_leg_length()
        #### Load from parameter server Frames ###
        self.ground_contact_frame = _get_param('ground_frame', 'ground_frame')
        ##########################################
        self.PotentialContactPoints = {'FL_Leg': None, 'FL_Joint': None, 'FR_Leg': None, 'FR_Joint': None,
                                       'RL_Leg': None, 'RL_Joint': None, 'RR_Leg': None, 'RR_Joint': None}
        self.listener = tf.TransformListener()
        self.publisher = tf.TransformBroadcaster()

    def get_leg_length(self):
        for child in self.robot.childNodes:
            if child.localName == 'joint' and child.getAttribute('name') == 'left_front_linkC_joint':
                orgval = child.getElementsByTagName('origin')[0].getAttribute('xyz')
                vals = orgval.split()[0]
                try:
                    num = float(vals)
                except ValueError:
                    raise RuntimeError('Could Not Get Link Length')
                return 2.0*num

    def convertPoint(self, from_frame, obs_frame, xyz):
        pt = PointStamped()
        pt.point.x = xyz[0]
        pt.point.y = xyz[1]
        pt.point.z = xyz[2]
        pt.header.stamp = rospy.Time(0)
        pt.header.frame_id = '/%s/%s'%(self.tfprefix, obs_frame)
        try:
            rpt = self.listener.transformPoint('/%s/%s'%(self.tfprefix, from_frame), pt)
            rval = [rpt.point.x, rpt.point.y, rpt.point.z]
            return rval
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None

    def updatePotentialContactPoints(self):
        fl_leg = self.convertPoint('base_link', 'left_front_quadleg_linkD', [self.leg_length, 0, 0.003])
        fr_leg = self.convertPoint('base_link', 'right_front_quadleg_linkD', [self.leg_length, 0, 0.003])
        rl_leg = self.convertPoint('base_link', 'left_rear_quadleg_linkD', [self.leg_length, 0, 0.003])
        rr_leg = self.convertPoint('base_link', 'right_rear_quadleg_linkD', [self.leg_length, 0, 0.003])
        fl_joint = self.convertPoint('base_link', 'left_front_quadleg_linkD', [0, 0, 0.003])
        fr_joint = self.convertPoint('base_link', 'right_front_quadleg_linkD', [0, 0, 0.003])
        rl_joint = self.convertPoint('base_link', 'left_rear_quadleg_linkD', [0, 0, 0.003])
        rr_joint = self.convertPoint('base_link', 'right_rear_quadleg_linkD', [0, 0, 0.003])
        if fl_leg is not None:
            self.PotentialContactPoints['FL_Leg'] = fl_leg
        if fr_leg is not None:
            self.PotentialContactPoints['FR_Leg'] = fr_leg
        if rl_leg is not None:
            self.PotentialContactPoints['RL_Leg'] = rl_leg
        if rr_leg is not None:
            self.PotentialContactPoints['RR_Leg'] = rr_leg
        if fl_joint is not None:
            self.PotentialContactPoints['FL_Joint'] = fl_joint
        if fr_joint is not None:
            self.PotentialContactPoints['FR_Joint'] = fr_joint
        if rl_joint is not None:
            self.PotentialContactPoints['RL_Joint'] = rl_joint
        if rr_joint is not None:
            self.PotentialContactPoints['RR_Joint'] = rr_joint

    def updatePose(self):
        _impossible = False
        for _pt in self.PotentialContactPoints:
            if self.PotentialContactPoints[_pt] is None:
                _impossible = True
                break
        if not _impossible:
            if self.PotentialContactPoints['FL_Leg'][2] < self.PotentialContactPoints['FL_Joint'][2]:
                _pA = self.PotentialContactPoints['FL_Leg']
            else:
                _pA = self.PotentialContactPoints['FL_Joint']
            if self.PotentialContactPoints['FR_Leg'][2] < self.PotentialContactPoints['FR_Joint'][2]:
                _pB = self.PotentialContactPoints['FR_Leg']
            else:
                _pB = self.PotentialContactPoints['FR_Joint']
            if self.PotentialContactPoints['RL_Leg'][2] < self.PotentialContactPoints['RL_Joint'][2]:
                _pC = self.PotentialContactPoints['RL_Leg']
            else:
                _pC = self.PotentialContactPoints['RL_Joint']
            if self.PotentialContactPoints['RR_Leg'][2] < self.PotentialContactPoints['RR_Joint'][2]:
                _pD = self.PotentialContactPoints['RR_Leg']
            else:
                _pD = self.PotentialContactPoints['RR_Joint']
            _Ts, _Tc = self._gplane_solutions(_pA, _pB, _pC, _pD)
            _Tfs, _Tfc = self._pick_closest_to_previous(_Ts, _Tc)
            ########## Publish Result Now ###########
            if _Tfs is not None:
                self.PreviousSolution = _Tfs
                self.publish_transform(_Tfs)

    def _pick_closest_to_previous(self, _sols, _solconts):
        if len(_sols) == 1:
            return _sols[0], _solconts[0]
        elif len(_sols) > 1:
            _dist = []
            for _i in xrange(len(_sols)):
                _diff = self.PreviousSolution - _sols[_i]
                _diff = np.multiply(_diff, _diff)
                _diff = np.sum(_diff)
                _dist.append(_diff)
            _imin = np.argmin(_dist)
            return _sols[_imin], _solconts[_imin]
        else:
            return None, None

    def _gplane_solutions(self, _pA, _pB, _pC, _pD):
        _Ans = []
        _Cnts = []
        _T0 = self._solve_ground_plane(_pA, _pB, _pC, _pD)
        _T1 = self._solve_ground_plane(_pA, _pB, _pD, _pC)
        _T2 = self._solve_ground_plane(_pD, _pB, _pC, _pA)
        _T3 = self._solve_ground_plane(_pA, _pD, _pC, _pB)
        if _T0 is not None:
            _Ans.append(_T0)
            _Cnts.append(['FL', 'FR', 'RL'])
        if _T1 is not None:
            _Ans.append(_T1)
            _Cnts.append(['FL', 'FR', 'RR'])
        if _T2 is not None:
            _Ans.append(_T2)
            _Cnts.append(['FR', 'RL', 'RR'])
        if _T3 is not None:
            _Ans.append(_T3)
            _Cnts.append(['FL', 'RL', 'RR'])
        if len(_Ans) > 1:
            _Ans, _Cnts = self._removeduplicates(_Ans, _Cnts)
        return  _Ans, _Cnts

    @staticmethod
    def _removeduplicates(_Sols, _Cnts):
        _drem = []
        _dcons = []
        for _i in xrange(len(_Sols)):
            _skipi = False
            for _j in xrange(_i+1, len(_Sols)):
                _diff = _Sols[_i] - _Sols[_j]
                _diff = np.multiply(_diff,_diff)
                _diff = np.sum(_diff)
                if _diff < 1e-6:
                    _skipi = True
                    break
            if not _skipi:
                _drem.append(_Sols[_i])
                _dcons.append(_Cnts[_i])
        return _drem, _dcons

    @staticmethod
    def _solve_ground_plane(_p1, _p2, _p3, _pTest):
        import scipy.optimize as sp
        _minimum_up = 20
        _maximum_up = 200
        _xyz_step = 0.001
        _p1mm = np.array([_p1[0] * 1000.0, _p1[1] * 1000.0, _p1[2] * 1000.0, 1.0], dtype=float)
        _p2mm = np.array([_p2[0] * 1000.0, _p2[1] * 1000.0, _p2[2] * 1000.0, 1.0], dtype=float)
        _p3mm = np.array([_p3[0] * 1000.0, _p3[1] * 1000.0, _p3[2] * 1000.0, 1.0], dtype=float)
        _ptesting = np.array([_pTest[0], _pTest[1], _pTest[2], 1.0])
        __x0 = np.zeros(3, dtype='float64')
        __x0[2] = -1.0 * (_p1mm[2] + _p2mm[2] + _p3mm[2]) / 3.0

        def _func(_params):
            _TMat = transformations.euler_matrix(math.radians(_params[0]), math.radians(_params[1]), 0.0)
            _TMat[2, 3] = _params[2]
            _residue = np.zeros(3, dtype='float64')
            _residue[0] = np.matmul(_TMat[2, :], _p1mm)
            _residue[1] = np.matmul(_TMat[2, :], _p2mm)
            _residue[2] = np.matmul(_TMat[2, :], _p3mm)
            return _residue
        try:
            sol = sp.least_squares(_func, __x0, loss='soft_l1', diff_step=[1e-2, 1e-2, _xyz_step],
                                   bounds=([-89.0, -89.0, _minimum_up],
                                           [89.0, 89.0, _maximum_up]))
        except ValueError as p:
            return None
        if sol['cost'] > 0.01:
            return None
        if sol['success']:
            _TMat = transformations.euler_matrix(math.radians(sol.x[0]), math.radians(sol.x[1]), 0.0)
            _TMat[2,3] = sol.x[2] / 1000.0
            _pTg = np.matmul(_TMat, _ptesting)
            if _pTg[2] > 0.0:
                return sol.x
            else:
                return None
        return None

    def publish_transform(self, _sol):
        _translation = (0.0, 0.0, _sol[2]/1000.0)
        _q = transformations.quaternion_from_euler(math.radians(_sol[0]), math.radians(_sol[1]), 0.0)
        try:
            self.publisher.sendTransform(_translation, _q, rospy.Time.now(),'/%s/%s'%(self.tfprefix,'base_link'),
                                         '/%s/%s'%(self.tfprefix, 'ground_plane'))
        except rospy.ROSException, ROSSerializationException:
            print 'Error occur publishing transform'

    def update_loop(self):
        hz = _get_param("rate", 25)
        r = rospy.Rate(hz)
        while not rospy.is_shutdown():
            self.updatePotentialContactPoints()
            self.updatePose()
            try:
                r.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                pass



if __name__ == '__main__':
    try:
        rospy.init_node('felis_static_ground_pose')
        rospy.loginfo('Starting Felis Static Ground Pose Estimator')
        fge = FelisStaticGroundPoseEstimator()
        fge.update_loop()
    except rospy.ROSInterruptException:
        pass