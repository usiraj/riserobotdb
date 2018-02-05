#!/usr/bin/env python

# reads the urdf of felis robot and publishes joint states after calculating them using control variables

import rospy
import random

from python_qt_binding.QtCore import pyqtSlot
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtCore import Signal
from python_qt_binding.QtWidgets import QApplication
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtWidgets import QSlider
from FelisJConf import Ui_FelisConfiguration
from FelisKinematics import FelisRobotv1Joints

import xml.dom.minidom
from sensor_msgs.msg import JointState
from math import pi
from threading import Thread
import sys
import signal
import math

RANGE = 10000

def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        print 'has param'
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value


######## Joint State Publisher ###########
class FelisJointStatePublisher():
    def __init__(self):
        description = get_param('robot_description')
        if description is None:
            raise EnvironmentError('No robot description found on parameter server, check param remapping before'
                                   'starting the application')
            sys.exit(-1)
        self.gui = None
        self.robot = xml.dom.minidom.parseString(description).getElementsByTagName('robot')[0]
        rospy.loginfo('Loaded URDF for felis')
        self.link_length = self.get_link_length() * 2
        rospy.loginfo('Link length loaded from urdf : %f', self.link_length)
        ######## Initialize All Joints #########
        self.joints = {}
        for jnt in self.get_all_joint_names():
            self.joints[jnt] = {'value': 0.0}
            _info = self.get_joint_info(jnt)
            self.joints[jnt]['max_effort'] = _info[0]
            self.joints[jnt]['max_velocity'] = _info[1]
            self.joints[jnt]['lower_val'] = _info[2]
            self.joints[jnt]['upper_val'] = _info[3]
        rospy.loginfo('Loaded Joint Constraints')
        self.rkinFL = FelisRobotv1Joints(self.link_length / 2)
        self.rkinFR = FelisRobotv1Joints(self.link_length / 2)
        self.rkinRL = FelisRobotv1Joints(self.link_length / 2)
        self.rkinRR = FelisRobotv1Joints(self.link_length / 2)
        #### Set Joint Names ####
        self.rkinFL.set_jointNames('left_front_linkA_joint', 'left_front_linkB_joint', 'left_front_linkC_joint',
                                   'left_front_linkD_joint', 'left_front_linkE_joint', 'left_front_linkF_joint')
        self.rkinFR.set_jointNames('right_front_linkA_joint', 'right_front_linkB_joint', 'right_front_linkC_joint',
                                   'right_front_linkD_joint', 'right_front_linkE_joint', 'right_front_linkF_joint')
        self.rkinRL.set_jointNames('left_rear_linkA_joint', 'left_rear_linkB_joint', 'left_rear_linkC_joint',
                                   'left_rear_linkD_joint', 'left_rear_linkE_joint', 'left_rear_linkF_joint')
        self.rkinRR.set_jointNames('right_rear_linkA_joint', 'right_rear_linkB_joint', 'right_rear_linkC_joint',
                                   'right_rear_linkD_joint', 'right_rear_linkE_joint', 'right_rear_linkF_joint')
        # TODO compute b_min and b_max
        limits = {'a_min':self.joints['left_front_linkA_joint']['lower_val'],
                  'a_max':self.joints['left_front_linkA_joint']['upper_val'],
                  'b_min':(self.link_length*1000)/float(4), 'b_max': (self.link_length*1000)-3}
        ####### GUI Functionality ##############
        use_gui = get_param("use_gui", True)
        self.close_on_exit = get_param("close_on_exit", True)
        if use_gui:
            self.app = QApplication(sys.argv)
            self.gui = FelisConfigurationGui(self, limits)
            self.gui.show()
        else:
            self.gui = None
        ######### Dummy Initial Publish #####
        self.rkinFL.syncJointInfo(self.joints)
        self.rkinFR.syncJointInfo(self.joints)
        self.rkinRL.syncJointInfo(self.joints)
        self.rkinRR.syncJointInfo(self.joints)
        ######## Subscribe for Control Parameters from outside #####
        # TODO
        ####### Publisher ##########
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=5)

    def loop(self):
        hz = get_param("rate", 100)
        r = rospy.Rate(hz)
        while not rospy.is_shutdown():
            if self.gui is not None and self.close_on_exit and not self.gui.isVisible():
                rospy.loginfo('Exiting Publishing Joint States, since GUI is closed')
                break
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = []
            msg.position = []
            msg.velocity = []
            msg.effort = []
            # Initialize msg.position, msg.velocity and msg.effort
            for joint in self.joints:
                msg.name.append(joint)
                msg.position.append(self.joints[joint]['value'])
            if msg.name or msg.position or msg.velocity or msg.effort:
                # Only publish non-empty messages
                self.pub.publish(msg)
            try:
                r.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                pass

    def get_link_length(self):
        for child in self.robot.childNodes:
            if child.localName == 'joint' and child.getAttribute('name') == 'left_front_linkC_joint':
                orgval = child.getElementsByTagName('origin')[0].getAttribute('xyz')
                vals = orgval.split()[0]
                try:
                    num = float(vals)
                except ValueError:
                    raise RuntimeError('Could Not Get Link Length')
                return num

    def get_all_joint_names(self):
        jnames = []
        for child in self.robot.childNodes:
            if child.localName == 'joint':
                jnames.append(str(child.getAttribute('name')))
        return jnames

    def get_joint_info(self, jname):
        for child in self.robot.childNodes:
            if child.localName == 'joint' and child.getAttribute('name') == jname:
                limit_node = child.getElementsByTagName('limit')[0]
                max_effort = limit_node.getAttribute('effort')
                lower_val = limit_node.getAttribute('lower')
                upper_val = limit_node.getAttribute('upper')
                max_velocity = limit_node.getAttribute('velocity')
                try:
                    max_effort = float(max_effort)
                except ValueError:
                    raise RuntimeError('Could not get maximum effort for joint %s'%jname)
                try:
                    max_velocity = float(max_velocity)
                except ValueError:
                    raise RuntimeError('Could not get maximum velocity for joint %s'%jname)
                try:
                    lower_val = math.degrees(float(lower_val))
                except ValueError:
                    raise RuntimeError('Could not get minimum angle for joint %s'%jname)
                try:
                    upper_val = math.degrees(float(upper_val))
                except ValueError:
                    raise RuntimeError('Could not get maximum angle for joint %s'%jname)
                return max_effort, max_velocity, lower_val, upper_val




######## Joint Configuration Optional GUI ###########
class FelisConfigurationGui(QWidget):
    sliderUpdateTrigger = Signal()

    def __init__(self, jsp, limits):
        super(FelisConfigurationGui, self).__init__()
        self.jsp = jsp
        self.ui = Ui_FelisConfiguration()
        self.ui.setupUi(self)
        ### Initialize other stuff ######
        self.ui.le_fl_a.setRange(limits['a_min'], limits['a_max'])
        self.ui.le_fr_a.setRange(limits['a_min'], limits['a_max'])
        self.ui.le_rl_a.setRange(limits['a_min'], limits['a_max'])
        self.ui.le_rr_a.setRange(limits['a_min'], limits['a_max'])
        self.ui.le_fl_b.setRange(limits['b_min'], limits['b_max'])
        self.ui.le_fr_b.setRange(limits['b_min'], limits['b_max'])
        self.ui.le_rl_b.setRange(limits['b_min'], limits['b_max'])
        self.ui.le_rr_b.setRange(limits['b_min'], limits['b_max'])
        self.ui.le_fl_a.setSingleStep(0.01)
        self.ui.le_fr_a.setSingleStep(0.01)
        self.ui.le_rl_a.setSingleStep(0.01)
        self.ui.le_rr_a.setSingleStep(0.01)
        self.ui.le_fl_b.setSingleStep(0.01)
        self.ui.le_fr_b.setSingleStep(0.01)
        self.ui.le_rl_b.setSingleStep(0.01)
        self.ui.le_rr_b.setSingleStep(0.01)
        self.ui.le_fl_x.setSingleStep(0.001)
        self.ui.le_fr_x.setSingleStep(0.001)
        self.ui.le_rl_x.setSingleStep(0.001)
        self.ui.le_rr_x.setSingleStep(0.001)
        self.ui.le_fl_z.setSingleStep(0.001)
        self.ui.le_fr_z.setSingleStep(0.001)
        self.ui.le_rl_z.setSingleStep(0.001)
        self.ui.le_rr_z.setSingleStep(0.001)
        self.ui.le_fl_x.setDecimals(3)
        self.ui.le_fr_x.setDecimals(3)
        self.ui.le_rl_x.setDecimals(3)
        self.ui.le_rr_x.setDecimals(3)
        self.ui.le_fl_z.setDecimals(3)
        self.ui.le_fr_z.setDecimals(3)
        self.ui.le_rl_z.setDecimals(3)
        self.ui.le_rr_z.setDecimals(3)
        self.ui.slider_fl_a.setRange(int(limits['a_min']*100), int(limits['a_max']*100))
        self.ui.slider_fr_a.setRange(int(limits['a_min'] * 100), int(limits['a_max'] * 100))
        self.ui.slider_rl_a.setRange(int(limits['a_min'] * 100), int(limits['a_max'] * 100))
        self.ui.slider_rr_a.setRange(int(limits['a_min'] * 100), int(limits['a_max'] * 100))
        self.ui.slider_fl_b.setRange(int(limits['b_min']*100), int(limits['b_max']*100))
        self.ui.slider_fr_b.setRange(int(limits['b_min'] * 100), int(limits['b_max'] * 100))
        self.ui.slider_rl_b.setRange(int(limits['b_min'] * 100), int(limits['b_max'] * 100))
        self.ui.slider_rr_b.setRange(int(limits['b_min'] * 100), int(limits['b_max'] * 100))
        self.ui.slider_fl_a.valueChanged.connect(self.fl_a_changed)
        self.ui.le_fl_a.valueChanged.connect(self.fl_a_finechanged)
        self.ui.slider_fr_a.valueChanged.connect(self.fr_a_changed)
        self.ui.le_fr_a.valueChanged.connect(self.fr_a_finechanged)
        self.ui.slider_rl_a.valueChanged.connect(self.rl_a_changed)
        self.ui.le_rl_a.valueChanged.connect(self.rl_a_finechanged)
        self.ui.slider_rr_a.valueChanged.connect(self.rr_a_changed)
        self.ui.le_rr_a.valueChanged.connect(self.rr_a_finechanged)
        self.ui.slider_fl_b.valueChanged.connect(self.fl_b_changed)
        self.ui.le_fl_b.valueChanged.connect(self.fl_b_finechanged)
        self.ui.slider_fr_b.valueChanged.connect(self.fr_b_changed)
        self.ui.le_fr_b.valueChanged.connect(self.fr_b_finechanged)
        self.ui.slider_rl_b.valueChanged.connect(self.rl_b_changed)
        self.ui.le_rl_b.valueChanged.connect(self.rl_b_finechanged)
        self.ui.slider_rr_b.valueChanged.connect(self.rr_b_changed)
        self.ui.le_rr_b.valueChanged.connect(self.rr_b_finechanged)
        ########## Values (Dummy Initial) ###############
        self.set_fl_a(150)
        self.set_fr_a(150)
        self.set_rl_a(150)
        self.set_rr_a(150)
        self.set_fl_b(35)
        self.set_fr_b(35)
        self.set_rl_b(35)
        self.set_rr_b(35)
        self.jsp.rkinFL.updateControlParams(self.val_a_fl, self.val_b_fl/float(1000))
        self.jsp.rkinFR.updateControlParams(self.val_a_fr, self.val_b_fr/float(1000))
        self.jsp.rkinRL.updateControlParams(self.val_a_rl, self.val_b_rl/float(1000))
        self.jsp.rkinRR.updateControlParams(self.val_a_rr, self.val_b_rr/float(1000))
        self.val_x_fl = 0.0
        self.val_x_fr = 0.0
        self.val_x_rl = 0.0
        self.val_x_rr = 0.0
        self.val_z_fl = 0.0
        self.val_z_fr = 0.0
        self.val_z_rl = 0.0
        self.val_z_rr = 0.0


        ################# Param A UI #############################
    def fl_a_changed(self, valtochange):
        if self.val_a_fl != float(valtochange)/100:
            self.val_a_fl = float(self.ui.slider_fl_a.value()) / 100
            self.ui.le_fl_a.setValue(self.val_a_fl)
            self.val_fl_a_updated()

    def fl_a_finechanged(self, valtochange):
        if valtochange != self.val_a_fl:
            self.val_a_fl = self.ui.le_fl_a.value()
            self.ui.slider_fl_a.setValue(int(self.val_a_fl*100))
            self.val_fl_a_updated()

    def fr_a_changed(self, valtochange):
        if self.val_a_fr != float(valtochange) / 100:
            self.val_a_fr = float(self.ui.slider_fr_a.value()) / 100
            self.ui.le_fr_a.setValue(self.val_a_fr)
            self.val_fr_a_updated()

    def fr_a_finechanged(self, valtochange):
        if valtochange != self.val_a_fr:
            self.val_a_fr = self.ui.le_fr_a.value()
            self.ui.slider_fr_a.setValue(int(self.val_a_fr * 100))
            self.val_fr_a_updated()

    def rl_a_changed(self, valtochange):
        if self.val_a_rl != float(valtochange) / 100:
            self.val_a_rl = float(self.ui.slider_rl_a.value()) / 100
            self.ui.le_rl_a.setValue(self.val_a_rl)
            self.val_rl_a_updated()

    def rl_a_finechanged(self, valtochange):
        if valtochange != self.val_a_rl:
            self.val_a_rl = self.ui.le_rl_a.value()
            self.ui.slider_rl_a.setValue(int(self.val_a_rl * 100))
            self.val_rl_a_updated()

    def rr_a_changed(self, valtochange):
        if self.val_a_rr != float(valtochange) / 100:
            self.val_a_rr = float(self.ui.slider_rr_a.value()) / 100
            self.ui.le_rr_a.setValue(self.val_a_rr)
            self.val_rr_a_updated()

    def rr_a_finechanged(self, valtochange):
        if valtochange != self.val_a_rr:
            self.val_a_rr = self.ui.le_rr_a.value()
            self.ui.slider_rr_a.setValue(int(self.val_a_rr * 100))
            self.val_rr_a_updated()

    def fl_b_changed(self, valtochange):
        if self.val_b_fl != float(valtochange) / 100:
            self.val_b_fl = float(self.ui.slider_fl_b.value()) / 100
            self.ui.le_fl_b.setValue(self.val_b_fl)
            self.val_fl_b_updated()

    def fl_b_finechanged(self, valtochange):
        if valtochange != self.val_b_fl:
            self.val_b_fl = self.ui.le_fl_b.value()
            self.ui.slider_fl_b.setValue(int(self.val_b_fl * 100))
            self.val_fl_b_updated()

    def fr_b_changed(self, valtochange):
        if self.val_b_fr != float(valtochange) / 100:
            self.val_b_fr = float(self.ui.slider_fr_b.value()) / 100
            self.ui.le_fr_b.setValue(self.val_b_fr)
            self.val_fr_b_updated()

    def fr_b_finechanged(self, valtochange):
        if valtochange != self.val_b_fr:
            self.val_b_fr = self.ui.le_fr_b.value()
            self.ui.slider_fr_b.setValue(int(self.val_b_fr * 100))
            self.val_fr_b_updated()

    def rl_b_changed(self, valtochange):
        if self.val_b_rl != float(valtochange) / 100:
            self.val_b_rl = float(self.ui.slider_rl_b.value()) / 100
            self.ui.le_rl_b.setValue(self.val_b_rl)
            self.val_rl_b_updated()

    def rl_b_finechanged(self, valtochange):
        if valtochange != self.val_b_rl:
            self.val_b_rl = self.ui.le_rl_b.value()
            self.ui.slider_rl_b.setValue(int(self.val_b_rl * 100))
            self.val_rl_b_updated()

    def rr_b_changed(self, valtochange):
        if self.val_b_rr != float(valtochange) / 100:
            self.val_b_rr = float(self.ui.slider_rr_b.value()) / 100
            self.ui.le_rr_b.setValue(self.val_b_rr)
            self.val_rr_b_updated()

    def rr_b_finechanged(self, valtochange):
        if valtochange != self.val_b_rr:
            self.val_b_rr = self.ui.le_rr_b.value()
            self.ui.slider_rr_b.setValue(int(self.val_b_rr * 100))
            self.val_rr_b_updated()

    ########### After Control Parameters are Updated ##############
    def val_fl_a_updated(self):
        self.jsp.rkinFL.updateControlParamA(self.val_a_fl)
        self.jsp.rkinFL.syncJointInfo(self.jsp.joints)

    def val_fr_a_updated(self):
        self.jsp.rkinFR.updateControlParamA(self.val_a_fr)
        self.jsp.rkinFR.syncJointInfo(self.jsp.joints)

    def val_rl_a_updated(self):
        self.jsp.rkinRL.updateControlParamA(self.val_a_rl)
        self.jsp.rkinRL.syncJointInfo(self.jsp.joints)

    def val_rr_a_updated(self):
        self.jsp.rkinRR.updateControlParamA(self.val_a_rr)
        self.jsp.rkinRR.syncJointInfo(self.jsp.joints)

    def val_fl_b_updated(self):
        self.jsp.rkinFL.updateControlParamB(self.val_b_fl/float(1000))
        self.jsp.rkinFL.syncJointInfo(self.jsp.joints)

    def val_fr_b_updated(self):
        self.jsp.rkinFR.updateControlParamB(self.val_b_fr / float(1000))
        self.jsp.rkinFR.syncJointInfo(self.jsp.joints)

    def val_rl_b_updated(self):
        self.jsp.rkinRL.updateControlParamB(self.val_b_rl / float(1000))
        self.jsp.rkinRL.syncJointInfo(self.jsp.joints)

    def val_rr_b_updated(self):
        self.jsp.rkinRR.updateControlParamB(self.val_b_rr / float(1000))
        self.jsp.rkinRR.syncJointInfo(self.jsp.joints)


    ###############################################################


    ########## Set Values ###############
    def set_fl_a(self, val):
        self.val_a_fl = val
        self.ui.le_fl_a.setValue(self.val_a_fl)
        self.ui.slider_fl_a.setSliderPosition(int(self.val_a_fl * 100))

    def set_fr_a(self, val):
        self.val_a_fr = val
        self.ui.le_fr_a.setValue(self.val_a_fr)
        self.ui.slider_fr_a.setSliderPosition(int(self.val_a_fr * 100))

    def set_rl_a(self, val):
        self.val_a_rl = val
        self.ui.le_rl_a.setValue(self.val_a_rl)
        self.ui.slider_rl_a.setSliderPosition(int(self.val_a_rl * 100))

    def set_rr_a(self, val):
        self.val_a_rr = val
        self.ui.le_rr_a.setValue(self.val_a_rr)
        self.ui.slider_rr_a.setSliderPosition(int(self.val_a_rr * 100))

    def set_fl_b(self, val):
        self.val_b_fl = val
        self.ui.le_fl_b.setValue(self.val_b_fl)
        self.ui.slider_fl_b.setSliderPosition(int(self.val_b_fl * 100))

    def set_fr_b(self, val):
        self.val_b_fr = val
        self.ui.le_fr_b.setValue(self.val_b_fr)
        self.ui.slider_fr_b.setSliderPosition(int(self.val_b_fr * 100))

    def set_rl_b(self, val):
        self.val_b_rl = val
        self.ui.le_rl_b.setValue(self.val_b_rl)
        self.ui.slider_rl_b.setSliderPosition(int(self.val_b_rl * 100))

    def set_rr_b(self, val):
        self.val_b_rr = val
        self.ui.le_rr_b.setValue(self.val_b_rr)
        self.ui.slider_rr_b.setSliderPosition(int(self.val_b_rr * 100))


# the main of module
if __name__ == '__main__':
    try:
        rospy.init_node('felis_joint_state_publisher')
        jsp = FelisJointStatePublisher()
        if jsp.gui is None:
            print 'some'
            jsp.loop()
        else:
            print 'Starting'
            Thread(target=jsp.loop).start()
            signal.signal(signal.SIGINT, signal.SIG_DFL)
            sys.exit(jsp.app.exec_())
            print 'Exiting'
    except rospy.ROSInterruptException:
        pass
