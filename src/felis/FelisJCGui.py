#!/usr/bin/env python
__author__ = 'Usama Siraj'
''' Joint Configuration Basic GUI'''


import rospy, sys, math
import xml.dom.minidom
import tf
from threading import Thread
import signal
from python_qt_binding.QtCore import Signal
from python_qt_binding.QtWidgets import QApplication
from python_qt_binding.QtWidgets import QWidget
from FelisJConf import Ui_FelisConfiguration
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped


######## Joint Configuration Optional GUI ###########
class FelisConfigurationGui(QWidget):
    sliderUpdateTrigger = Signal()

    def __init__(self):
        super(FelisConfigurationGui, self).__init__()
        self.ui = Ui_FelisConfiguration()
        self.ui.setupUi(self)
        ### Initialize other stuff ######
        self.tfprefix = None
        self.robot = None
        self.link_lengthC = None
        self.limits = None
        self.FootPositions = {'FL': [0, 0, 0], 'FR': [0, 0, 0], 'RL': [0, 0, 0], 'RR': [0, 0, 0]}
        self.initialize_robot()

        self.ui.le_fl_a.setRange(self.limits['a_min'], self.limits['a_max'])
        self.ui.le_fr_a.setRange(self.limits['a_min'], self.limits['a_max'])
        self.ui.le_rl_a.setRange(self.limits['a_min'], self.limits['a_max'])
        self.ui.le_rr_a.setRange(self.limits['a_min'], self.limits['a_max'])
        self.ui.le_fl_b.setRange(self.limits['b_min'], self.limits['b_max'])
        self.ui.le_fr_b.setRange(self.limits['b_min'], self.limits['b_max'])
        self.ui.le_rl_b.setRange(self.limits['b_min'], self.limits['b_max'])
        self.ui.le_rr_b.setRange(self.limits['b_min'], self.limits['b_max'])
        self.ui.le_fl_a.setSingleStep(0.01)
        self.ui.le_fr_a.setSingleStep(0.01)
        self.ui.le_rl_a.setSingleStep(0.01)
        self.ui.le_rr_a.setSingleStep(0.01)
        self.ui.le_fl_b.setSingleStep(0.01)
        self.ui.le_fr_b.setSingleStep(0.01)
        self.ui.le_rl_b.setSingleStep(0.01)
        self.ui.le_rr_b.setSingleStep(0.01)
        self.ui.slider_fl_a.setRange(int(self.limits['a_min']*100), int(self.limits['a_max']*100))
        self.ui.slider_fr_a.setRange(int(self.limits['a_min'] * 100), int(self.limits['a_max'] * 100))
        self.ui.slider_rl_a.setRange(int(self.limits['a_min'] * 100), int(self.limits['a_max'] * 100))
        self.ui.slider_rr_a.setRange(int(self.limits['a_min'] * 100), int(self.limits['a_max'] * 100))
        self.ui.slider_fl_b.setRange(int(self.limits['b_min']*100), int(self.limits['b_max']*100))
        self.ui.slider_fr_b.setRange(int(self.limits['b_min'] * 100), int(self.limits['b_max'] * 100))
        self.ui.slider_rl_b.setRange(int(self.limits['b_min'] * 100), int(self.limits['b_max'] * 100))
        self.ui.slider_rr_b.setRange(int(self.limits['b_min'] * 100), int(self.limits['b_max'] * 100))
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
        self.updateFootPositionsDisplay()
        ######### Publisher / Subscriber ################
        self.listener = tf.TransformListener()
        self.pub = rospy.Publisher('felis_control_params', JointState, queue_size=5)

    def initialize_robot(self):
        description = self.get_param('robot_description')
        if description is None:
            raise EnvironmentError('No robot description found on parameter server, check param remapping before'
                                   'starting the application')
            sys.exit(-1)
        self.tfprefix = self.get_param('tf_prefix')
        if self.tfprefix is None:
            raise EnvironmentError('tf_prefix must be set in parameter server')
            sys.exit(-1)
        self.robot = xml.dom.minidom.parseString(description).getElementsByTagName('robot')[0]
        rospy.loginfo('Loaded URDF for felis')
        self.link_lengthC = self._get_link_length() * 2
        rospy.loginfo('Leg length loaded from urdf : %f', self.link_lengthC)
        lower_val = None
        upper_val = None
        for child in self.robot.childNodes:
            if child.localName == 'joint' and child.getAttribute('name') == 'left_front_linkA_joint':
                limit_node = child.getElementsByTagName('limit')[0]
                lower_val = limit_node.getAttribute('lower')
                upper_val = limit_node.getAttribute('upper')
                try:
                    lower_val = math.degrees(float(lower_val))
                except ValueError:
                    raise RuntimeError('Could not get minimum angle for joint left_front_linkA_joint')
                try:
                    upper_val = math.degrees(float(upper_val))
                except ValueError:
                    raise RuntimeError('Could not get maximum angle for joint left_front_linkA_joint')
                break
        ########## Update Limits ########
        if lower_val is not None and upper_val is not None:
            self.limits = {'a_min':lower_val, 'a_max':upper_val,
                           'b_min':(self.link_lengthC*1000)/float(4), 'b_max': (self.link_lengthC*1000)-3}

    @staticmethod
    def get_param( name, value=None):
        private = "~%s" % name
        if rospy.has_param(private):
            print 'has param'
            return rospy.get_param(private)
        elif rospy.has_param(name):
            return rospy.get_param(name)
        else:
            return value

    def _get_link_length(self):
        for child in self.robot.childNodes:
            if child.localName == 'joint' and child.getAttribute('name') == 'left_front_linkC_joint':
                orgval = child.getElementsByTagName('origin')[0].getAttribute('xyz')
                vals = orgval.split()[0]
                try:
                    num = float(vals)
                except ValueError:
                    raise RuntimeError('Could Not Get Link Length')
                return num

    ################# Param A UI #############################
    def fl_a_changed(self, valtochange):
        if self.val_a_fl != float(valtochange)/100:
            self.val_a_fl = float(self.ui.slider_fl_a.value()) / 100
            self.ui.le_fl_a.setValue(self.val_a_fl)

    def fl_a_finechanged(self, valtochange):
        if valtochange != self.val_a_fl:
            self.val_a_fl = self.ui.le_fl_a.value()
            self.ui.slider_fl_a.setValue(int(self.val_a_fl*100))

    def fr_a_changed(self, valtochange):
        if self.val_a_fr != float(valtochange) / 100:
            self.val_a_fr = float(self.ui.slider_fr_a.value()) / 100
            self.ui.le_fr_a.setValue(self.val_a_fr)

    def fr_a_finechanged(self, valtochange):
        if valtochange != self.val_a_fr:
            self.val_a_fr = self.ui.le_fr_a.value()
            self.ui.slider_fr_a.setValue(int(self.val_a_fr * 100))

    def rl_a_changed(self, valtochange):
        if self.val_a_rl != float(valtochange) / 100:
            self.val_a_rl = float(self.ui.slider_rl_a.value()) / 100
            self.ui.le_rl_a.setValue(self.val_a_rl)

    def rl_a_finechanged(self, valtochange):
        if valtochange != self.val_a_rl:
            self.val_a_rl = self.ui.le_rl_a.value()
            self.ui.slider_rl_a.setValue(int(self.val_a_rl * 100))

    def rr_a_changed(self, valtochange):
        if self.val_a_rr != float(valtochange) / 100:
            self.val_a_rr = float(self.ui.slider_rr_a.value()) / 100
            self.ui.le_rr_a.setValue(self.val_a_rr)

    def rr_a_finechanged(self, valtochange):
        if valtochange != self.val_a_rr:
            self.val_a_rr = self.ui.le_rr_a.value()
            self.ui.slider_rr_a.setValue(int(self.val_a_rr * 100))

    def fl_b_changed(self, valtochange):
        if self.val_b_fl != float(valtochange) / 100:
            self.val_b_fl = float(self.ui.slider_fl_b.value()) / 100
            self.ui.le_fl_b.setValue(self.val_b_fl)

    def fl_b_finechanged(self, valtochange):
        if valtochange != self.val_b_fl:
            self.val_b_fl = self.ui.le_fl_b.value()
            self.ui.slider_fl_b.setValue(int(self.val_b_fl * 100))

    def fr_b_changed(self, valtochange):
        if self.val_b_fr != float(valtochange) / 100:
            self.val_b_fr = float(self.ui.slider_fr_b.value()) / 100
            self.ui.le_fr_b.setValue(self.val_b_fr)

    def fr_b_finechanged(self, valtochange):
        if valtochange != self.val_b_fr:
            self.val_b_fr = self.ui.le_fr_b.value()
            self.ui.slider_fr_b.setValue(int(self.val_b_fr * 100))

    def rl_b_changed(self, valtochange):
        if self.val_b_rl != float(valtochange) / 100:
            self.val_b_rl = float(self.ui.slider_rl_b.value()) / 100
            self.ui.le_rl_b.setValue(self.val_b_rl)

    def rl_b_finechanged(self, valtochange):
        if valtochange != self.val_b_rl:
            self.val_b_rl = self.ui.le_rl_b.value()
            self.ui.slider_rl_b.setValue(int(self.val_b_rl * 100))

    def rr_b_changed(self, valtochange):
        if self.val_b_rr != float(valtochange) / 100:
            self.val_b_rr = float(self.ui.slider_rr_b.value()) / 100
            self.ui.le_rr_b.setValue(self.val_b_rr)

    def rr_b_finechanged(self, valtochange):
        if valtochange != self.val_b_rr:
            self.val_b_rr = self.ui.le_rr_b.value()
            self.ui.slider_rr_b.setValue(int(self.val_b_rr * 100))

    ###############################################################
    def loop(self):
        hz = self.get_param("rate", 100)
        r = rospy.Rate(hz)
        while not rospy.is_shutdown():
            if not self.isVisible():
                rospy.loginfo('Exiting Publishing Joint States, since GUI is closed')
                break
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = []
            msg.position = []
            msg.velocity = []
            msg.effort = []
            ######### Fill up message #######
            msg.name.append('felis_fl_cpa')
            msg.position.append(self.val_a_fl)
            msg.name.append('felis_fr_cpa')
            msg.position.append(self.val_a_fr)
            msg.name.append('felis_rl_cpa')
            msg.position.append(self.val_a_rl)
            msg.name.append('felis_rr_cpa')
            msg.position.append(self.val_a_rr)
            msg.name.append('felis_fl_cpb')
            msg.position.append(self.val_b_fl)
            msg.name.append('felis_fr_cpb')
            msg.position.append(self.val_b_fr)
            msg.name.append('felis_rl_cpb')
            msg.position.append(self.val_b_rl)
            msg.name.append('felis_rr_cpb')
            msg.position.append(self.val_b_rr)
            self.pub.publish(msg)
            self.updateFootPositions()
            try:
                r.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                pass

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

    def updateFootPositions(self):
        fl = self.convertPoint('base_link', 'left_front_quadleg_linkD', [self.link_lengthC, 0, 0.003])
        fr = self.convertPoint('base_link', 'right_front_quadleg_linkD', [self.link_lengthC, 0, 0.003])
        rl = self.convertPoint('base_link', 'left_rear_quadleg_linkD', [self.link_lengthC, 0, 0.003])
        rr = self.convertPoint('base_link', 'right_rear_quadleg_linkD', [self.link_lengthC, 0, 0.003])
        if fl is not None:
            self.FootPositions['FL'] = fl
        if fr is not None:
            self.FootPositions['FR'] = fr
        if rl is not None:
            self.FootPositions['RL'] = rl
        if rr is not None:
            self.FootPositions['RR'] = rr
        self.updateFootPositionsDisplay()

    def updateFootPositionsDisplay(self):
        self.ui.disp_fl_x.setText('X:%9.2f mm' % (self.FootPositions['FL'][0] * 1000.0))
        self.ui.disp_fl_y.setText('Y:%9.2f mm' % (self.FootPositions['FL'][1] * 1000.0))
        self.ui.disp_fl_z.setText('Z:%9.2f mm' % (self.FootPositions['FL'][2] * 1000.0))
        self.ui.disp_fr_x.setText('X:%9.2f mm' % (self.FootPositions['FR'][0] * 1000.0))
        self.ui.disp_fr_y.setText('Y:%9.2f mm' % (self.FootPositions['FR'][1] * 1000.0))
        self.ui.disp_fr_z.setText('Z:%9.2f mm' % (self.FootPositions['FR'][2] * 1000.0))
        self.ui.disp_rl_x.setText('X:%9.2f mm' % (self.FootPositions['RL'][0] * 1000.0))
        self.ui.disp_rl_y.setText('Y:%9.2f mm' % (self.FootPositions['RL'][1] * 1000.0))
        self.ui.disp_rl_z.setText('Z:%9.2f mm' % (self.FootPositions['RL'][2] * 1000.0))
        self.ui.disp_rr_x.setText('X:%9.2f mm' % (self.FootPositions['RR'][0] * 1000.0))
        self.ui.disp_rr_y.setText('Y:%9.2f mm' % (self.FootPositions['RR'][1] * 1000.0))
        self.ui.disp_rr_z.setText('Z:%9.2f mm' % (self.FootPositions['RR'][2] * 1000.0))

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


if __name__ == '__main__':
    try:
        rospy.init_node('felis_cp_simple_publisher_gui')
        app = QApplication(sys.argv)
        gui = FelisConfigurationGui()
        gui.show()
        Thread(target=gui.loop).start()
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        pass