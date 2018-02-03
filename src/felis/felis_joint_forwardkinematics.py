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
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value


######## Joint State Publisher ###########
class FelisJointStatePublisher():
    def __init__(self):
        self.link_length = 0.07
        self.gui = None

        limits = {'a_min':0, 'a_max':179, 'b_min':3, 'b_max': (self.link_length*1000)-3}
        #TODO stuff
        use_gui = get_param("use_gui", True)
        if use_gui:
            self.app = QApplication(sys.argv)
            self.gui = FelisConfigurationGui(self, limits)
            self.gui.show()
        else:
            self.gui = None

    def loop(self):
        hz = get_param("rate", 10)
        r = rospy.Rate(hz)
        delta = get_param("delta", 0.0 )

        # Do other stuff


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
        self.set_fl_a(90)
        self.set_fr_a(90)
        self.set_rl_a(90)
        self.set_rr_a(90)
        self.set_fl_b(35)
        self.set_fr_b(35)
        self.set_rl_b(35)
        self.set_rr_b(35)
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
        print 'Control Parameter A FL Updated'

    def val_fr_a_updated(self):
        print 'Control Parameter A FR Updated'

    def val_rl_a_updated(self):
        print 'Control Parameter A RL Updated'

    def val_rr_a_updated(self):
        print 'Control Parameter A RR Updated'

    def val_fl_b_updated(self):
        print 'Control Parameter B FL Updated'

    def val_fr_b_updated(self):
        print 'Control Parameter B FR Updated'

    def val_rl_b_updated(self):
        print 'Control Parameter B RL Updated'

    def val_rr_b_updated(self):
        print 'Control Parameter B RR Updated'


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
            Thread(target=jsp.loop).start()
            signal.signal(signal.SIGINT, signal.SIG_DFL)
            sys.exit(jsp.app.exec_())
    except rospy.ROSInterruptException:
        pass
