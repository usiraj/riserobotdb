#!/usr/bin/env python
__author__ = 'Usama Siraj'
''' Joint State Publisher Based on Control Param A, B'''


import rospy, sys, math
from FelisKinematics import FelisRobotv1Joints
import xml.dom.minidom
from sensor_msgs.msg import JointState


def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        print 'has param'
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value


class FelisJointStatePublisher():
    def __init__(self):
        description = get_param('robot_description')
        if description is None:
            raise EnvironmentError('No robot description found on parameter server, check param remapping before'
                                   'starting the application')
            sys.exit(-1)
        self.tfprefix = get_param('tf_prefix')
        if self.tfprefix is None:
            raise EnvironmentError('tf_prefix must be set in parameter server')
            sys.exit(-1)
        self.robot = xml.dom.minidom.parseString(description).getElementsByTagName('robot')[0]
        rospy.loginfo('Loaded URDF for felis')
        self.link_length = self.get_link_length()
        rospy.loginfo('Link length loaded from urdf : %f', self.link_length)
        self.joints = {}
        for jnt in self.get_all_joint_names():
            self.joints[jnt] = {'value': 0.0, 'velocity': None, 'effort': None}
            _info = self.get_joint_info(jnt)
            self.joints[jnt]['max_effort'] = _info[0]
            self.joints[jnt]['max_velocity'] = _info[1]
            self.joints[jnt]['lower_val'] = _info[2]
            self.joints[jnt]['upper_val'] = _info[3]
        rospy.loginfo('Loaded Joint Constraints')
        self.rkinFL = FelisRobotv1Joints(self.link_length)
        self.rkinFR = FelisRobotv1Joints(self.link_length)
        self.rkinRL = FelisRobotv1Joints(self.link_length)
        self.rkinRR = FelisRobotv1Joints(self.link_length)
        #### Set Joint Names ####
        self.rkinFL.set_jointNames('left_front_linkA_joint', 'left_front_linkB_joint', 'left_front_linkC_joint',
                                   'left_front_linkD_joint', 'left_front_linkE_joint', 'left_front_linkF_joint')
        self.rkinFR.set_jointNames('right_front_linkA_joint', 'right_front_linkB_joint', 'right_front_linkC_joint',
                                   'right_front_linkD_joint', 'right_front_linkE_joint', 'right_front_linkF_joint')
        self.rkinRL.set_jointNames('left_rear_linkA_joint', 'left_rear_linkB_joint', 'left_rear_linkC_joint',
                                   'left_rear_linkD_joint', 'left_rear_linkE_joint', 'left_rear_linkF_joint')
        self.rkinRR.set_jointNames('right_rear_linkA_joint', 'right_rear_linkB_joint', 'right_rear_linkC_joint',
                                   'right_rear_linkD_joint', 'right_rear_linkE_joint', 'right_rear_linkF_joint')
        ###### Initial Starting Values ######
        self.rkinFL.updateControlParams(150, 0.035)
        self.rkinFR.updateControlParams(150, 0.035)
        self.rkinRL.updateControlParams(150, 0.035)
        self.rkinRR.updateControlParams(150, 0.035)
        ######### Dummy Initial Publish #####
        self.rkinFL.syncJointInfo(self.joints)
        self.rkinFR.syncJointInfo(self.joints)
        self.rkinRL.syncJointInfo(self.joints)
        self.rkinRR.syncJointInfo(self.joints)
        ##### Subscribe for joint info #######
        sub = rospy.Subscriber('felis_control_params', JointState, self.cpupdate)
        ########### Publish Joints ###########
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=5)

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

    def cpupdate(self, msg):
        if len(msg.name) > 0:
            for _i in range(len(msg.name)):
                if _i > len(msg.position):
                    continue
                if msg.name[_i] == 'felis_fl_cpa':
                    self.rkinFL.updateControlParamA(msg.position[_i])
                elif msg.name[_i] == 'felis_fl_cpb':
                    self.rkinFL.updateControlParamB(msg.position[_i]/float(1000))
                elif msg.name[_i] == 'felis_fr_cpa':
                    self.rkinFR.updateControlParamA(msg.position[_i])
                elif msg.name[_i] == 'felis_fr_cpb':
                    self.rkinFR.updateControlParamB(msg.position[_i] / float(1000))
                elif msg.name[_i] == 'felis_rl_cpa':
                    self.rkinRL.updateControlParamA(msg.position[_i])
                elif msg.name[_i] == 'felis_rl_cpb':
                    self.rkinRL.updateControlParamB(msg.position[_i] / float(1000))
                elif msg.name[_i] == 'felis_rr_cpa':
                    self.rkinRR.updateControlParamA(msg.position[_i])
                elif msg.name[_i] == 'felis_rr_cpb':
                    self.rkinRR.updateControlParamB(msg.position[_i] / float(1000))
            ########### Update ##########
            self.rkinFL.syncJointInfo(self.joints)
            self.rkinFR.syncJointInfo(self.joints)
            self.rkinRL.syncJointInfo(self.joints)
            self.rkinRR.syncJointInfo(self.joints)

    def loop(self):
        hz = get_param("rate")
        r = rospy.Rate(hz)
        while not rospy.is_shutdown():
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = []
            msg.position = []
            msg.velocity = []   #######todo If decided to incorporate in future
            msg.effort = []     #######todo If decided to incorporate in future
            # Initialize msg.position, msg.velocity and msg.effort
            for joint in self.joints:
                msg.name.append(joint)
                msg.position.append(math.radians(self.joints[joint]['value']))
            if msg.name or msg.position or msg.velocity or msg.effort:
                # Only publish non-empty messages
                self.pub.publish(msg)
            try:
                r.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                pass


if __name__=='__main__':
    try:
        rospy.init_node('felis_joint_state_publisher')
        jsp = FelisJointStatePublisher()
        jsp.loop()
    except rospy.ROSInterruptException:
        pass
