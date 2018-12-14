#!/usr/bin/env python

import argparse
import importlib

import rospy
from dynamic_reconfigure.server import Server
from std_msgs.msg import Empty

import intera_interface
from intera_interface import CHECK_VERSION

class JointSprings(object):

    def __init__(self, reconfig_server, limb = "right"):
        self._dyn = reconfig_server

        self._limb = intera_interface.Limb(limb)

        self._springs = dict()
        self._damping = dict()
        self._start_angles = dict()

        cuff_ns = 'robot/limb/' + limb + '/suppress_cuff_interaction'
        self._pub_cuff_disable = rospy.Publisher(cuff_ns, Empty, queue_size=1)

        self._rs = intera_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        self._rs.enable()

    def _update_parameters(self):
        for joint in self._limb.joint_names():

            # Set custom stiffness and damping coefficient
            self._springs[joint] = 0.01
            self._damping[joint] = 100

    def _update_forces(self):
        
        # Calculate the force and joint torque
        self._update_parameters()

        self._pub_cuff_disable.publish()

        cmd = dict()
        cur_pos = self._limb.joint_angles()
        cur_vel = self._limb.joint_velocities()
        for joint in self._start_angles.keys():
            cmd[joint] = self._springs[joint] * (self._start_angles[joint] -
                                                   cur_pos[joint])
            cmd[joint] -= self._damping[joint] * cur_vel[joint]
        self._limb.set_joint_torques(cmd)

    def enter_impedance(self):

        self._start_angles = self._limb.joint_angles()

        while not rospy.is_shutdown():
            if not self._rs.state().enabled:
                rospy.logerr("Joint torque example failed to meet "
                             "specified control rate timeout.")
                break
            self._update_forces()


def main():

    # Enters impedance control mode
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    robot_name = intera_interface.RobotParams().get_robot_name().lower().capitalize()
    arg_fmt = argparse.ArgumentDefaultsHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt)
    parser.add_argument(
        "-l", "--limb", dest="limb", default=valid_limbs[0],
        choices=valid_limbs,
        help='limb on which to attach joint springs'
        )
    args = parser.parse_args(rospy.myargv()[1:])
    config_name = ''.join([robot_name,"JointSpringsExampleConfig"])
    config_module = "intera_examples.cfg"
    cfg = importlib.import_module('.'.join([config_module,config_name]))
    rospy.init_node("sdk_joint_torque_springs_{0}".format(args.limb))
    dynamic_cfg_srv = Server(cfg, lambda config, level: config)
    js = JointSprings(dynamic_cfg_srv, limb=args.limb)
    js.enter_impedance()


if __name__ == "__main__":
    main()