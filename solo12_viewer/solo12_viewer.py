#! /usr/bin/env python3
# coding : utf-8
# author : hduarte
# Date   : 07/09/2022

"""Pinocchio publisher node."""


# ##MODULES ===========================================================
# ROS messages
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

# ROS
import rclpy
from rclpy.node import Node

# Pinocchio
import pinocchio as pin
from pinocchio.utils import *
from pinocchio.robot_wrapper import RobotWrapper

# ODRI
# import libodri_control_interface_pywrap as oci


# ##CONSTANTS =========================================================
# Defaults files
urdf = '/opt/openrobots/share/example-robot-data/robots/solo_description/robots/solo12.urdf'
yaml = '/home/aschroeter/devel/odri_control_interface/demos/config_solo12.yaml'

# Physical constants
dt = 0.001 # in [s]

# ##CLASS ============================================================

class Solo12Viewer(Node):
    """ROS2 Interface for odri and Pinocchio."""

    # Initialisation functions -----------------------------------------
    def __init__(self):
        """Initialize the ROS node."""
        super().__init__('solo12_viewer')
        # Creation of the ROS publisher
        self._init_publisher()
        # Creation of the ROS timer
        self.timer = self.create_timer(dt, self.loop)
        # Initialization for Aurel
        self.setup()
   
    def _getAngleValue(self, joint_id):
        joint_name = self.pin_robot.model.names[joint_id]
        previous_id = self.pin_robot.model.frames[self.pin_robot.model.getFrameId(joint_name)].previousFrame
        parent_id = self.pin_robot.model.frames[previous_id].parent
        joint_1_name = self.pin_robot.model.names[parent_id]
        joint_1_id = self.pin_robot.index(joint_1_name)
        Moj = self.pin_robot.data.oMi[joint_id]
        Moj_1 = self.pin_robot.data.oMi[joint_1_id]
        Mj_1_j = Moj_1.inverse()*Moj
        quat = pin.Quaternion(Mj_1_j.rotation)
        vec = quat.vec()
        theta = 2*np.arctan2(np.linalg.norm(vec), quat.w)
        theta = (theta + np.pi) % (2 * np.pi) - np.pi
        return np.around(theta, decimals=2)

    def _init_publisher(self):
        """Initialize the ROS broadcaster."""
        # Data from pinocchio
        self.pub_state = self.create_publisher(JointState, 'joint_states', 1) 
        
    def update_rviz(self):
        """Update RVIZ regarding Pinocchio"""
        msg_joint_state = JointState()
        msg_joint_state.header = Header()
        msg_joint_state.header.stamp = self.get_clock().now().to_msg()
        msg_joint_state.header.frame_id = 'base_link'
        for joint in self.pin_robot.model.joints:
            if joint.id < self.pin_robot.model.njoints:
                msg_joint_state.name.append(self.pin_robot.model.names[joint.id])
                msg_joint_state.position.append(self._getAngleValue(joint.id))
                msg_joint_state.velocity.append(0)
                msg_joint_state.effort.append(0)
        self.pub_state.publish(msg_joint_state)


   # Customs functions -------------------------------------------------

    def setup(self):
        # Creation of the 2 entities
        self.pin_robot = RobotWrapper.BuildFromURDF(urdf)
        #self.odri_robot = oci.robot_from_yaml_file(yaml)
        
        # Initialization
        nq = self.pin_robot.model.nq
        q = zero(nq)

        pin.framesForwardKinematics(self.pin_robot.model, self.pin_robot.data, q)
        #self.odri_robot.initialize(q)

    def loop(self):
        # Collecting datas
        # odri_robot.parse_sensor_data()

        # imu_attitude = odri_robot.imu.attitude_euler
        # positions = odri_robot.joints.positions
        # velocities = odri_robot.joints.velocities

        nq = self.pin_robot.model.nq
        bound = np.full((nq, 1), np.pi)
        q = pin.randomConfiguration(self.pin_robot.model, -bound, bound)
        pin.framesForwardKinematics(self.pin_robot.model, self.pin_robot.data, q)

        # Send data
        self.update_rviz()


# ## MAIN ================================================================
def main(args=None):
   """Define the main function."""
   rclpy.init(args=args)

   solo12_viewer = Solo12Viewer()
   rclpy.spin(solo12_viewer)

   solo12_viewer.destroy_node()
   rclpy.shutdown()

if __name__ == '__main__':
   main()



