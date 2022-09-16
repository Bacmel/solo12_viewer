#! /usr/bin/env python3
# coding : utf-8
# author : hduarte
# Date   : 07/09/2022

"""Pinocchio publisher node."""

# Exemple robot
import example_robot_data
# ODRI
import libodri_control_interface_pywrap as oci
# Pinocchio
import pinocchio as pin
# ROS
import rclpy
from geometry_msgs.msg import TransformStamped
# Custom ROS messages
from odri_msgs.msg import MotorState, RobotState
from pinocchio.utils import *
from rclpy.node import Node
from sensor_msgs.msg import JointState
# ##MODULES ===========================================================
# ROS messages
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

# 
# ##CONSTANTS =========================================================
# Defaults files
yaml_file = '/home/aschroeter/devel/odri_control_interface/demos/config_solo12.yaml'
trajectory_file = ''

# Physical constants
dt = 1e-3 # in [s]

# ##CLASS ============================================================

class Solo12Viewer(Node):
    """ROS2 Interface for odri and Pinocchio."""

    # Initialisation functions -----------------------------------------
    def __init__(self):
        """Initialize the ROS node."""
        super().__init__('solo12_viewer')
        # Creation of the ROS publishers
        self._init_publisher()
        # Creation of the ROS broadcaster
        self._init_broadcaster()
        # Creation of the ROS timer
        self.timer = self.create_timer(dt, self.loop)
        # Initialization for Aurel
        self.setup()
   
    def _getAngleValue(self, joint_id):
        """Deprecated method."""
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
        """Initialize the ROS publisher."""
        self.pub_joint_state = self.create_publisher(JointState, 'joint_states', 1)
        self.pub_odri_data = self.create_publisher(RobotState, 'odri_data', 1)

    def _init_broadcaster(self):
        """Initialize the ROS broadcaster"""
        self.br_tf = TransformBroadcaster(self)

    def publish_joint_state(self):
        """Update joints regarding Pinocchio"""
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
        self.pub_joint_state.publish(msg_joint_state)

    def broadcast_tf(self):
        """Update frames regarding Pinocchio."""
        for frame in self.pin_robot.model.frames:
            t = TransformStamped()
            
            # Separate joint frame and body frame
            if frame.type == pin.JOINT:
                parent_frame_id = 'universe'
                M_p_c = self.pin_robot.data.oMi[frame.parent]
            else:
                parent_frame_id = self.pin_robot.model.names[frame.parent]
                M_p_c = frame.placement
            if parent_frame_id == frame.name:
                pass
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = parent_frame_id
            t.child_frame_id = frame.name
            
            x, y, z, q_x, q_y, q_z, q_w = pin.SE3ToXYZQUAT(M_p_c)

            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            t.transform.rotation.x = q_x
            t.transform.rotation.y = q_y
            t.transform.rotation.z = q_z
            t.transform.rotation.w = q_w

            self.br_tf.sendTransform(t)

    def publish_odri_data(self):
        """Publish odri state."""
        msg_robot_state = RobotState()
        positions = self.odri_robot.joints.positions.copy()
        velocities = self.odri_robot.joints.velocities.copy()
        torques = self.odri_robot.joints.measured_torques.copy()

        msg_robot_state.header.stamp = self.get_clock().now().to_msg()
        for i, _ in enumerate(positions):
            motor_state = MotorState()
            motor_state.position = positions[i]
            motor_state.velocity = velocities[i]
            motor_state.current = torques[i]

            msg_robot_state.motor_states.append(motor_state)
        self.pub_odri_data.publish(msg_robot_state)

   # Customs functions -------------------------------------------------

    def setup(self):
        # Creation pinocchio
        self.pin_robot = example_robot_data.load('solo12')
        nq = self.pin_robot.model.nq
        q0 = zero(nq)
        nv = self.pin_robot.model.nv
        v0 = zero(nv)
        pin.framesForwardKinematics(self.pin_robot.model, self.pin_robot.data, q0, v0)
        self.broadcast_tf()

        # Creation ODRI
        self.odri_robot = oci.robot_from_yaml_file(yaml_file)
        self.odri_robot.initialize(q0)
        q = self.odri_robot.joints.positions
        self.odri_robot.joints.set_position_offsets(-q)
        
        # Get Trajectory
        self.Xs = np.load(trajectory_file)

        input("Press Enter to launch the trajectory...")

    def loop(self):

        # Get new configuration
        x_t = self.Xs[self.t]
        q = x_t[:self.nq]
        v = x_t[self.nq:]

        # Collecting datas
        #self.odri_robot.parse_sensor_data()

        # imu_attitude = self.odri_robot.imu.attitude_euler
        # positions = self.odri_robot.joints.positions
        # velocities = self.odri_robot.joints.velocities

        # nq = self.pin_robot.model.nq
        # bound = np.full((nq, 1), np.pi)
        # q = positions # pin.randomConfiguration(self.pin_robot.model, -bound, bound)
        pin.framesForwardKinematics(self.pin_robot.model, self.pin_robot.data, q, v)

        #if self.t < len(self.solver.xs)-1:
        #    self.t = self.t+1

        # Send data
        # self.publish_joint_state()
        self.broadcast_tf()
        # self.publish_odri_data()


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



