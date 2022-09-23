#! /usr/bin/env python3
# coding : utf-8
# author : hduarte
# Date   : 07/09/2022

"""Pinocchio publisher node."""

# ##MODULES ===========================================================
# Exemple robot
import example_robot_data
# ODRI
import libodri_control_interface_pywrap as oci
# Pinocchio
import pinocchio as pin
# ROS
import rclpy
# ROS messages
from geometry_msgs.msg import TransformStamped
# Custom ROS messages
from odri_msgs.msg import MotorState, RobotState
from pinocchio.utils import *
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
# ROS service
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster

# ##CONSTANTS =========================================================
# Physical constants
DEFAULT_DT = 1e-3 # in [s]

# ##CLASS ============================================================

class Solo12Viewer(Node):
    """ROS2 Interface for odri and Pinocchio."""

    # Initialisation functions -----------------------------------------
    def __init__(self):
        """Initialize the ROS node."""
        super().__init__('solo12_viewer')
        # Creation of ROS parameters
        self._init_parameters()
        # Creation of the ROS publishers
        self._init_publishers()
        # Creation of the ROS broadcaster
        self._init_broadcaster()
        # Creation of the ROS service
        self._init_service()
        # Creation of the ROS timer
        self.timer = self.create_timer(self.dt, self.loop)
        # Initialization for Aurel
        self.setup()
    
    def _init_parameters(self):
        """Initialize the ROS parameters."""
        # odri parameters
        self.is_odri_enabled = self.declare_parameter('is_odri_enabled', False).value
        self.odri_config_file = self.declare_parameter('odri_config_file', '').value
        # trajectory parameters
        self.dt = self.declare_parameter('dt', DEFAULT_DT).value
        self.trajectory_file = self.declare_parameter('trajectory_file', '').value

    def _init_publishers(self):
        """Initialize the ROS publishers."""
        self.pub_joint_state = self.create_publisher(JointState, 'joint_states', 1)
        if self.is_odri_enabled:
            self.pub_odri_data = self.create_publisher(RobotState, 'odri_data', 1)

    def _init_broadcaster(self):
        """Initialize the ROS broadcaster."""
        self.br_tf = TransformBroadcaster(self)

    def _init_service(self):
        """Initialize the ROS service."""
        self.srv_enable = self.create_service(SetBool, 'enable_loop', self.enable_loop_callback)

    def publish_joint_state(self):
        """Update joints regarding Pinocchio"""
        msg_joint_state = JointState()
        msg_joint_state.header = Header()
        msg_joint_state.header.stamp = self.get_clock().now().to_msg()
        msg_joint_state.header.frame_id = 'base_link'
        for joint in self.pin_robot.model.joints:
            if joint.id < self.pin_robot.model.njoints:
                msg_joint_state.name.append(self.pin_robot.model.names[joint.id])
                msg_joint_state.position.append(self.getAngleValue(joint.id))
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
        if self.is_odri_enabled:
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

    def enable_loop_callback(self, request, response):
        """Response to the request"""
        self.loop_enabled = request.data
        response.success = True
        if self.loop_enabled:
            response.message = 'Loop enabled.'
        else:
            response.message = 'Loop disabled.'
        return response


   # Custom functions -------------------------------------------------

    def getAngleValue(self, joint_id):
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


    def set_odri(self, q, v, t, kp, kd):
        if self.is_odri_enabled:
            self.odri_robot.joints.set_desired_positions(q)
            self.odri_robot.joints.set_desired_velocities(v)
            self.odri_robot.joints.set_torques(t)
            self.odri_robot.joints.set_position_gains(np.full((len(q), 1), kp))
            self.odri_robot.joints.set_velocity_gains(np.full((len(v), 1), kd))
            self.odri_robot.send_command_and_wait_end_of_cycle(self.dt)

   # Core functions -------------------------------------------------

    def setup(self):
        # Creation pinocchio
        self.pin_robot = example_robot_data.load('solo12')
        self.nq = self.pin_robot.model.nq
        q0 = self.pin_robot.q0
        nv = self.pin_robot.model.nv
        v0 = self.pin_robot.v0
        pin.framesForwardKinematics(self.pin_robot.model, self.pin_robot.data, q0)
        self.broadcast_tf()

        # Creation ODRI
        if self.is_odri_enabled:
            self.odri_robot = oci.robot_from_yaml_file(self.odri_config_file)
            self.odri_robot.initialize(q0[7:])
            self.odri_robot.parse_sensor_data()
            self.publish_odri_data()
        
        # Get Trajectory
        S = np.load(self.trajectory_file)
        self.Xs = S['xs']
        self.Us = S['us']
        self.t = 0

        # Start
        self.loop_enabled = False
        
    def loop(self):
        # Get new configuration
        x_t = self.Xs[self.t]
        # print(self.t)
        if self.t == 0:
            u_t_1 = np.zeros(12)
        else:
            u_t_1 = self.Us[int(self.t-1)]
            
        q = x_t[:self.nq]
        v = x_t[self.nq:]
        t = u_t_1

        # Odri
        if self.is_odri_enabled:
            self.odri_robot.parse_sensor_data()
            self.set_odri(q[7:], v[6:], t, 6, 0.3)
            self.publish_odri_data()

        # Pinocchio
        pin.framesForwardKinematics(self.pin_robot.model, self.pin_robot.data, q)
        # self.publish_joint_state()
        self.broadcast_tf()

        if self.t < len(self.Xs)-1 and self.loop_enabled:
           self.t = self.t+1


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

