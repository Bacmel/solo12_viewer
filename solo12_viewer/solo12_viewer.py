#! /usr/bin/env python3
# coding : utf-8
# author : hduarte
# Date   : 07/09/2022

"""Pinocchio publisher node."""

# ##MODULES ===========================================================
# Exemple robot
import example_robot_data

# Pinocchio
import pinocchio as pin
from pinocchio.utils import *

# ROS
import rclpy
from rclpy.node import Node

# ROS messages
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# Custom ROS messages
from odri_msgs.msg import MotorCommand, RobotCommand

# ROS service
from std_srvs.srv import SetBool

# ##CONSTANTS =========================================================
# Physical constants
DEFAULT_DT = 10e-3 # in [s]

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
        self._init_publisher()
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
        # trajectory parameters
        self.dt = self.declare_parameter('dt', DEFAULT_DT).value
        self.trajectory_file = self.declare_parameter('trajectory_file', '').value

    def _init_publisher(self):
        """Initialize the ROS publishers."""
        if self.is_odri_enabled:
            self.pub_solo_command = self.create_publisher(RobotCommand, 'robot_command', 1)
    
    def _init_broadcaster(self):
        """Initialize the ROS broadcaster."""
        self.br_tf = TransformBroadcaster(self)

    def _init_service(self):
        """Initialize the ROS service."""
        self.srv_enable = self.create_service(SetBool, 'enable_loop', self.enable_loop_callback)

    def broadcast_tf(self):
        """Update frames regarding Pinocchio."""
        for frame in self.robot.model.frames:
            t = TransformStamped()
            
            # Separate joint frame and body frame
            if frame.type == pin.JOINT:
                parent_frame_id = 'universe'
                M_p_c = self.robot.data.oMi[frame.parent]
            else:
                parent_frame_id = self.robot.model.names[frame.parent]
                M_p_c = frame.placement
            if parent_frame_id == frame.name:
                continue
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

    def publish_solo_command(self, q, v, t, Kp=6.0, Kv=0.3, i_sat=8):
        """Publish odri command."""
        if self.is_odri_enabled:
            msg_robot_command = RobotCommand()
            msg_robot_command.header.stamp = self.get_clock().now().to_msg()

            for i, _ in enumerate(q):
                motor_command = MotorCommand()
                motor_command.position_ref = q[i]
                motor_command.velocity_ref = v[i]
                motor_command.current_ref = t[i]
                motor_command.kp = float(Kp)
                motor_command.kd = float(Kv)
                motor_command.i_sat = float(i_sat)

                msg_robot_command.motor_commands.append(motor_command)
            self.pub_solo_command.publish(msg_robot_command)

    def enable_loop_callback(self, request, response):
        """Response to the request"""
        self.loop_enabled = request.data
        response.success = True
        if self.loop_enabled:
            response.message = 'Loop enabled.'
        else:
            response.message = 'Loop disabled.'
        return response

   # Core functions -------------------------------------------------

    def setup(self):
        # Creation pinocchio
        self.robot = example_robot_data.load('solo12')
        self.nq = self.robot.nq
        q0 = self.robot.q0
        pin.framesForwardKinematics(self.robot.model, self.robot.data, q0)

        # Get Trajectory
        S = np.load(self.trajectory_file)
        self.Xs = S['xs']
        self.Us = S['us']
        self.t = 0

        # Start
        self.loop_enabled = False
        self.Kp = 6
        self.Kv = 0.3
        
    def loop(self):
        # Get new configuration
        x_t = self.Xs[self.t]
        if self.t == 0:
            u_t_1 = np.zeros(12)
        else:
            u_t_1 = self.Us[self.t-1]
            
        q = x_t[:self.nq]
        v = x_t[self.nq:]
        t = u_t_1

        # Odri
        if self.is_odri_enabled:
            self.publish_solo_command(q[7:], v[6:], t, 5, 0.3) #self.Kp, self.Kv) # pos vel torque Kp Kv

        # Pinocchio
        pin.framesForwardKinematics(self.robot.model, self.robot.data, q)
        self.broadcast_tf()

        if self.loop_enabled:
            if self.t < len(self.Xs)-1:
                self.t = self.t+1
            else:
                pass
                #self.Kp = 6
                #self.Kv = 0.3


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

