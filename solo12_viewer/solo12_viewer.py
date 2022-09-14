#! /usr/bin/env python3
# coding : utf-8
# author : hduarte
# Date   : 07/09/2022

"""Pinocchio publisher node."""


# ##MODULES ===========================================================
# ROS messages
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped

# Custom ROS messages
from odri_msgs.msg import RobotState
from odri_msgs.msg import MotorState

# ROS
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

# Pinocchio
import pinocchio as pin
from pinocchio.utils import *
from pinocchio.robot_wrapper import RobotWrapper

# ODRI
import libodri_control_interface_pywrap as oci

# Crocoddyl
import crocoddyl


# ##CONSTANTS =========================================================
# Defaults files
urdf = '/opt/openrobots/share/example-robot-data/robots/solo_description/robots/solo12.urdf'
yaml = '/home/aschroeter/devel/odri_control_interface/demos/config_solo12.yaml'

# Goal frame
frame_name = 'base_link'
frame_goal = pin.SE3(np.eye(3), np.array([.05, .0, -0.1]))

# Physical constants
dt = 1e-3 # in [s]
T = 1000 # knots

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
        self.pub_odri = self.create_publisher(RobotState, 'solo12_states', 1)
        self.br_state = TransformBroadcaster(self)

    def update_joint(self):
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
        self.pub_state.publish(msg_joint_state)

    def update_frame(self):
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
                parent_frame_id = 'map'
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

            self.br_state.sendTransform(t)

    def publish_odri_state(self):
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
        self.pub_odri.publish(msg_robot_state)


   # Customs functions -------------------------------------------------
    
    def crocoddyl_setup(self):
        # Creation state
        state = crocoddyl.StateMultibody(self.pin_robot.model)

        # Creation Cost
        runningCM  = crocoddyl.CostModelSum(state)
        terminalCM = crocoddyl.CostModelSum(state)

        ## Command Cost
        uRes = crocoddyl.ResidualModelControl(state)
        uCost = crocoddyl.CostModelResidual(state, uRes)
        runningCM.addCost('uReg', uCost, 1e-4)
        
        ## State Cost
        xRes = crocoddyl.ResidualModelControl(state)
        xCost = crocoddyl.CostModelResidual(state, xRes)
        runningCM.addCost('xReg', xCost, 1e-4)
        
        
        #
        frameid = self.pin_robot.model.getFrameId('FL_FOOT')
        frameRes = crocoddyl.ResidualModelFrameTranslation(state, frameid, np.array([0.1946, 0.14695, -0.2]))
        goalCost = crocoddyl.CostModelResidual(state, frameRes)
        runningCM.addCost('FL_FOOT', goalCost, 1)
        terminalCM.addCost('FL_FOOT', goalCost, 1)
        
        frameid = self.pin_robot.model.getFrameId('FR_FOOT')
        frameRes = crocoddyl.ResidualModelFrameTranslation(state, frameid, np.array([0.1946, -0.14695, -0.3]))
        goalCost = crocoddyl.CostModelResidual(state, frameRes)
        runningCM.addCost('FR_FOOT', goalCost, 1)
        terminalCM.addCost('FR_FOOT', goalCost, 1)

        frameid = self.pin_robot.model.getFrameId('HL_FOOT')
        frameRes = crocoddyl.ResidualModelFrameTranslation(state, frameid, np.array([-0.1946, 0.14695, -0.2]))
        goalCost = crocoddyl.CostModelResidual(state, frameRes)
        runningCM.addCost('HL_FOOT', goalCost, 1)
        terminalCM.addCost('HL_FOOT', goalCost, 1)

        frameid = self.pin_robot.model.getFrameId('HR_FOOT')
        frameRes = crocoddyl.ResidualModelFrameTranslation(state, frameid, np.array([-0.1946, -0.14695, -0.2]))
        goalCost = crocoddyl.CostModelResidual(state, frameRes)
        runningCM.addCost('HR_FOOT', goalCost, 1)
        terminalCM.addCost('HR_FOOT', goalCost, 1)
        
        
        
        ## Goal Cost
        #frameid = self.pin_robot.model.getFrameId(frame_name)
        #frameRes = crocoddyl.ResidualModelFramePlacement(state, frameid, frame_goal)
        #goalCost = crocoddyl.CostModelResidual(state, frameRes)
        #runningCM.addCost(frame_name, goalCost, 1)
        #terminalCM.addCost(frame_name, goalCost, 1)

        # Creation Action Model
        actuationM = crocoddyl.ActuationModelFull(state)
        self.runningM = crocoddyl.IntegratedActionModelEuler(crocoddyl.DifferentialActionModelFreeFwdDynamics(state, actuationM, runningCM), dt)
        self.runningM.differential.armature = np.full((self.nq, 1), 0.1)
        
        self.terminalM = crocoddyl.IntegratedActionModelEuler(crocoddyl.DifferentialActionModelFreeFwdDynamics(state, actuationM, terminalCM), 0.)
        self.terminalM.differential.armature = np.full((self.nq, 1), 0.1)

    def setup(self):
        # Creation of the 2 entities
        self.pin_robot = RobotWrapper.BuildFromURDF(urdf)
        self.odri_robot = oci.robot_from_yaml_file(yaml)
        
        # Initialization
        self.nq = self.pin_robot.model.nq
        q0 = zero(self.nq)
        nv = self.pin_robot.model.nv
        v0 = zero(nv)
        x0 = np.concatenate([q0, v0])
        
        ## Pinocchio
        pin.framesForwardKinematics(self.pin_robot.model, self.pin_robot.data, q0)

        ## ODRI
        self.odri_robot.initialize(q0)
        q = self.odri_robot.joints.positions
        self.odri_robot.joints.set_position_offsets(-q)

        ## Crocoddyl
        self.crocoddyl_setup()
        problem = crocoddyl.ShootingProblem(x0, [self.runningM]*T, self.terminalM)
        self.solver = crocoddyl.SolverDDP(problem)
        self.solver.solve()
        self.t = 0
        
    def loop(self):
        # Get new configuration
        x_t = self.solver.xs[self.t]
        q = x_t[:self.nq]
        v = x_t[self.nq:]

        # Collecting datas
        self.odri_robot.parse_sensor_data()

        imu_attitude = self.odri_robot.imu.attitude_euler
        positions = self.odri_robot.joints.positions
        velocities = self.odri_robot.joints.velocities

        #nq = self.pin_robot.model.nq
        #bound = np.full((nq, 1), np.pi)
        #q = positions # pin.randomConfiguration(self.pin_robot.model, -bound, bound)
        pin.framesForwardKinematics(self.pin_robot.model, self.pin_robot.data, q)

        if self.t < len(self.solver.xs)-1:
            self.t = self.t+1

        # Send data
        # self.update_joint()
        self.update_frame()
        self.publish_odri_state()


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



