from tempfile import TemporaryFile

import crocoddyl as croco
import example_robot_data
import numpy as np
import pinocchio as pin

# Initialize Pinocchio
robot = example_robot_data.load('solo12')
nq = robot.model.nq
q0 = pin.zero(nq)
nv = robot.model.nv
v0 = pin.zero(nv)
pin.framesForwardKinematics(robot.model, robot.data, q0, v0)

############################################ YOU CAN CHANGE #################################################
# Physical constants
dt = 1e-3 # in [s]
T = 1000 # knots

# Creation state
state = croco.StateMultibody(robot.model)

# Creation Cost
running_cost_model  = croco.CostModelSum(state)
terminal_cost_model = croco.CostModelSum(state)

## Command Regulation Cost
u_res = croco.ResidualModelControl(state)
u_cost = croco.CostModelResidual(state, u_res)
running_cost_model.addCost('uReg', u_cost, 1e-4)
        
## State Cost
x_res = croco.ResidualModelControl(state)
x_cost = croco.CostModelResidual(state, x_res)
running_cost_model.addCost('xReg', x_cost, 1e-4)
              
## Feet Goal Cost
foot1_id  = robot.model.getFrameId('FL_FOOT')
foot1_res = croco.ResidualModelFrameTranslation(state, foot1_id, np.array([0.1946, 0.14695, -0.2]))
foot1_goal_cost = croco.CostModelResidual(state, foot1_res)
running_cost_model.addCost('FL_FOOT', foot1_goal_cost, 1)
terminal_cost_model.addCost('FL_FOOT', foot1_goal_cost, 1)

foot2_id  = robot.model.getFrameId('FR_FOOT')
foot2_res = croco.ResidualModelFrameTranslation(state, foot2_id, np.array([0.1946, -0.14695, -0.2]))
foot2_goal_cost = croco.CostModelResidual(state, foot2_res)
running_cost_model.addCost('FR_FOOT', foot2_goal_cost, 1)
terminal_cost_model.addCost('FR_FOOT', foot2_goal_cost, 1)

foot3_id  = robot.model.getFrameId('HL_FOOT')
foot3_res = croco.ResidualModelFrameTranslation(state, foot3_id, np.array([-0.1946, 0.14695, -0.2, 1]))
foot3_goal_cost = croco.CostModelResidual(state, foot3_res)
running_cost_model.addCost('HL_FOOT', foot3_goal_cost, 1)
terminal_cost_model.addCost('HL_FOOT', foot3_goal_cost, 1)

foot4_id  = robot.model.getFrameId('HR_FOOT')
foot4_res = croco.ResidualModelFrameTranslation(state, foot4_id, np.array([-0.1946, -0.14695, -0.2]))
foot4_goal_cost = croco.CostModelResidual(state, foot4_res)
running_cost_model.addCost('HR_FOOT', foot4_goal_cost, 1)
terminal_cost_model.addCost('HR_FOOT', foot4_goal_cost, 1)
        
## Base Link Goal Cost
frame_goal = pin.SE3(np.eye(3), np.array([.05, .0, -0.1]))
base_link_id  = robot.model.getFrameId('base_link')
base_link_res = croco.ResidualModelFramePlacement(state, base_link_id, frame_goal)
base_link_goal_cost = croco.CostModelResidual(state, base_link_res)
terminal_cost_model.addCost('base_link', base_link_goal_cost, 1)

# Creation Action Model
actuation_model = croco.ActuationModelFull(state)
running_model = croco.IntegratedActionModelEuler(croco.DifferentialActionModelFreeFwdDynamics(state, actuation_model, running_cost_model), dt)
running_model.differential.armature = np.full((nq, 1), 0.1)
      
terminal_model = croco.IntegratedActionModelEuler(croco.DifferentialActionModelFreeFwdDynamics(state, actuation_model, terminal_cost_model), 0.)
terminal_model.differential.armature = np.full((nq, 1), 0.1)

## Solve
x0 = np.concatenate(q0, v0)
problem = croco.ShootingProblem(x0, [running_model]*T, terminal_model)
solver = croco.SolverDDP(problem)
solver.solve()

# Save the trajectory
outfile = TemporaryFile()
np.save(outfile, solver.xs)
