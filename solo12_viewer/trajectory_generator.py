import crocoddyl as croco
import example_robot_data
import numpy as np
import pinocchio as pin
from pinocchio.utils import *


## LISTS OF DATA
feet_name_list = ['FL_FOOT', 'FR_FOOT', 'HL_FOOT', 'HR_FOOT']
feet_pos_list = [np.array([0.1946, 0.14695, -0.2]),
                 np.array([0.1946, -0.14695, -0.2]),
                 np.array([-0.1946, 0.14695, -0.2]),
                 np.array([-0.1946, -0.14695, -0.2])]

## Initialize Pinocchio
robot = example_robot_data.load('solo12')
nq = robot.nq
q0 = robot.q0
nv = robot.nv
v0 = robot.v0
x0 = np.concatenate([q0, v0])
pin.framesForwardKinematics(robot.model, robot.data, q0)

## Physical constants
dt = 1e-3 # in [s]

T = 1000 # knots

## Creation state / actuation
state = croco.StateMultibody(robot.model)
actuation = croco.ActuationModelFull(state)

actions_model = []

########################################## COST

## Command Regulation Cost
u_res = croco.ResidualModelControl(state)
u_cost = croco.CostModelResidual(state, u_res)
        
## State Regulation Cost 
x_res = croco.ResidualModelState(state, x0)
x_cost = croco.CostModelResidual(state, x_res)


def feet_contact_cost(contact_model, cost_model):
    motion_zero = pin.Motion(np.array([.0, .0, .0]), np.array([.0, .0, .0]))
    for idx, foot_name in enumerate(feet_name_list):
        foot_id = robot.model.getFrameId(foot_name)
        foot_pos = feet_pos_list[idx]
        foot_contact = croco.ContactModel3D(state, foot_id, foot_pos)
        contact_model.addContact(foot_name, foot_contact)
        foot_pos_res = croco.ResidualModelFrameTranslation(state, foot_id, foot_pos)
        foot_pos_cost = croco.CostModelResidual(state, foot_pos_res)
        cost_model.addCost(foot_name+' position', foot_pos_cost, 1)
        foot_vel_res = croco.ResidualModelFrameVelocity(state, foot_id, motion_zero, type=pin.LOCAL)
        foot_vel_cost = croco.CostModelResidual(state, foot_vel_res)
        cost_model.addCost(foot_name+' velocity', foot_vel_cost, 1)


###################################################################################### Action 1 Preparation
action_1_T = int(1/dt)

## COSTS
action_1_cost_model  = croco.CostModelSum(state)

# Reg cost
action_1_cost_model.addCost('uReg', u_cost, 1e-4)
action_1_cost_model.addCost('xReg', x_cost, 1e-4)

## CONTACTS
action_1_contact_model = croco.ContactModelMultiple(state)
# Feet Contact & Cost
feet_contact_cost(action_1_contact_model, action_1_cost_model)
## Integrated Action
action_1_dif_model = croco.DifferentialActionModelContactFwdDynamics(state, actuation, action_1_contact_model, action_1_cost_model)
action_1_model = croco.IntegratedActionModelEuler(action_1_dif_model, dt)

actions_model = actions_model + [action_1_model]*action_1_T


###################################################################################### Action 2 Takeoff
action_2_T = int(0.025/dt)

## COSTS
action_2_cost_model  = croco.CostModelSum(state)

# Reg cost
action_2_cost_model.addCost('uReg', u_cost, 1e-4)
action_2_cost_model.addCost('xReg', x_cost, 1e-4)

# Base Link Goal Cost
motion_goal = pin.Motion(np.array([.0, .0, 2]), np.array([.0, .0, .0]))
base_link_id  = robot.model.getFrameId('base_link')
base_link_res = croco.ResidualModelFrameVelocity(state, base_link_id, motion_goal, type=pin.WORLD)
base_link_goal_cost = croco.CostModelResidual(state, base_link_res)
action_2_cost_model.addCost('base_link', base_link_goal_cost, 100)

## CONTACTS
action_2_contact_model = croco.ContactModelMultiple(state)
# Feet Contact & Cost
feet_contact_cost(action_2_contact_model, action_2_cost_model)
## Integrated Action
action_2_dif_model = croco.DifferentialActionModelContactFwdDynamics(state, actuation, action_2_contact_model, action_2_cost_model)
action_2_model = croco.IntegratedActionModelEuler(action_2_dif_model, dt)

actions_model = actions_model + [action_2_model]*action_2_T


###################################################################################### Action 3 Flying
action_3_T = int(1/dt)

## COSTS
action_3_cost_model  = croco.CostModelSum(state)

# Reg cost
action_3_cost_model.addCost('uReg', u_cost, 1e-4)
action_3_cost_model.addCost('xReg', x_cost, 1e-4)

## Integrated Action
action_3_dif_model = croco.DifferentialActionModelFreeFwdDynamics(state, actuation, action_3_cost_model)
action_3_model = croco.IntegratedActionModelEuler(action_3_dif_model, dt)

actions_model = actions_model + [action_3_model]*action_3_T

###################################################################################### Action 4 Landing
action_4_T = int(1/dt)

## COSTS
action_4_cost_model  = croco.CostModelSum(state)

# Reg cost
action_4_cost_model.addCost('uReg', u_cost, 1e-4)
action_4_cost_model.addCost('xReg', x_cost, 1e-4)

## CONTACTS
action_4_contact_model = croco.ContactModelMultiple(state)
# Feet Contact & Cost
feet_contact_cost(action_4_contact_model, action_4_cost_model)
## Integrated Action
action_4_dif_model = croco.DifferentialActionModelContactFwdDynamics(state, actuation, action_4_contact_model, action_4_cost_model)
action_4_model = croco.IntegratedActionModelEuler(action_4_dif_model, dt)

actions_model = actions_model + [action_4_model]*action_4_T

###################################################################################### Action 5 Rest
action_5_T = int(0.025/dt)

## COSTS
action_5_cost_model  = croco.CostModelSum(state)

# Reg cost
action_5_cost_model.addCost('uReg', u_cost, 1e-4)
action_5_cost_model.addCost('xReg', x_cost, 1e-1)

## CONTACTS
action_5_contact_model = croco.ContactModelMultiple(state)
# Feet Contact & Cost
feet_contact_cost(action_5_contact_model, action_5_cost_model)
## Integrated Action
action_5_dif_model = croco.DifferentialActionModelContactFwdDynamics(state, actuation, action_5_contact_model, action_5_cost_model)
action_5_model = croco.IntegratedActionModelEuler(action_5_dif_model, dt)

actions_model = actions_model + [action_5_model]*action_5_T

###################################################################################### Action Terminal
## COSTS
action_terminal_cost_model  = croco.CostModelSum(state)

# Reg cost
action_terminal_cost_model.addCost('uReg', u_cost, 1e-4)
action_terminal_cost_model.addCost('xReg', x_cost, 1e-1)

## CONTACTS
action_terminal_contact_model = croco.ContactModelMultiple(state)
# Feet Contact & Cost
feet_contact_cost(action_terminal_contact_model, action_terminal_cost_model)
## Integrated Action
action_terminal_dif_model = croco.DifferentialActionModelContactFwdDynamics(state, actuation, action_terminal_contact_model, action_terminal_cost_model)
action_terminal_model = croco.IntegratedActionModelEuler(action_terminal_dif_model, 0)


## Solve
problem = croco.ShootingProblem(x0, actions_model, action_terminal_model)
solver = croco.SolverFDDP(problem)
solver.solve()

# Save the trajectory
np.save('ressource/jump_move.npy', solver.xs)
