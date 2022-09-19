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

## Creation Cost
running_cost_model  = croco.CostModelSum(state)
terminal_cost_model = croco.CostModelSum(state)

## Command Regulation Cost
u_res = croco.ResidualModelControl(state)
u_cost = croco.CostModelResidual(state, u_res)
        
## State Regulation Cost 
x_res = croco.ResidualModelState(state, x0)
x_cost = croco.CostModelResidual(state, x_res)


###################################################################################### Action 1 Preparation 5s
action_1_T = int(2/dt)
print("action_1_T: ", action_1_T)
## COSTS
action_1_cost_model  = croco.CostModelSum(state)
# Base Link Goal Cost
frame_goal = pin.SE3(rpyToMatrix(np.array([0, 0, 0])), np.array([.0, .0, 0.1]))
base_link_id  = robot.model.getFrameId('base_link')
base_link_res = croco.ResidualModelFramePlacement(state, base_link_id, frame_goal)
base_link_goal_cost = croco.CostModelResidual(state, base_link_res)
action_1_cost_model.addCost('base_link', base_link_goal_cost, 10)
# Reg cost
action_1_cost_model.addCost('uReg', u_cost, 1e-4)
action_1_cost_model.addCost('xReg', x_cost, 1e-3)
## CONTACTS
action_1_contact_model = croco.ContactModelMultiple(state)
# Feet Contact         
for idx, foot_name in enumerate(feet_name_list) :
    foot_id = robot.model.getFrameId(foot_name)
    foot_pos = feet_pos_list[idx]
    foot_contact = croco.ContactModel3D(state, foot_id, foot_pos)
    action_1_contact_model.addContact(foot_name, foot_contact)
## Integrated Action
action_1_dif_model = croco.DifferentialActionModelContactFwdDynamics(state, actuation, action_1_contact_model, action_1_cost_model)
action_1_model = croco.IntegratedActionModelEuler(action_1_dif_model, dt)

actions_model = actions_model + [action_1_model]*action_1_T


###################################################################################### Action 2 Push
action_2_T = int(.5/dt)
print("action_2_T: ", action_2_T)
## COSTS
action_2_cost_model  = croco.CostModelSum(state)
# Reg cost
action_2_cost_model.addCost('uReg', u_cost, 1e-4)
action_2_cost_model.addCost('xReg', x_cost, 1e-4)
## CONTACTS
action_2_contact_model = croco.ContactModelMultiple(state)
# Feet Contact         
for idx, foot_name in enumerate(feet_name_list) :
    foot_id = robot.model.getFrameId(foot_name)
    foot_pos = feet_pos_list[idx]
    foot_contact = croco.ContactModel3D(state, foot_id, foot_pos)
    action_2_contact_model.addContact(foot_name, foot_contact)
## Integrated Action
#action_2_dif_model = croco.DifferentialActionModelContactFwdDynamics(state, actuation, action_2_contact_model, action_2_cost_model)
action_2_dif_model = croco.DifferentialActionModelFreeFwdDynamics(state, actuation, action_2_cost_model)
action_2_model = croco.IntegratedActionModelEuler(action_2_dif_model, dt)

actions_model = actions_model + [action_2_model]*action_2_T


###################################################################################### Action 3 Fly
action_3_T = int(0.05/dt)
print("action_3_T: ", action_3_T)
## COSTS
action_3_cost_model  = croco.CostModelSum(state)
# Base Link Goal Cost
frame_goal = pin.SE3(rpyToMatrix(np.array([0, 0, 0])), np.array([.0, .0, .4]))
base_link_id  = robot.model.getFrameId('base_link')
base_link_res = croco.ResidualModelFramePlacement(state, base_link_id, frame_goal)
base_link_goal_cost = croco.CostModelResidual(state, base_link_res)
action_3_cost_model.addCost('base_link', base_link_goal_cost, 100)
# Reg cost
action_3_cost_model.addCost('uReg', u_cost, 1e-4)
action_3_cost_model.addCost('xReg', x_cost, 1e-4)
## Integrated Action
action_3_dif_model = croco.DifferentialActionModelFreeFwdDynamics(state, actuation, action_3_cost_model)
action_3_model = croco.IntegratedActionModelEuler(action_3_dif_model, dt)

actions_model = actions_model + [action_3_model]*action_3_T

###################################################################################### Action 4 In air
action_4_T = int(.5/dt)
print("action_4_T: ", action_4_T)
## COSTS
action_4_cost_model  = croco.CostModelSum(state)
# Reg cost
action_4_cost_model.addCost('uReg', u_cost, 1e-4)
action_4_cost_model.addCost('xReg', x_cost, 1e-4)
## CONTACTS
action_4_contact_model = croco.ContactModelMultiple(state)
# Feet Contact         
for idx, foot_name in enumerate(feet_name_list) :
    foot_id = robot.model.getFrameId(foot_name)
    foot_pos = feet_pos_list[idx]
    foot_contact = croco.ContactModel3D(state, foot_id, foot_pos)
    action_4_contact_model.addContact(foot_name, foot_contact)
## Integrated Action
#action_4_dif_model = croco.DifferentialActionModelContactFwdDynamics(state, actuation, action_4_contact_model, action_4_cost_model)
action_4_dif_model = croco.DifferentialActionModelFreeFwdDynamics(state, actuation, action_4_cost_model)
action_4_model = croco.IntegratedActionModelEuler(action_4_dif_model, dt)

actions_model = actions_model + [action_4_model]*action_4_T

###################################################################################### Action 5 Landing## Creation Contact
action_5_T = int(1/dt)
print("action_5_T: ", action_5_T)
## COSTS
action_5_cost_model  = croco.CostModelSum(state)
# Reg cost
action_5_cost_model.addCost('uReg', u_cost, 1e-4)
action_5_cost_model.addCost('xReg', x_cost, 1e-2)
## CONTACTS
action_5_contact_model = croco.ContactModelMultiple(state)
# Feet Contact         
for idx, foot_name in enumerate(feet_name_list) :
    foot_id = robot.model.getFrameId(foot_name)
    foot_pos = feet_pos_list[idx]
    foot_contact = croco.ContactModel3D(state, foot_id, foot_pos)
    action_5_contact_model.addContact(foot_name, foot_contact)
## Integrated Action
action_5_dif_model = croco.DifferentialActionModelContactFwdDynamics(state, actuation, action_5_contact_model, action_5_cost_model)
action_5_model = croco.IntegratedActionModelEuler(action_5_dif_model, dt)

actions_model = actions_model + [action_5_model]*action_5_T

###################################################################################### Action Terminal
## COSTS
action_terminal_cost_model  = croco.CostModelSum(state)
# Reg cost
action_terminal_cost_model.addCost('uReg', u_cost, 1e-4)
action_terminal_cost_model.addCost('xReg', x_cost, 1e-3)
## CONTACTS
action_terminal_contact_model = croco.ContactModelMultiple(state)
# Feet Contact         
for idx, foot_name in enumerate(feet_name_list) :
    foot_id = robot.model.getFrameId(foot_name)
    foot_pos = feet_pos_list[idx]
    foot_contact = croco.ContactModel3D(state, foot_id, foot_pos)
    action_terminal_contact_model.addContact(foot_name, foot_contact)
## Integrated Action
action_terminal_dif_model = croco.DifferentialActionModelContactFwdDynamics(state, actuation, action_terminal_contact_model, action_terminal_cost_model)
action_terminal_model = croco.IntegratedActionModelEuler(action_terminal_dif_model, 0)


## Solve
problem = croco.ShootingProblem(x0, actions_model, action_terminal_model)
solver = croco.SolverFDDP(problem)
solver.solve()

# Save the trajectory
np.save('jump_move.npy', solver.xs)
