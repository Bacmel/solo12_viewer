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

## Creation state
state = croco.StateMultibody(robot.model)

########################################## COST

## Creation Cost
running_cost_model  = croco.CostModelSum(state)
terminal_cost_model = croco.CostModelSum(state)

## Command Regulation Cost
u_res = croco.ResidualModelControl(state)
u_cost = croco.CostModelResidual(state, u_res)
running_cost_model.addCost('uReg', u_cost, 1e-4)
        
## State Regulation Cost 
x_res = croco.ResidualModelState(state, x0)
x_cost = croco.CostModelResidual(state, x_res)
running_cost_model.addCost('xReg', x_cost, 1e-3)

## 

     
## Feet Goal Cost
# for idx, foot_name in enumerate(feet_name_list) :
#    foot_id = robot.model.getFrameId(foot_name)
#    foot_pos = feet_pos_list[idx]
#    foot_res = croco.ResidualModelFrameTranslation(state, foot_id, foot_pos)
#    foot_goal_cost = croco.CostModelResidual(state, foot_res)
#    running_cost_model.addCost(foot_name, foot_goal_cost, 1)
#    terminal_cost_model.addCost(foot_name, foot_goal_cost, 1)      
        
## Base Link Goal Cost
frame_goal = pin.SE3(rpyToMatrix(np.array([0, 0, np.deg2rad(20)])), np.array([.0, .0, 0.2]))
base_link_id  = robot.model.getFrameId('base_link')
base_link_res = croco.ResidualModelFramePlacement(state, base_link_id, frame_goal)
base_link_goal_cost = croco.CostModelResidual(state, base_link_res)
terminal_cost_model.addCost('base_link', base_link_goal_cost, 1)

#################################################################################### CONTACT

## Creation Contact
contact_model = croco.ContactModelMultiple(state)

## Feet Contact         
for idx, foot_name in enumerate(feet_name_list) :
    foot_id = robot.model.getFrameId(foot_name)
    foot_pos = feet_pos_list[idx]
    foot_contact = croco.ContactModel3D(state, foot_id, foot_pos)
    contact_model.addContact(foot_name, foot_contact)

#################################################################################### Action Model

## Creation Action Model
actuation_model = croco.ActuationModelFull(state)

running_dif_model = croco.DifferentialActionModelContactFwdDynamics(state, actuation_model, contact_model, running_cost_model)
running_model = croco.IntegratedActionModelEuler(running_dif_model, dt)

terminal_dif_model = croco.DifferentialActionModelContactFwdDynamics(state, actuation_model, contact_model, terminal_cost_model)
terminal_model = croco.IntegratedActionModelEuler(terminal_dif_model, 0)    


## Solve
problem = croco.ShootingProblem(x0, [running_model]*T, terminal_model)
solver = croco.SolverFDDP(problem)
solver.solve()

# Save the trajectory
np.save('base_move.npy', solver.xs)
