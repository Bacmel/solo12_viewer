import crocoddyl as croco
import example_robot_data
import numpy as np
import pinocchio as pin
from pinocchio.utils import *
import matplotlib.pyplot as plt



############################################################### DATA
## LISTS OF DATA
feet_name_list = ['FL_FOOT', 'FR_FOOT', 'HL_FOOT', 'HR_FOOT']

lower_legs_name_list = ['FL_LOWER_LEG', 'FR_LOWER_LEG', 'HL_LOWER_LEG', 'HR_LOWER_LEG']
lower_legs_pos_list = [0.2, 0.2, -0.2, -0.2]

upper_legs_name_list = ['FL_UPPER_LEG', 'FR_UPPER_LEG', 'HL_UPPER_LEG', 'HR_UPPER_LEG']
upper_legs_pos_list = [-0.3, -0.3, 0.3, 0.3]


shoulders_name_list = ['FL_SHOULDER', 'FR_SHOULDER', 'HL_SHOULDER', 'HR_SHOULDER']

# feet_pos_list = [np.array([0.1946, 0.14695, 0.0]),
#                  np.array([0.1946, -0.14695, 0.0]),
#                  np.array([-0.1946, 0.14695, 0.0]),
#                  np.array([-0.1946, -0.14695, 0.0])]

feet_pos_list = [np.array([0.3, 0.14695, 0.0]),
                 np.array([0.3, -0.14695, 0.0]),
                 np.array([-0.3, 0.14695, 0.0]),
                 np.array([-0.3, -0.14695, 0.0])]

################################################################ Global variables
robot = example_robot_data.load('solo12')
robot.model.effortLimit[6:] = np.ones(12) * 1.2
nq = robot.nq
q0 = robot.q0
nv = robot.nv
v0 = robot.v0

q_start = np.array([
                    0.0, 0.0, 0.07,
                    0.0, 0.0, 0.0, 1.0,
                    0.0, -2.2, 2.4,
                    0.0, -2.2, 2.4,
                    0.0, 2.2, -2.4,
                    0.0, 2.2, -2.4])

q0 = q_start

x0 = np.concatenate([q0, v0])

pin.framesForwardKinematics(robot.model, robot.data, q0)



## Physical constants
dt = 10e-3 # in [s]


## Creation state / actuation
state = croco.StateMultibody(robot.model)
actuation = croco.ActuationModelFloatingBase(state)

base_link_id  = robot.model.getFrameId('base_link')
actions_model = []

## Free Command Regulation Cost
u_free_res = croco.ResidualModelControl(state, actuation.nu)
u_free_cost = croco.CostModelResidual(state, u_free_res)

## Contact Command Regulation Cost
u_contact_res = croco.ResidualModelControl(state, actuation.nu)
u_contact_cost = croco.CostModelResidual(state, u_contact_res)

## State Regulation Cost 
x_res = croco.ResidualModelState(state, x0, actuation.nu)
x_cost = croco.CostModelResidual(state, x_res)


def feet_contact_cost(contact_model, cost_model, w=10):
    motion_zero = pin.Motion(np.array([.0, .0, .0]), np.array([.0, .0, .0]))
    for idx, foot_name in enumerate(feet_name_list):
        foot_id = robot.model.getFrameId(foot_name)
        foot_pos = feet_pos_list[idx]
        foot_contact = croco.ContactModel3D(state, foot_id, foot_pos, actuation.nu)
        contact_model.addContact(foot_name, foot_contact)
        foot_pos_res = croco.ResidualModelFrameTranslation(state, foot_id, foot_pos, actuation.nu)
        foot_pos_cost = croco.CostModelResidual(state, foot_pos_res)
        cost_model.addCost(foot_name+' position', foot_pos_cost, w)
        foot_vel_res = croco.ResidualModelFrameVelocity(state, foot_id, motion_zero, pin.LOCAL, actuation.nu)
        foot_vel_cost = croco.CostModelResidual(state, foot_vel_res)
        cost_model.addCost(foot_name+' velocity', foot_vel_cost, w)

def shoulders_cost(cost_model, w=100):
    rotation_zero = rpyToMatrix(np.array([0, 0, 0]))
    for shoulder_name in shoulders_name_list:
        shoulder_id = robot.model.getFrameId(shoulder_name)
        shoulder_res = croco.ResidualModelFrameRotation(state, shoulder_id, rotation_zero, actuation.nu)
        shoulder_cost = croco.CostModelResidual(state, shoulder_res)
        cost_model.addCost(shoulder_name+' rotation', shoulder_cost, w) 

def upper_leg_cost(cost_model, w=1000):  
    for idx, upper_leg_name in enumerate(upper_legs_name_list):
        rotation_zero = rpyToMatrix(np.array([0, upper_legs_pos_list[idx], 0]))
        upper_leg_id = robot.model.getFrameId(upper_leg_name)
        upper_leg_res = croco.ResidualModelFrameRotation(state, upper_leg_id, rotation_zero, actuation.nu)
        upper_leg_cost = croco.CostModelResidual(state, upper_leg_res)
        cost_model.addCost(upper_leg_name+' rotation', upper_leg_cost, w)

def lower_leg_cost(cost_model, w=1000):
    for idx, lower_leg_name in enumerate(lower_legs_name_list):
        rotation_zero = rpyToMatrix(np.array([0, lower_legs_pos_list[idx], 0]))
        lower_leg_id = robot.model.getFrameId(lower_leg_name)
        lower_leg_res = croco.ResidualModelFrameRotation(state, lower_leg_id, rotation_zero, actuation.nu)
        lower_leg_cost = croco.CostModelResidual(state, lower_leg_res)
        cost_model.addCost(lower_leg_name+' rotation', lower_leg_cost, w)
    

###################################################################################### Action 1 Preparation
action_1_T = int(0.3/dt)  #0.135

## COSTS
action_1_cost_model  = croco.CostModelSum(state, actuation.nu)

# Reg cost
action_1_cost_model.addCost('uReg', u_contact_cost, 1e3)
action_1_cost_model.addCost('xReg', x_cost, 1e-2)

# ## Base Link Start Position
# frame_goal = np.array([.0, .0, 0.05])
# base_link_res = croco.ResidualModelFrameTranslation(state, base_link_id, frame_goal, actuation.nu)
# base_link_goal_cost = croco.CostModelResidual(state, base_link_res)
# action_1_cost_model.addCost('base_link', base_link_goal_cost, 100)

rotation_goal = rpyToMatrix(np.array([0, 0, 0]))
base_link_rotation_res = croco.ResidualModelFrameRotation(state, base_link_id, rotation_goal, actuation.nu)
base_link_rotation_goal_cost = croco.CostModelResidual(state, base_link_rotation_res)
action_1_cost_model.addCost('base_link rotation', base_link_rotation_goal_cost, 1e4)

#shoulders_cost(action_1_cost_model, 1e2)

## CONTACTS
action_1_contact_model = croco.ContactModelMultiple(state, actuation.nu)
# Feet Contact & Cost
feet_contact_cost(action_1_contact_model, action_1_cost_model)
## Integrated Action
action_1_dif_model = croco.DifferentialActionModelContactFwdDynamics(state, actuation, action_1_contact_model, action_1_cost_model)
action_1_model = croco.IntegratedActionModelEuler(action_1_dif_model, dt)

actions_model = actions_model + [action_1_model]*action_1_T


###################################################################################### Action 2 Takeoff
action_2_T = 1

## COSTS
action_2_cost_model  = croco.CostModelSum(state, actuation.nu)

# Reg cost
action_2_cost_model.addCost('uReg', u_contact_cost, 1e-3)
#action_2_cost_model.addCost('xReg', x_cost, 1e-4)

# # Base Link Goal Cost
# motion_goal = pin.Motion(np.array([.0, .0, 3]), np.array([.0, .0, .0]))
# base_link_id  = robot.model.getFrameId('base_link')
# base_link_motion_res = croco.ResidualModelFrameVelocity(state, base_link_id, motion_goal, pin.WORLD, actuation.nu)
# base_link_motion_goal_cost = croco.CostModelResidual(state, base_link_motion_res)
# action_2_cost_model.addCost('base_link motion', base_link_motion_goal_cost, 1e4*action_1_T)

# # Takeoff position
# takeoff_goal = np.array([0, 0, 0.32])
# base_link_pos_res = croco.ResidualModelFrameTranslation(state, base_link_id, takeoff_goal, actuation.nu)
# base_link_pos_goal_cost = croco.CostModelResidual(state, base_link_pos_res)
# action_2_cost_model.addCost('base_link translation', base_link_pos_goal_cost, 1e-4)

## CONTACTS
action_2_contact_model = croco.ContactModelMultiple(state, actuation.nu)
# Feet Contact & Cost
feet_contact_cost(action_2_contact_model, action_2_cost_model)
## Integrated Action
action_2_dif_model = croco.DifferentialActionModelContactFwdDynamics(state, actuation, action_2_contact_model, action_2_cost_model)
action_2_model = croco.IntegratedActionModelEuler(action_2_dif_model, dt)

#actions_model = actions_model + [action_2_model]*action_2_T


###################################################################################### Action 3 Flying
action_3_T = int(0.3/dt)    #0.30

## COSTS
action_3_cost_model  = croco.CostModelSum(state, actuation.nu)

# Reg cost
action_3_cost_model.addCost('uReg', u_free_cost, 1e0)
action_3_cost_model.addCost('xReg', x_cost, 1e1)

#shoulders_cost(action_3_cost_model)
#shoulders_cost(action_3_cost_model)
shoulders_cost(action_3_cost_model, 1e3)
upper_leg_cost(action_3_cost_model, 1e2)
lower_leg_cost(action_3_cost_model, 1e2)

rotation_goal = rpyToMatrix(np.array([0, 0, 0]))
base_link_rotation_res = croco.ResidualModelFrameRotation(state, base_link_id, rotation_goal, actuation.nu)
base_link_rotation_goal_cost = croco.CostModelResidual(state, base_link_rotation_res)
action_3_cost_model.addCost('base_link rotation', base_link_rotation_goal_cost, 1e3)


## Integrated Action
action_3_dif_model = croco.DifferentialActionModelFreeFwdDynamics(state, actuation, action_3_cost_model)
action_3_model = croco.IntegratedActionModelEuler(action_3_dif_model, dt)

actions_model = actions_model + [action_3_model]*action_3_T

###################################################################################### Action 4 Landing
# action_4_T = int(1/dt)

# ## COSTS
# action_4_cost_model = croco.CostModelSum(state, actuation.nu)

# # Reg cost
# action_4_cost_model.addCost('uReg', u_contact_cost, 1e-3)
# action_4_cost_model.addCost('xReg', x_cost, 1e-1)

# ## CONTACTS
# action_4_contact_model = croco.ContactModelMultiple(state, actuation.nu)
# # Feet Contact & Cost
# feet_contact_cost(action_4_contact_model, action_4_cost_model)
# ## Integrated Action
# action_4_dif_model = croco.DifferentialActionModelContactFwdDynamics(state, actuation, action_4_contact_model, action_4_cost_model)
# action_4_model = croco.IntegratedActionModelEuler(action_4_dif_model, dt)

# actions_model = actions_model + [action_4_model]*action_4_T

# ###################################################################################### Action 5 Rest
# action_5_T = int(0.025/dt)

# ## COSTS
# action_5_cost_model  = croco.CostModelSum(state, actuation.nu)

# # Reg cost
# action_5_cost_model.addCost('uReg', u_contact_cost, 1e-4)
# action_5_cost_model.addCost('xReg', x_cost, 1e-2)

# ## CONTACTS
# action_5_contact_model = croco.ContactModelMultiple(state, actuation.nu)
# # Feet Contact & Cost
# feet_contact_cost(action_5_contact_model, action_5_cost_model)
# ## Integrated Action
# action_5_dif_model = croco.DifferentialActionModelContactFwdDynamics(state, actuation, action_5_contact_model, action_5_cost_model)
# action_5_model = croco.IntegratedActionModelEuler(action_5_dif_model, dt)

# actions_model = actions_model + [action_5_model]*action_5_T

###################################################################################### Action Terminal
## COSTS
action_terminal_cost_model  = croco.CostModelSum(state, actuation.nu)

# Reg cost
#action_terminal_cost_model.addCost('uReg', u_contact_cost, 1e-4)
action_terminal_cost_model.addCost('xReg', x_cost, 1e2)

shoulders_cost(action_terminal_cost_model)
upper_leg_cost(action_terminal_cost_model)
lower_leg_cost(action_terminal_cost_model)

# terminal position
terminal_goal = np.array([0.0, 0, 1.0])
base_link_pos_res = croco.ResidualModelFrameTranslation(state, base_link_id, terminal_goal, actuation.nu)
base_link_pos_goal_cost = croco.CostModelResidual(state, base_link_pos_res)
action_terminal_cost_model.addCost('base_link translation', base_link_pos_goal_cost, 1e4)

# terminal rotation
rotation_goal = rpyToMatrix(np.array([0, 0, 0]))
base_link_rotation_res = croco.ResidualModelFrameRotation(state, base_link_id, rotation_goal, actuation.nu)
base_link_rotation_goal_cost = croco.CostModelResidual(state, base_link_rotation_res)
action_terminal_cost_model.addCost('base_link rotation', base_link_rotation_goal_cost, 1e4)


## CONTACTS
action_terminal_contact_model = croco.ContactModelMultiple(state, actuation.nu)
# Feet Contact & Cost
# feet_contact_cost(action_terminal_contact_model, action_terminal_cost_model)
## Integrated Action
action_terminal_dif_model = croco.DifferentialActionModelContactFwdDynamics(state, actuation, action_terminal_contact_model, action_terminal_cost_model)
action_terminal_model = croco.IntegratedActionModelEuler(action_terminal_dif_model, 0)


## Solve
problem = croco.ShootingProblem(x0, actions_model, action_terminal_model)
solver = croco.SolverBoxFDDP(problem)
solver.setCallbacks([croco.CallbackVerbose()])
solver.solve([], [], 500)

# Save the trajectory
np.savez('ressource/jump_move', xs=solver.xs, us=solver.us)


# ----- DISPLAY -----

display = croco.GepettoDisplay(robot)
display.displayFromSolver(solver, factor=1)
display.displayFromSolver(solver, factor=2)
display.displayFromSolver(solver, factor=3)
display.displayFromSolver(solver, factor=4)



# ----- PLOTS -----
ts_a = np.array([dt * i for i in range(len(solver.us))])
us_a = np.vstack(solver.us)

fig0, axs0 = plt.subplots(nrows=4, ncols=1)
names = [
    'FL_SHOULDER_0', 'FL_SHOULDER_1', 'FL_ELBOW', 'FR_SHOULDER_0',
    'FR_SHOULDER_1', 'FR_ELBOW', 'RL_SHOULDER_0', 'RL_SHOULDER_1', 'RL_ELBOW',
    'RR_SHOULDER_0', 'RR_SHOULDER_1', 'RR_ELBOW'
]
for idx, ax in enumerate(axs0):
    for i in range(3):
        ax.plot(ts_a, us_a[:, idx * 3 + i])
    ax.legend(names[idx * 3:idx * 3 + 3])

# Plot state base_link
ts_a = np.array([dt * i for i in range(len(solver.xs))])
xs_a = np.vstack(solver.xs)

fig1, axs1 = plt.subplots(nrows=4, ncols=1)

# pos
for i in range(3):
    axs1[0].plot(ts_a, xs_a[:, i])
axs1[0].legend(['x', 'y', 'z'])

# lin vel
for i in range(3):
    axs1[2].plot(ts_a, xs_a[:, robot.model.nq + i])
axs1[2].legend(['x', 'y', 'z'])

# # Plot contact forces
# fs = display.getForceTrajectoryFromSolver(solver)
# fs_a = np.zeros([len(fs), 4])

# for idx_feet, f_feet in enumerate(fs):
#     for idx_foot, f_foot in enumerate(f_feet):
#         fs_a[idx_feet, idx_foot] = np.linalg.norm(f_foot['f'])

# ts_a = np.array([dt * i for i in range(len(fs))])

# fig2, axs2 = plt.subplots(nrows=4, ncols=1)

# for idx, ax in enumerate(axs2):
#     ax.plot(ts_a, fs_a[:, idx])

plt.show()

