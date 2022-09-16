from tempfile import TemporaryFile

import crocoddyl as croco

outfile = TemporaryFile()


# Goal frame
frame_name = 'base_link'
frame_goal = pin.SE3(np.eye(3), np.array([.05, .0, -0.1]))

# Physical constants
dt = 1e-3 # in [s]
T = 1000 # knots


    def crocoddyl_setup(self):
        # Creation state
        state = croco.StateMultibody(self.pin_robot.model)

        # Creation Cost
        runningCM  = croco.CostModelSum(state)
        terminalCM = croco.CostModelSum(state)

        ## Command Cost
        uRes = croco.ResidualModelControl(state)
        uCost = croco.CostModelResidual(state, uRes)
        runningCM.addCost('uReg', uCost, 1e-4)
        
        ## State Cost
        xRes = croco.ResidualModelControl(state)
        xCost = croco.CostModelResidual(state, xRes)
        runningCM.addCost('xReg', xCost, 1e-4)
        
        
        M_world_base_link = self.get_M_w_f('FL_FOOT')
        
        #
        frameid = self.pin_robot.model.getFrameId('FL_FOOT')
        frameRes = croco.ResidualModelFrameTranslation(state, frameid, M_world_base_link*np.array([0.1946, 0.14695, -0.2, 1]))
        goalCost = croco.CostModelResidual(state, frameRes)
        runningCM.addCost('FL_FOOT', goalCost, 1)
        terminalCM.addCost('FL_FOOT', goalCost, 1)
        
        frameid = self.pin_robot.model.getFrameId('FR_FOOT')
        frameRes = croco.ResidualModelFrameTranslation(state, frameid, M_world_base_link*np.array([0.1946, -0.14695, -0.2, 1]))
        goalCost = croco.CostModelResidual(state, frameRes)
        runningCM.addCost('FR_FOOT', goalCost, 1)
        terminalCM.addCost('FR_FOOT', goalCost, 1)

        frameid = self.pin_robot.model.getFrameId('HL_FOOT')
        frameRes = croco.ResidualModelFrameTranslation(state, frameid, M_world_base_link*np.array([-0.1946, 0.14695, -0.2, 1]))
        goalCost = croco.CostModelResidual(state, frameRes)
        runningCM.addCost('HL_FOOT', goalCost, 1)
        terminalCM.addCost('HL_FOOT', goalCost, 1)

        frameid = self.pin_robot.model.getFrameId('HR_FOOT')
        frameRes = croco.ResidualModelFrameTranslation(state, frameid, M_world_base_link*np.array([-0.1946, -0.14695, -0.2, 1]))
        goalCost = croco.CostModelResidual(state, frameRes)
        runningCM.addCost('HR_FOOT', goalCost, 1)
        terminalCM.addCost('HR_FOOT', goalCost, 1)
        
        
        
        ## Goal Cost
        #frameid = self.pin_robot.model.getFrameId(frame_name)
        #frameRes = croco.ResidualModelFramePlacement(state, frameid, frame_goal)
        #goalCost = croco.CostModelResidual(state, frameRes)
        #runningCM.addCost(frame_name, goalCost, 1)
        #terminalCM.addCost(frame_name, goalCost, 1)np.save(outfile, x)

        # Creation Action Model
        actuationM = croco.ActuationModelFull(state)
        self.runningM = croco.IntegratedActionModelEuler(croco.DifferentialActionModelFreeFwdDynamics(state, actuationM, runningCM), dt)
        self.runningM.differential.armature = np.full((self.nq, 1), 0.1)
        
        self.terminalM = croco.IntegratedActionModelEuler(croco.DifferentialActionModelFreeFwdDynamics(state, actuationM, terminalCM), 0.)
        self.terminalM.differential.armature = np.full((self.nq, 1), 0.1)
        ## Crocoddyl
        self.crocoddyl_setup()
        problem = crocoddyl.ShootingProblem(x0, [self.runningM]*T, self.terminalM)
        self.solver = crocoddyl.SolverDDP(problem)
        self.solver.solve()
        self.t = 0


## Crocoddyl
self.crocoddyl_setup()
problem = crocoddyl.ShootingProblem(x0, [self.runningM]*T, self.terminalM)
self.solver = crocoddyl.SolverDDP(problem)
self.solver.solve()
self.t = 0
np.save(outfile, self.solver.xs)
