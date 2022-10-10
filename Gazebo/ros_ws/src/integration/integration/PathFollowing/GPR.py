import numpy as np
import sklearn.gaussian_process as gp
from math import ceil

# private libs.

class GPR():
    def __init__(self) -> None:

        ## GPR params
        self.TrainMax   =   20
        self.Estimation_per_dtMPPI  =   5
        self.Update_per_Estimation  =   3

        self.dt_Est         =   0.08
        self.dt_GPR         =   self.dt_Est / 4
        self.EstimateCycle  =   ceil(self.dt_Est * self.Estimation_per_dtMPPI / self.dt_GPR)
        self.UpdateCycle    =   ceil(self.dt_Est * self.Estimation_per_dtMPPI * self.Update_per_Estimation / self.dt_GPR)
        self.N          =   50

        ## GPR vars
        self.GPRfullFLAG = 0
        self.GPRupdtFLAG = 0
        self.xTrain     =   []
        self.yTrainX    =   []
        self.yTrainY    =   []
        self.yTrainZ    =   []
        self.yTrain     =   []
        
        self.yPred      =   np.zeros((self.N, 3))
        self.var        =   np.zeros((self.N, 3))

        self.count      =   0


    #.. GPR learning
        # kernel          =   gp.kernels.RBF(length_scale=1e-2, length_scale_bounds=(1e-3, 100.0)) * gp.kernels.ConstantKernel(1, (1e-2, 200.0))
        kernel          =   gp.kernels.RBF(length_scale=10, length_scale_bounds=(1e-5, 10.0)) * gp.kernels.ConstantKernel(1, (1e-2, 200.0)) + gp.kernels.DotProduct()
        self.gpr        =   gp.GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=0, alpha=0.1, normalize_y=True) #0.001
        pass    

    def GPRparams_from_MPPIparams(self, dt_MPPI, MPPIcycle_per_MPPIdt, N_MPPI):
        self.dt_Est         =   dt_MPPI
        self.dt_GPR         =   self.dt_Est / 4
        self.Estimation_per_dtMPPI  =   MPPIcycle_per_MPPIdt
        self.EstimateCycle  =   ceil(self.dt_Est * self.Estimation_per_dtMPPI / self.dt_GPR)
        self.UpdateCycle    =   ceil(self.dt_Est * self.Estimation_per_dtMPPI * self.Update_per_Estimation / self.dt_GPR)
        self.N              =   N_MPPI
        pass

    def GPR_dataset(self, x_new, Y_new):
        self.GPRfullFLAG =   0

        self.xTrain     =   np.append(self.xTrain,  x_new)

        self.yTrainX    =   np.append(self.yTrainX, Y_new[0])
        self.yTrainY    =   np.append(self.yTrainY, Y_new[1])
        self.yTrainZ    =   np.append(self.yTrainZ, Y_new[2])

        # Maintaining buffer size
        if (len(self.yTrainX) > self.TrainMax):
            self.xTrain     =   np.delete(self.xTrain, 0, 0)

            self.yTrainX    =   np.delete(self.yTrainX, 0, 0)
            self.yTrainY    =   np.delete(self.yTrainY, 0, 0)
            self.yTrainZ    =   np.delete(self.yTrainZ, 0, 0)

            self.GPRfullFLAG = 1

        pass


    def GPR_update(self):

        if (self.GPRfullFLAG == 1):
            XTrain = self.xTrain.reshape(-1,1)
            yTrain = np.concatenate((self.yTrainX.reshape(-1,1), self.yTrainY.reshape(-1,1), self.yTrainZ.reshape(-1,1)), axis=1)
            
            self.gpr.fit(XTrain, yTrain)
            self.GPRupdtFLAG = 1

            # print("GPR dataset XTrain")
            # print(XTrain)
            # print("GPR dataset yTrain")
            # print(yTrain)

        pass


    def GPR_estimate(self, xTemp, testSize, dt):

        if (self.GPRupdtFLAG == 1): # and GCUiRun % testPeriod == 0):
            
            # start = time.time()
            # Get test points

            if(testSize > 1):
                xTest = np.arange(xTemp, xTemp + testSize * dt, dt)
            else:
                xTest = np.array([xTemp])
                
            XTest = xTest.reshape(-1,1)
            
            self.yPred, self.var = self.gpr.predict(XTest, return_std=True)
            
            aa = 0

            # print("time :", time.time() - start)

            # xTest_temp = np.arange(xTemp, xTemp + 5, 0.004)
            # XTest_temp = xTest_temp.reshape(-1,1)
            
            # yPred_temp, var_temp = gpr.predict(XTest_temp, return_std=True)
            # print(yPred_temp)

        pass