import numpy as np
from robot import Bipedal
import time
from mpl_toolkits.mplot3d import Axes3D



if __name__ == "__main__":
    bipedal = Bipedal() 
    zc = 0.4 #Center of Mass height

    targetRPY = [0.0, 0.0, 0.0]
    targetPositionL = [0.0,0.065,-zc]
    targetPositionR = [0.0,-0.065,-zc]
    bipedal.positionInitialize(initializeTime=0.2)
    time.sleep(10)

    while(1):
        for i in np.arange(0,0.10,0.001):
            targetRPY = [0.0, 0.0, 0.0]
            targetPositionL[2] -= 0.001
            targetPositionR[2] -= 0.001
            PosL = bipedal.inverseKinematics(targetPositionL, targetRPY, bipedal.L)
            PosR = bipedal.inverseKinematics(targetPositionR, targetRPY, bipedal.R)
            bipedal.setLeftLegJointPositions(PosL)
            bipedal.setRightLegJointPositions(PosR)

            bipedal.oneStep()
        
            
        
            

        for i in np.arange(0,0.10,0.001):
            targetRPY = [0.0, 0.0, 0.0]
            targetPositionL[2] += 0.001
            targetPositionR[2] += 0.001
            PosL = bipedal.inverseKinematics(targetPositionL, targetRPY, bipedal.L)
            PosR = bipedal.inverseKinematics(targetPositionR, targetRPY, bipedal.R)
            bipedal.setLeftLegJointPositions(PosL)
            bipedal.setRightLegJointPositions(PosR)

            bipedal.oneStep()
        
        
    bipedal.disconnect()

