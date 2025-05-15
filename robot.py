import pybullet as pb
import pybullet_data
import numpy as np
import time
import scipy.linalg as la
import os 



def getTransFromRp(R,p):
    T = np.vstack((np.hstack((R, np.array([[p[0]],[p[1]],[p[2]]]))), np.array([0,0,0,1])))
    return T

def getPositionFromT(T):
    p = T[0:3,3]
    return p

def getRotationFromT(T):
    R = T[0:3,0:3]
    return R

def getRotationAndPositionFromT(T):
    p = T[0:3,3]
    R = T[0:3,0:3]
    return R, p

def getRollPitchYawFromR(R):
    pitch = np.arcsin(-R[2,0])
    yaw = np.arctan2(R[1,0], R[0,0])
    roll = np.arctan2(R[2,1],R[2,2])

    return np.array([roll, pitch, yaw])

def skewSymmetricMatrix(v):
    matrix = np.array([[0, -v[2], v[1]],
                        [v[2], 0, -v[0]],
                        [-v[1], v[0], 0]])
    return matrix


def roadriguesEquation(E, a, theta):
    a_h = skewSymmetricMatrix(a)
    return E + a_h * np.sin(theta) + a_h.dot(a_h) * (1-np.cos(theta))

class Bipedal():

    def __init__(self, maxForce=9.0, controlMode=pb.POSITION_CONTROL, robotPATH = os.path.dirname(__file__) + "/urdf/bipedal.urdf", planePATH="plane.urdf"):
        
        physicsClient = pb.connect(pb.GUI)
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        pb.setGravity(0,0,-9.8)
        
        self._planeId = pb.loadURDF(planePATH)
        self._robotId = pb.loadURDF(robotPATH)
        self._controlMode = controlMode
        self.numJoint = pb.getNumJoints(self._robotId)
       
        self._jointIdList = list(range(self.numJoint))
        self.maxForce = maxForce
        self._maxForceList = [maxForce]*12

        self._timeStep = 1./240. 
        self._lambda = 0.95
        self._L1 = 0.18
        self._L2 = 0.18
        self.R = np.array([0,-0.065,-0.175])
        self.L = np.array([0,0.065,-0.175])
        self.LEG_DOF = 6

        self._jointIdListR = [0,1,2,3,4,5]
        self._jointIdListL = [6,7,8,9,10,11]
        self._maxForceListForLeg = [maxForce]*self.LEG_DOF

        self._a = np.array([[0,0,1], 
                                [1,0,0],
                                [0,1,0],
                                [0,1,0],
                                [0,1,0],
                                [1,0,0]])

        self._E = np.eye(3)


    def resetRobotPositionAndOrientation(self, position, orientation):
        pb.resetBasePositionAndOrientation(self._robotId, position, orientation)

    def oneStep(self):      
        robotPosition, _ = pb.getBasePositionAndOrientation(self._robotId)
        pb.resetDebugVisualizerCamera(cameraDistance = 1, 
                                      cameraYaw = 135, 
                                      cameraPitch = -10, 
                                      cameraTargetPosition = robotPosition)
        pb.stepSimulation()
        time.sleep(self._timeStep)

    def setRightLegJointPositions(self, targetJointPositions):
        pb.setJointMotorControlArray(self._robotId, 
                                     jointIndices = self._jointIdListR, 
                                     controlMode = self._controlMode, 
                                     forces = self._maxForceListForLeg, 
                                     targetPositions = targetJointPositions)

    def setLeftLegJointPositions(self, targetJointPositions):
        pb.setJointMotorControlArray(self._robotId, 
                                     jointIndices = self._jointIdListL, 
                                     controlMode = self._controlMode, 
                                     forces = self._maxForceListForLeg, 
                                     targetPositions = targetJointPositions)


        
    def getLegTrans(self, jointPositions, leg):
        hipyaw = jointPositions[0]
        hiproll = jointPositions[1]
        hippitch = jointPositions[2]
        knee = jointPositions[3]
        anklepitch = jointPositions[4]
        ankleroll = jointPositions[5]
        zero_v = np.zeros(3)

        T_0_1 = getTransFromRp(roadriguesEquation(self._E, self._a[0], hipyaw),leg)
        T_0_2 = T_0_1.dot( getTransFromRp(roadriguesEquation(self._E, self._a[1], hiproll), zero_v))
        T_0_3 = T_0_2.dot( getTransFromRp(roadriguesEquation(self._E, self._a[2], hippitch), zero_v))
        T_0_4 = T_0_3.dot( getTransFromRp(roadriguesEquation(self._E, self._a[3], knee), [0,0,-self._L1]))
        T_0_5 = T_0_4.dot( getTransFromRp(roadriguesEquation(self._E, self._a[4], anklepitch), [0,0,-self._L2]))
        T_0_6 = T_0_5.dot( getTransFromRp(roadriguesEquation(self._E, self._a[5], ankleroll), zero_v))

        return T_0_1, T_0_2, T_0_3, T_0_4, T_0_5, T_0_6
        
        
        
    def forwardKinematics(self, jointPositions, leg):

        T_0_6 = self.getLegTrans(jointPositions, leg)[5]

        return getRotationAndPositionFromT(T_0_6)

    def inverseKinematics(self, p_ref, omega_ref, leg):

        q = self.getJointPositions(leg)
        R, p = self.forwardKinematics(q, leg)
        omega = np.array(getRollPitchYawFromR(R))

        dp = p_ref - p
        domega = omega_ref - omega
        dp_domega = np.append(dp,domega)

        dq = self._lambda * la.inv(self.jacobian(q, leg)).dot(dp_domega)


        return q+dq

    def jacobian(self, q, leg):
        T0 = self.getLegTrans(q, leg)
        zero_v = np.zeros(3)

        R = [getRotationFromT(T0[i]) for i in range(len(T0))]
        p = [getPositionFromT(T0[i]) for i in range(len(T0))]

        wa = [R[i].dot(self._a[i]) for i in range(len(R))]

        Jp = np.vstack([np.hstack((np.cross(wa[i], (p[5] - p[i])), wa[i])) for i in range(len(wa) - 1)])
        J = np.vstack((Jp, np.hstack((zero_v, wa[5])))).T

        return J

    def getJointPositions(self, leg):
        if np.sum(leg == self.R) == len(leg):
            jointStates = pb.getJointStates(self._robotId, jointIndices=self._jointIdListR)
            jointPositions = [jointStates[i][0] for i in range(len(jointStates))]

        elif np.sum(leg == self.L) == len(leg):
            jointStates = pb.getJointStates(self._robotId, jointIndices=self._jointIdListL)
            jointPositions = [jointStates[i][0] for i in range(len(jointStates))]
        
        else:
            raise ValueError("invalid parameter")

        return jointPositions

    def positionInitialize(self, 
                           startCOMheight=0.45, 
                           initialLegRPY=[0, 0, 0],
                           initialJointPosRL=[0.0,0.0,-0.44,0.88,0,0.0]):
        

        for _ in range(10): 
            self.setLeftLegJointPositions(initialJointPosRL)
            self.setRightLegJointPositions(initialJointPosRL)
            self.resetRobotPositionAndOrientation(position = [0, 0,startCOMheight], 
                                                  orientation = pb.getQuaternionFromEuler(initialLegRPY))
            self.oneStep()
    
    def disconnect(self):
        pb.disconnect()
