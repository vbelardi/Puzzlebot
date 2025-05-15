
import numpy as np
import matplotlib.pyplot as plt

class Robot_planar:
    def __init__(self, dh_parameters):
        self.dh_parameters = dh_parameters
        self.n = len(dh_parameters)

    def dh_transform(self, theta, d, a, alpha):
        return np.array([
            [np.cos(theta), -np.sin(theta), 0, a],
            [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha), -d * np.sin(alpha)],
            [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha), d * np.cos(alpha)],
            [0, 0, 0, 1]
        ])

    def forward_kinematics(self, joint_angles):
        T = np.eye(4)
        positions = [(0, 0)]

        for i in range(self.n):
            theta, d, a, alpha = self.dh_parameters[i]
            theta += joint_angles[i] 
            A_i = self.dh_transform(theta, d, a, alpha)
            T = T @ A_i
            x, y, _ = T[:3, 3]
            positions.append((x, y))

        return positions

    def compute_jacobian(self, joint_angles):
        T = np.eye(4)
        J = np.zeros((2, self.n))

        positions = []
        for i in range(self.n):
            theta, d, a, alpha = self.dh_parameters[i]
            theta += joint_angles[i]
            A_i = self.dh_transform(theta, d, a, alpha)
            T = T @ A_i
            positions.append(T[:3, 3])

        for i in range(self.n):
            z = np.array([0, 0, 1, 0])  
            p_end_effector = positions[-1]
            p_i = positions[i]
            J[:, i] = np.cross(z[:3], (p_end_effector - p_i)[:3])[:2]  

        return J


    def inverse_kinematics(self, target_x, target_y, initial_guess = None, gamma = 0.95, tolerance=1e-2):
        
        if initial_guess is None:
            joint_angles = np.zeros(self.n, dtype=np.float64)
        else:
            joint_angles = np.array(initial_guess, dtype=np.float64)

        joint_angles_history = []
        joint_angles_history.append(joint_angles)
        positions = self.forward_kinematics(joint_angles)
        end_effector_pos = np.array(positions[-1])
        error = np.array([target_x, target_y]) - end_effector_pos[:2]

        while (np.linalg.norm(error) > tolerance):
            positions = self.forward_kinematics(joint_angles)
            end_effector_pos = np.array(positions[-1])
            error = np.array([target_x, target_y]) - end_effector_pos[:2]
            J = self.compute_jacobian(joint_angles)
            J_inv = np.linalg.pinv(J)
            joint_angles += J_inv.dot(error * gamma)
            joint_angles_history.append(joint_angles)
            

        return joint_angles_history

    def plot_robot(self, joint_angles_list, x_ref, y_ref):

        for joint_angles in joint_angles_list:
            positions = self.forward_kinematics(joint_angles)
            x_coords, y_coords = zip(*positions)
            plt.plot(x_ref, y_ref, color = 'black',marker = 'o',markersize=2)
            tmp1,  = plt.plot(x_coords, y_coords, 'bo-', label='Links')
            tmp2,  = plt.plot(x_coords[-1], y_coords[-1], 'go', label='Efector final')
            
            plt.xlim(-sum([p[2] for p in self.dh_parameters]) - 1, sum([p[2] for p in self.dh_parameters]) + 1)
            plt.ylim(-sum([p[2] for p in self.dh_parameters]) - 1, sum([p[2] for p in self.dh_parameters]) + 1)
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.title(f'Robot planar de {self.n} Grafos de Libertad')
            plt.legend()
            plt.grid(True)
            plt.gca().set_aspect('equal', adjustable='box')
            plt.pause(0.001)
            tmp1.remove()
            tmp2.remove()
        

if __name__ == '__main__':
    # Parametros DH
    dh_parameters = [
        (0, 0, 0, 0),  # (theta, d, a, alpha)
        (0, 0, 1.0, 0),
        (0, 0, 1.0, 0),

    ]
    robot = Robot_planar(dh_parameters)
    # Trajectoria circular x(t) = px + r * cos(t)
    #                      y(t) = py + r * sin(t)
    r = 3
    t = np.arange(0,5,0.2)
    x_ref = np.ones(25)
    y_ref = np.ones(25)
    print(t)
    print(x_ref)
    # Se obtienen los angulos en home position
    joint_angles = [p[0] for p in robot.dh_parameters]

    for i in range(len(t)):
        joint_angles_history = robot.inverse_kinematics(x_ref[i], y_ref[i], joint_angles)
        joint_angles = joint_angles_history[-1]
        print(joint_angles_history)
        robot.plot_robot(joint_angles_history, x_ref[i], y_ref[i])
