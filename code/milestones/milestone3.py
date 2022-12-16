import modern_robotics as mr
import numpy as np

def FeedbackControl(x, x_d, x_d_next, Kp, Ki, dt):
    """
    Function to compute the twist and configuration error based on PI control

    Args:
        x: current configuration of robot
        x_d: desired configuration of robot from generated trajectory
        x_d_next: desired configuration of robot from generated trajectory at next time step
        Kp: 6x6 matrix of proportional gain terms
        Ki: 6x6 matrix of integral gain terms
        dt: time step


    Returns
    -------
        V: robot control twist
        x_err_vec: error in robot configuration from generated trajectory
    """

    #compute the configuration error of robot based on generated trajectory
    x_err = mr.TransInv(x)@x_d
    x_err_bracket = mr.MatrixLog6(x_err)

    #compute desired twist for next time step
    V_d = (1/dt)*mr.MatrixLog6(mr.TransInv(x_d)@x_d_next)

    V_d_vec = mr.se3ToVec(V_d)
    x_err_vec = mr.se3ToVec(x_err_bracket)

    #compute the commanding twist based on the error, desired twist, proportional and integral gain terms, and the time step
    V = mr.Adjoint(x_err)@V_d_vec + Kp@x_err_vec + Ki@(x_err_vec*dt)

    return V


current_config = np.array([0,0,0,0,0,0.2,-1.6,0])

#compute configuration x matrix based on chassis and arm configuration and robot home transformations using forward kinematics
B_list = np.array([[0, 0, 1, 0, 0.033, 0], [0, -1, 0, -0.5076, 0, 0], [0, -1, 0, -0.3526, 0, 0], [0, -1, 0, -0.2176, 0, 0], [0, 0, 1, 0, 0, 0]]).T
thetalist = np.array([current_config[3], current_config[4], current_config[5], current_config[6], current_config[7]])
T_sb = np.array([[np.cos(current_config[0]), -np.sin(current_config[0]), 0, current_config[1]], [np.sin(current_config[0]), np.cos(current_config[0]), 0, current_config[2]], [0, 0, 1, 0.0963], [0,0,0,1]])
T_b0 = np.array([[1, 0, 0, 0.1662], [0, 1, 0, 0], [0, 0, 1, 0.0026], [0, 0, 0, 1]])
M_0e = np.array([[1, 0, 0, 0.033], [0, 1, 0, 0], [0, 0, 1, 0.6546], [0, 0, 0, 1]])
T_0e = mr.FKinBody(M_0e, B_list, thetalist)
T_chassis = T_sb@T_b0@M_0e
x = mr.FKinBody(T_chassis, B_list, thetalist)

#robot parameters
l = 0.235
w = 0.15
r = 0.0475

#compute 6 array of pseudoinverse of H(0)
F = (r/4)*np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)], [1, 1, 1, 1], [-1, 1, -1, 1]])
zero_array = np.array([0, 0, 0, 0])
F_6 = np.vstack((zero_array, zero_array, F, zero_array))

#compute jacobian of base and arm
Je_arm = mr.JacobianBody(B_list, thetalist)
adj_Je = (mr.Adjoint(mr.TransInv(T_0e)@mr.TransInv(T_b0)))
Je_base = adj_Je@F_6
Je = np.hstack((Je_base, Je_arm))

#desired configuration of current and next time step
x_d = np.array([[0, 0, 1, 0.5], [0, 1, 0, 0], [-1, 0, 0, 0.5], [0, 0, 0, 1]])
x_d_next = np.array([[0, 0, 1, 0.6], [0, 1, 0, 0], [-1, 0, 0, 0.3], [0, 0, 0, 1]])

#proportional and integral gains
Kp = np.array([[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 1]])
Ki = np.array([[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]])

dt = 0.01

V = FeedbackControl(x, x_d, x_d_next, Kp, Ki, dt)
Je_t = np.linalg.pinv(Je)

#end-effector twist for controls
V_e = Je_t@V
print(V_e)