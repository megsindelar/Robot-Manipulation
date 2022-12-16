import modern_robotics as mr
import numpy as np
import matplotlib.pyplot as plt

"""
To run this code:


"""

def NextState(current_config, controls, dt, max_speed, gripper_state):
    """
    Function to compute the configuration at the next time step for the robot

    Args:
        current_config: current configuration of robot
        controls: joint and wheel speeds
        dt: time step
        max_speed: maximum allowable speed of robot
        gripper_state: whether gripper is open or closed


    Returns
    -------
        config_traj: robot configuration
    """

    #dimensions of the robot
    l = 0.235
    w = 0.15
    r = 0.0475

    #pseudoinverse of H(0)
    F = np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)], [1, 1, 1, 1], [-1, 1, -1, 1]])

    #robot base configuration variables
    phi = current_config[0]
    x = current_config[1]
    y = current_config[2]
    
    #check if commanded speeds are over the max speed limits
    for i in range(len(controls)):
        if controls[i] > max_speed:
            controls[i] = max_speed
    
    #compute new joint angles using Euler integration
    J1 = current_config[3] + controls[4]*dt
    J2 = current_config[4] + controls[5]*dt
    J3 = current_config[5] + controls[6]*dt
    J4 = current_config[6] + controls[7]*dt
    J5 = current_config[7] + controls[8]*dt

    #compute new wheel configurations using Euler integration
    W1 = current_config[8] + controls[0]*dt
    W2 = current_config[9] + controls[1]*dt
    W3 = current_config[10] + controls[2]*dt
    W4 = current_config[11] + controls[3]*dt

    #array of wheel speeds
    u_dot = np.array([[controls[0]], [controls[1]], [controls[2]], [controls[3]]])

    #body twist using pseudoinverse of H(0)
    Vb = (r/4)*F@u_dot

    w_bz = Vb[0][0]
    v_bx = Vb[1][0]
    v_by = Vb[2][0]

    #use Euler integration to compute new chassis configuration
    #small change in config depends on if w_bz is 0
    if w_bz == 0:
        delta_qb = np.array([0, v_bx, v_by])
    else:
        delta_qb = np.array([w_bz, (((v_bx*np.sin(w_bz)) + (v_by*(np.cos(w_bz) - 1)))/w_bz), (((v_by*np.sin(w_bz)) + (v_bx*(1 - np.cos(w_bz))))/w_bz)])

    m = np.array([[1, 0, 0], [0, np.cos(phi), -np.sin(phi)], [0, np.sin(phi), np.cos(phi)]])
    dq = m@delta_qb
    q = np.array([current_config[0], current_config[1], current_config[2]])
    q_new = q + dq*dt

    #new robot configuration, consisiting of chassis config, joint angles, wheel config, and gripper state
    config_traj = np.array([q_new[0], q_new[1], q_new[2], J1, J2, J3, J4, J5, W1, W2, W3, W4, gripper_state])

    return config_traj



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

    return V, x_err_vec

def testJointLimits(Ve, Je_arm):
    """
    Function to test any joints will violate joint limits

    Args:
        Ve: computed end-effector twist for robot controls
        Je_arm: Jacobian of the arm relative to the end-effector


    Returns
    -------
        Je_arm: the updated Jacobian of the arm relative to the end-effector
    """

    controls = np.array([Ve[0], Ve[1], Ve[2], Ve[3], Ve[4], Ve[5], Ve[6], Ve[7], Ve[8]])
    #test these controls to see if will hit joint limits with new config
    test_current_config = NextState(current_config, controls, dt, max_speed, config_d_next)
    joints = [test_current_config[3], test_current_config[4], test_current_config[5], test_current_config[6], test_current_config[7]]
    #set collison to true to check all limited joints (Joints 1, 2, 3, and 4)
    collision = 1
    for i in range(10):
      collision = 0
      #check Joint 1
      if joints[0] > 0.4:
        for i in range(len(Je_arm)):
          #if collision, set entire column of Jacobian for joint 1 to be 0
          Je_arm[i][0] = 0
          collision = 1
      elif joints[0] < -0.4:
        for i in range(len(Je_arm)):
          #if collision, set entire column of Jacobian for joint 1 to be 0
          Je_arm[i][0] = 0
          collision = 1

      #check Joint 2
      if joints[1] > -0.1:
        for i in range(len(Je_arm)):
          #if collision, set entire column of Jacobian for joint 2 to be 0
          Je_arm[i][1] = 0
          collision = 1
      elif joints[1] < -1.6:
        for i in range(len(Je_arm)):
          #if collision, set entire column of Jacobian for joint 2 to be 0
          Je_arm[i][1] = 0
          collision = 1

      #check Joint 3
      if joints[2] > -0.1:
        for i in range(len(Je_arm)):
          #if collision, set entire column of Jacobian for joint 3 to be 0
          Je_arm[i][2] = 0
          collision = 1
      elif joints[2] < -2.5:
        for i in range(len(Je_arm)):
          #if collision, set entire column of Jacobian for joint 3 to be 0
          Je_arm[i][2] = 0
          collision = 1

      #check Joint 4
      if joints[3] > -0.1:
        for i in range(len(Je_arm)):
          #if collision, set entire column of Jacobian for joint 4 to be 0
          Je_arm[i][3] = 0
          collision = 1
      elif joints[3] < -2.5:
        for i in range(len(Je_arm)):
          #if collision, set entire column of Jacobian for joint 4 to be 0
          Je_arm[i][3] = 0
          collision = 1

      #recompute Jacobian to check if another joint violation occurs
      adj_Je = (mr.Adjoint(mr.TransInv(T_0e)@mr.TransInv(T_b0)))
      Je_base = adj_Je@F_6
      Je = np.hstack((Je_base, Je_arm))
      Je_t = np.linalg.pinv(Je)
      #recompute end-effector twist to check if another joint violation occurs
      V_e = Je_t@V
      controls = np.array([Ve[0], Ve[1], Ve[2], Ve[3], Ve[4], Ve[5], Ve[6], Ve[7], Ve[8]])
      #recompute new configuration to check if another joint violation occurs
      test_current_config = NextState(current_config, controls, dt, max_speed, config_d_next)
      joints = [test_current_config[3], test_current_config[4], test_current_config[5], test_current_config[6], test_current_config[7]]

    return Je_arm

def TrajectoryGenerator2(T_se_init, T_sc_init, T_sc_final, T_ce_grasp, T_ce_standoff, k):
    """
    Generate a reference trajectory for the end-effector frame {e}.

    Args:
        T_se_init: Transformation of initial position from world to end-effector frame
        T_sc_init: Transformation of initial position from world to cube frame
        T_sc_final: Transformation of final position from world to cube frame
        T_ce_grasp: Transformation of grasp position from cube to end-effector frame
        T_ce_standoff: Transformation of standoff position from cube to end-effector frame
        k: Number of trajectory reference configurations per 0.01 seconds


    Returns
    -------
        trajectory: full trajectory for the end-effector frame
    """

    N = 1000

    # move end effector across plane from initial position to initial cube standoff position
    # calculate transform standoff relative to world frame
    T_se_standoff = np.matmul(T_sc_init, T_ce_standoff)

    traj1 = mr.ScrewTrajectory(Xstart= T_se_init, Xend= T_se_standoff, Tf=((N/4)/100) , N=N , method=5)
    grippers1 = np.zeros(len(traj1))

    # move end effector down from standoff to grasp position
    T_se_grasp = np.matmul(T_sc_init, T_ce_grasp)
    traj2 = mr.CartesianTrajectory(Xstart= T_se_standoff, Xend= T_se_grasp, Tf=((N/8)/100) , N=N/4 , method=5)
    grippers2 = np.zeros(len(traj2))

    # close grippers (add a few copies of the final grasp trajectory position to wait for grippers to close)
    traj3 = mr.CartesianTrajectory(Xstart= T_se_grasp, Xend= T_se_grasp, Tf=2 , N=30 , method=5)
    grippers3 = np.ones(len(traj3))

    # move the gripper (which is grasping the cube) back up from grasp to standoff position
    T_se_grasp = np.matmul(T_sc_init, T_ce_grasp)
    traj4 = mr.CartesianTrajectory(Xstart= T_se_grasp, Xend= T_se_standoff, Tf=((N/8)/100) , N=N/4 , method=5)
    grippers4 = np.ones(len(traj4))

    # move gripper and cube across the plane to the final cube standoff position
    T_final_se_standoff = np.matmul(T_sc_final, T_ce_standoff)
    traj5 = mr.ScrewTrajectory(Xstart= T_se_standoff, Xend= T_final_se_standoff, Tf=((N/4)/100) , N=N , method=5)
    grippers5 = np.ones(len(traj5))

    # move gripper and cube down from standoff position to grasp position to place cube on floor
    T_final_se_grasp = np.matmul(T_sc_final, T_ce_grasp)
    traj6 = mr.CartesianTrajectory(Xstart= T_final_se_standoff, Xend= T_final_se_grasp, Tf=((N/8)/100) , N=N/4 , method=5)
    grippers6 = np.ones(len(traj6))

    # open grippers (add a few copies of the final grasp trajectory position to wait for grippers to open)
    traj7 = mr.CartesianTrajectory(Xstart= T_final_se_grasp, Xend= T_final_se_grasp, Tf=2 , N=30 , method=5)
    grippers7 = np.zeros(len(traj7))

    # move grippers back up from final grasp position to final standoff position
    T_final_se_grasp = np.matmul(T_sc_final, T_ce_grasp)
    traj8 = mr.CartesianTrajectory(Xstart= T_final_se_grasp, Xend= T_final_se_standoff, Tf=((N/8)/100) , N=N/4 , method=5)
    grippers8 = np.zeros(len(traj8))

    trajectory = np.concatenate((traj1, traj2, traj3, traj4, traj5, traj6, traj7, traj8))
    grippers = np.concatenate((grippers1, grippers2, grippers3, grippers4, grippers5, grippers6, grippers7, grippers8))

    return trajectory, grippers


#set end-effector angle
angle = (3*np.pi)/4

#initial and final transforms for generating trajectory
T_se_init = np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0.25],[0,0,0,1]])
T_sc_init = np.array([[1,0,0,1],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
T_sc_final = np.array([[0,1,0,0],[-1,0,0,-1],[0,0,1,0.025],[0,0,0,1]])
T_ce_grasp = np.array([[np.cos(angle),0,np.sin(angle),0],[0,1,0,0],[-np.sin(angle),0,np.cos(angle),0.025],[0,0,0,1]])
T_ce_standoff = np.array([[np.cos(angle),0,np.sin(angle),0],[0,1,0,0],[-np.sin(angle),0,np.cos(angle),0.05],[0,0,0,1]])
k = 1

#call function to generate trajectory of grippers
traj, grippers = TrajectoryGenerator2(T_se_init, T_sc_init, T_sc_final, T_ce_grasp, T_ce_standoff, k)

#robot configuration parameters
B_list = np.array([[0, 0, 1, 0, 0.033, 0], [0, -1, 0, -0.5076, 0, 0], [0, -1, 0, -0.3526, 0, 0], [0, -1, 0, -0.2176, 0, 0], [0, 0, 1, 0, 0, 0]]).T
M_0e = np.array([[1, 0, 0, 0.033], [0, 1, 0, 0], [0, 0, 1, 0.6546], [0, 0, 0, 1]])

dt = 0.01
max_speed = 10

N = len(traj)

#initial configuration
current_config = np.array([np.pi/4,0,0.23,0,-0.25,-0.25,-0.25,0,0,0,0,0,0])
config_total_traj = np.zeros((int(N),13))

l = 0.235
w = 0.15
r = 0.0475
F = (r/4)*np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)], [1, 1, 1, 1], [-1, 1, -1, 1]])
zero_array = np.array([0, 0, 0, 0])
F_6 = np.vstack((zero_array, zero_array, F, zero_array))

#Proportional and integral gain terms
Kp = 1.7*np.identity(6)
Ki = 0.001*np.identity(6)

#initial gripper state
config_d_next = grippers[0]

#lists to store configuration errors for plotting
full_x_err_list = np.zeros(((N - 1), 6))
roll_err_list = np.zeros(((N - 1), 6))
pitch_err_list = np.zeros(((N - 1), 6))
yaw_err_list = np.zeros(((N - 1), 6))
x_err_list = np.zeros(((N - 1), 6))
y_err_list = np.zeros(((N - 1), 6))
z_err_list = np.zeros(((N - 1), 6))

for i in range(N - 1):
    robot_config = [current_config[0], current_config[1], current_config[2]]
    thetalist = [current_config[3], current_config[4], current_config[5], current_config[6], current_config[7]]
    
    #compute configuration x matrix based on chassis and arm configuration and robot home transformations using forward kinematics
    T_sb = np.array([[np.cos(current_config[0]), -np.sin(current_config[0]), 0, current_config[1]], [np.sin(current_config[0]), np.cos(current_config[0]), 0, current_config[2]], [0, 0, 1, 0.0963], [0,0,0,1]])
    T_b0 = np.array([[1, 0, 0, 0.1662], [0, 1, 0, 0], [0, 0, 1, 0.0026], [0, 0, 0, 1]])
    T_0e = mr.FKinBody(M_0e, B_list, thetalist)
    T_chassis = T_sb@T_b0@M_0e
    x = mr.FKinBody(T_chassis, B_list, thetalist)

    #compute the Jacobian of the chassis and arm
    Je_arm = mr.JacobianBody(B_list, thetalist)
    adj_Je = (mr.Adjoint(mr.TransInv(T_0e)@mr.TransInv(T_b0)))
    Je_base = adj_Je@F_6
    Je = np.hstack((Je_base, Je_arm))

    #set desired configuration for current and next time step based on generated trajectory
    x_d = traj[i]
    x_d_next = traj[i+1]

    #compute end-effector commanding control twist based on feedback control
    V, x_err = FeedbackControl(x, x_d, x_d_next, Kp, Ki, dt)
    Je_t = np.linalg.pinv(Je)
    V_e = Je_t@V

    #test for joint limit violations and re-compute end-effector commanding control twist
    Je_arm = testJointLimits(V_e, Je_arm)
    adj_Je = (mr.Adjoint(mr.TransInv(T_0e)@mr.TransInv(T_b0)))
    Je_base = adj_Je@F_6
    Je = np.hstack((Je_base, Je_arm))
    Je_t = np.linalg.pinv(Je)
    V_e = Je_t@V

    controls = np.array([V_e[0], V_e[1], V_e[2], V_e[3], V_e[4], V_e[5], V_e[6], V_e[7], V_e[8]])

    #store the current configuration for a total trajectory of the robot
    config_total_traj[i,:] = current_config
    config_d_next = grippers[i+1]

    #compute the next robot configuration
    current_config = NextState(current_config, controls, dt, max_speed, config_d_next)

    #store the configuration errors for plotting
    full_x_err_list[i,:] = x_err
    roll_err_list[i, 0] = x_err[0]
    pitch_err_list[i, 1] = x_err[1]
    yaw_err_list[i, 2] = x_err[2]
    x_err_list[i, 3] = x_err[3]
    y_err_list[i, 4] = x_err[4]
    z_err_list[i, 5] = x_err[5]

print("Generating animation csv file")
np.savetxt("robot_config_best_limits.csv", config_total_traj, delimiter = ",")
np.savetxt("x_error_best_limits.csv", full_x_err_list, delimiter = ",")


#Plot the configuration errors over time
print("Writing error plot data")
fig, axs = plt.subplots(2,3)
axs[0,0].plot(roll_err_list)
axs[0,0].set_title('Roll Error')
axs[0,1].plot(pitch_err_list)
axs[0,1].set_title('Pitch Error')
axs[0,2].plot(yaw_err_list)
axs[0,2].set_title('Yaw Error')
axs[1,0].plot(x_err_list)
axs[1,0].set_title('X Error')
axs[1,1].plot(y_err_list)
axs[1,1].set_title('Y Error')
axs[1,2].plot(z_err_list)
axs[1,2].set_title('Z Error')
fig.tight_layout()
plt.show()
print("Done.")