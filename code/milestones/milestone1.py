import modern_robotics as mr
import numpy as np

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


dt = 0.01
# max_speed = 10
max_speed = 5

#initial configuration
current_config = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0])
gripper_state = 0

# controls = np.array([10,10,10,10,0,0,0,0,0])
# controls = np.array([-10,10,-10,10,0,0,0,0,0])
controls = np.array([-10,10,10,-10,0,0,0,0,0])


config_traj = NextState(current_config, controls, dt, max_speed, gripper_state)
print(config_traj)