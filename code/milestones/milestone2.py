import modern_robotics as mr
import numpy as np

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