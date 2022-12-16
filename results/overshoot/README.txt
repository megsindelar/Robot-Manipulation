This case uses a feedforward plus PI controller. This case also contains two separate parts, one with joint limits implemented and the other without. For this controller, I have
different gains depending on whether the robot has joint limits. But, for both
of these cases, I change the initial generated trajectory end-effector position
from 0.25 to 0.5, as seen in my T_se_init transformation in my code.

For the case without joint limits, the proportional gain is: 
    Kp = [[30, 0, 0, 0, 0, 0]
          [0, 30, 0, 0, 0, 0]
          [0, 0, 30, 0, 0, 0]
          [0, 0, 0, 30, 0, 0]
          [0, 0, 0, 0, 30, 0]
          [0, 0, 0, 0, 0, 30]]
and the integral gain is:
    Ki = [[14, 0, 0, 0, 0, 0]
          [0, 14, 0, 0, 0, 0]
          [0, 0, 14, 0, 0, 0]
          [0, 0, 0, 14, 0, 0]
          [0, 0, 0, 0, 14, 0]
          [0, 0, 0, 0, 0, 14]]
          

For the case with joint limits, the proportional gain is: 
    Kp = [[2, 0, 0, 0, 0, 0]
          [0, 2, 0, 0, 0, 0]
          [0, 0, 2, 0, 0, 0]
          [0, 0, 0, 2, 0, 0]
          [0, 0, 0, 0, 2, 0]
          [0, 0, 0, 0, 0, 2]]
and the integral gain is:
    Ki = [[1.2, 0, 0, 0, 0, 0]
          [0, 1.2, 0, 0, 0, 0]
          [0, 0, 1.2, 0, 0, 0]
          [0, 0, 0, 1.2, 0, 0]
          [0, 0, 0, 0, 1.2, 0]
          [0, 0, 0, 0, 0, 1.2]]

The initial conditions for both cases were:
current_config = np.array([np.pi/4,0,0.23,0,-0.25,-0.25,-0.25,0,0,0,0,0,0])

The videos and graphs are named accordingly, specifying which one has the joint limits and which
does not. 
