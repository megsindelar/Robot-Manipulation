This case also contains two separate parts, one with joint limits implemented and the other without. This case uses a feedforward plus PI controller. For this controller,
the proportional gain is: 
    Kp = [[1.7, 0, 0, 0, 0, 0]
          [0, 1.7, 0, 0, 0, 0]
          [0, 0, 1.7, 0, 0, 0]
          [0, 0, 0, 1.7, 0, 0]
          [0, 0, 0, 0, 1.7, 0]
          [0, 0, 0, 0, 0, 1.7]]
and the integral gain is:
    Ki = [[0.001, 0, 0, 0, 0, 0]
          [0, 0.001, 0, 0, 0, 0]
          [0, 0, 0.001, 0, 0, 0]
          [0, 0, 0, 0.001, 0, 0]
          [0, 0, 0, 0, 0.001, 0]
          [0, 0, 0, 0, 0, 0.001]]

Both cases used the same proportional and integral gain terms, and the same initial conditions. 

The initial conditions were:
current_config = np.array([np.pi/4,0,0.23,0,-0.25,-0.25,-0.25,0,0,0,0,0,0])


For both of these cases, I change the initial generated trajectory end-effector position from 0.25 to 0.5, as seen in my T_se_init transformation in my code. Additionally, for both cases I changed the initial and final positions of the block, as seen in my T_sc_init and T_sc_final transformations in my code. I changed the initial block position for both cases to be at x = -0.5 and y = 1.0. The final block position for both cases was at x = 1.0 and y = -0.5. The videos and graphs are named accordingly, specifying which one has the joint limits and which
does not. 
