This case uses a feedforward plus PI controller. For this controller,
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

The initial conditions were:
current_config = np.array([np.pi/4,0,0.23,0,-0.25,-0.25,-0.25,0,0,0,0,0,0])

This case also contains two separate parts, one with joint limits implemented and the other without.
Both cases used the same proportional and integral gain terms, and the same initial conditions.
The videos and graphs are named accordingly, specifying which one has the joint limits and which
does not. 