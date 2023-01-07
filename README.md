The goal of this project was to implement a feedforward plus PI feedback controller, based on calculated odometry, for a simulated mobile Kuka youBot robot to follow a generated pick and place trajectory.

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/LQAwk1nPf0k/0.jpg)](https://www.youtube.com/watch?v=LQAwk1nPf0k)

Software: Python, CoppeliaSim

There were three steps to this project. First, I generated a desired end-effector trajectory to pick and place a block. Next, I then determined the robot chassis configuation, based on its odometry, and the arm configuration, from first-order Euler Integration, at each time step. Finally, I then implemented a feedforward plus PI feedback controller to correct the robots path if it deviates from the desired generated trajectory. The model of my feedforward plus PI feedback controller is shown below.

![PID_eqn](https://user-images.githubusercontent.com/87098227/208170586-fdfa0f09-c960-4b79-ab06-243c4e7074a1.png)

The feedback contoller outputs a twist which is then used, in combination with the pseudoinverse of the end-effector Jacobian, to find the wheel and arm joint speeds.

![Jacobian_eqn](https://user-images.githubusercontent.com/87098227/208170667-6a9553cc-211b-47c9-9895-cdb1a323c556.png)

These new speeds are used to calculate the next state the robot is in at the next time step. The importance of a well-tuned feedforward plus PI controller can be seen in the error graphs below, where the overshoot graph many rapid oscillations. This means that the robot would initially exhibit crazy, and in real life dangerous, motions as it tried to get back on track to its desired trajectory.

Well-tuned feedforward plus PI feedback controller

![Sindelar_Best_NoLimits](https://user-images.githubusercontent.com/87098227/208169399-5b6344ef-dcc1-44b5-b957-aea600fb3c40.png)


Overshoot due to a poorly-tuned feedforward plus PI feedback controller

![Sindelar_Overshoot_NoLimit](https://user-images.githubusercontent.com/87098227/208169528-6de048e2-8201-4e85-ae7a-f6fb77446a67.png)


To make the robot more realistic, I also implemented joint limits on 4 of the 5 arm joints. I chose only the first 4 out of the total 5 joints to constrain because in the desired trajectory I command a specific position for joint 5. Therefore, a constraint on joint 5 seemed to be an unnecessary addition that would overconstrain the problem. Adding the joint constraints forces the robot to plan trajectories to pick and place the block without any self-collisions or singularities. This process works by first planning the robot configuration at the next time step without executing. Then, the four specified joints are check to see if they remain between the upper and lower bounds of the limits. If not, the column of the Jacobian matrix corresponding to that joint is set to all zeros. This suggests to the controls that moving that specific joint won't cause any end-effector motion, thus the new controls won't command any motion for that specific joint. The video above demonstrates a well-tuned feedforward plus PI feedback controller without joint limits. For comparison, a video of a well-tuned feedforward plus PI feedback controller with joint limits is shown below.

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/VP_wHaPdKIw/0.jpg)](https://www.youtube.com/watch?v=VP_wHaPdKIw)

