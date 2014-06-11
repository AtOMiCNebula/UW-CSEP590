CSE P 590 Homework #3 (State Estimation with Extended Kalman Filter)
Jeff Weiner (jdweiner@cs.washington.edu)

Homework files submitted:
MujocoClient.exe

Compiler used: Visual Studio Express 2013 Ultimate
No additional dependencies.


Notes:

First off, it seems that due to the random noise that Mujoco adds, these
resulst are not perfectly repeatable like they have been for past assignments,
where the random number generator was always seeded the same way (or rather,
not seeded at all).  I've attempted to get representative averages here.

My submitted executable uses one iteration, though it is trivial to change
this (look for iEKFIterations).  Additionally, I did not attempt the extra
credit for part 2.

Using my program, I achieved the following scores with the following error
values and iteration counts:

                      Iterations      Hits
R=.05,.05,.30           1              10
                       10              10
                       50              10
R=1.0,1.0,1.0           1               8
                       10               7
                       50               8
R=1.0,1.0,10.0          1               3
                       10               5
                       50               4

Increasing EKF iterations did not appear to make an appreciable difference on
the outcome, likely because Iterative EKF helps more when experiencing noise
which is not gaussian-based.

When varying the initial state covariance matrix, we see the following results:

                      Iterations      Hits
alpha=1e0               1              10
                       10              10
                       50              10
alpha=1e-2              1               7
                       10               7
                       50               7
alpha=1e2               1              10
                       10              10
                       50              10

As a larger value of alpha is used, it seems that the estimator converges much
quicker on the actual object location.  With smaller values, it converges
slower, sometimes not quickly enough to let the paddle accurately return the
ball.

Lastly, there appears to be a bug in the hit-detection: if the paddle impacts
the floor in the model, it is counted as a "hit".  I've attempted to not count
these false hits in my results above.
