CSE P 590 Homework #2 (Motion Planning)
Jeff Weiner (jdweiner@cs.washington.edu)

Homework files submitted:
MujocoClient.exe

Compiler used: Visual Studio Express 2012 (for Windows Desktop)
No additional dependencies.


Notes:

Provided executable is pre-populated with N=200, and K=50.  The shortest paths
are calculated using Dijkstra's Algorithm.  Additionally, I am not seeding the
random number generator, so the following results should be easily repeatable.

Here's a rough chart of intermediate steps that needed to be taken to get from
each init position to the goal position, using K=10 and a variety of values for
N:
		 N=25	 N=50	 N=75	N=100	N=150	N=200	N=250
qinit0		    2	    2	    2	    3	    4	    4	    3
qinit1		    3	    2	    2	    2	    2	    3	    2
qinit2		 none	    2	    2	    2	    2	    4	    4

We see that while too-low values of N can yield an inability to determine a
path, there also exist too-high values as well, where path sizes begin to grow
beyond their small lengths with values like N=50 and N=75.  This is likely due
to the constant K that ends up eliminating valid neighbor paths because the
paths are simply longer than other neighboring paths found.  As such, we end up
with a higher amount of steps, each one taking a smaller distance.

This time, holding N constant at N=200 and varying K:
		  K=5	 K=10	 K=15	 K=25	 K=50	K=100
qinit0		    5	    4	    4	    2	    1	    1
qinit1		    4	    3	    3	    2	    2	    2
qinit2		    4	    4	    2	    2	    2	    2

This confirms the above suspicion, that holding K low while increasing N can
yield inefficient paths.  When increasing N, K must also be raised as well.

The best paths I found are as follows, using N=200 and K=50:
	Solution for qinit[0]: 200 --> 45 --> 203
	Solution for qinit[1]: 201 --> 62 --> 194 --> 203
	Solution for qinit[2]: 202 --> 47 --> 8 --> 203
