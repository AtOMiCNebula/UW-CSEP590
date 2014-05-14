CSE P 590 Homework #1 (Jacobian Control Methods)
Jeff Weiner (jdweiner@cs.washington.edu)

Homework files submitted:
MujoClient1.cpp/.exe - Position Control, using Pseudo Inverse
MujoClient2.cpp/.exe - Orientation Control, using Jacobian Transpose
MujoClient3.cpp/.exe - Composition, using Jacobian Transpose
MujoClient4.cpp/.exe - Picking up an object, using Pseudo Inverse

Compiler used: Visual Studio Express 2012 (for Windows Desktop)
No additional dependencies.


Notes:

All submitted assignment code files are identical, except for some changed
variables to alter which modes are active, such as:
* fJacobian (true for Jacobian Transpose, false for Pseudo Inverse)
* iEnabledControlMethods (0x1 for part 1, 0x2 for part 2, 0x3 for both)
* fPart4StateMachine (true to enable the object movement controller)

Jacobian Transpose takes a bit longer to converge than Pseudo Inverse does, but
it still reaches the same final result!

The object is not placed over the target in a pixel perfect manner, but it is
still very close (hard to measure "1-2mm" as was suggested on the forum).  This
difference is likely due to either how I offset the wrist away from the object
to avoid pushing it away (a simple X offset), or over-gripping the object and
it springing away slightly upon release due to contact forces.

I found that as smaller alpha values are used, the simulation takes longer, as
each movement interval is smaller.  However, the movement is also smoother, and
less prone to errors.  When larger alpha values are used, the simulation moves
more quickly, but for too large values, the simulation can move too far in one
step, and end up having to backtrack.
