// mujoco communication interface
#include "comm.h"

// algebra classes
#define _USE_MATH_DEFINES
#include <cmath>
#include "GMatrix.h"
#include "GVector.h"
#include "Quaternion.h"
#include "windows.h"

typedef GVector<double> Vector;
typedef GMatrix<double> Matrix;
typedef Quaternion<double> Quat;

// indices for joints of two gripper fingers
static const int L_GRIP_JOINT_INDEX = 7;
static const int R_GRIP_JOINT_INDEX = 8;

// sets thetahat such that gripper fingers open or close by gripAmount
// gripAmount = 0: default gipper posture
// gripAmount > 0: gripper fingers opened
// gripAmount < 0: gripper fingers closed
// between -1 and +1 are acceptable gripAmount values
void setGrip(double gripAmount, Vector &thetahat)
{
	thetahat[L_GRIP_JOINT_INDEX] = -gripAmount;
	thetahat[R_GRIP_JOINT_INDEX] = +gripAmount;
}

// Jacobian Transpose method
// delta_theta = alpha * J^t(theta) * delta_x
Vector computeJacobianTranspose(double alpha, const Matrix &J, const Vector &delta_x)
{
	return (alpha * J.transpose() * delta_x);
}

// Pseudo Inverse method
// delta_theta = alpha * J^# * delta_x + (I - J^# * J)(theta_0 - theta)
Vector computePseudoInverse(double alpha, const Matrix &J, const Vector &delta_x, const Vector &theta, const Matrix &I)
{
	const Vector thetaNaught(theta.getSize());
	Matrix Jsharp = (J.transpose() * (J * J.transpose()).inverse());
	return ((alpha * Jsharp * delta_x) + ((I - (Jsharp * J)) * (thetaNaught - theta)));
}

// Part 4 State Machine Stuff
enum PART4STATE
{
	STATE_INIT,         // start opening grip
	STATE_MOVETOOBJECT, // move hand to object
	STATE_CLOSEGRIP,    // start closing grip
	STATE_MOVETOTARGET, // move hand to target
	STATE_DONE,         // open hand, and we're done!
	STATE_DONEANIM1,    // "done" animation, part 1
	STATE_DONEANIM2,    // "done" animation, part 2
};

void StateMachineCheckEndCondition(PART4STATE &stateCurrent, PART4STATE stateNext, Vector vCheck, double dEpsilon)
{
	static bool fLastCheckInited = false;
	static double dLastCheck = 0;

	double dCheck = 0;
	for (int i = 0; i < vCheck.getSize(); i++)
	{
		dCheck += vCheck[i]*vCheck[i];
	}

	if (!fLastCheckInited || dCheck > dEpsilon)
	{
		fLastCheckInited = true;
		dLastCheck = dCheck;
	}
	else
	{
		stateCurrent = stateNext;
		fLastCheckInited = false;
	}
}

void main(void)
{
	// indices for nodes in the scene
	static const int TARGET_GEOM_INDEX = 1;
	static const int HAND_GEOM_INDEX = 4;
	static const int OBJECT_GEOM_INDEX = 13; 

	// connect to mujoco server
	mjInit();
	mjConnect(10000);

	// load hand model
	if(mjIsConnected())
	{
		mjLoad("hand.xml");
		Sleep(1000);  // wait till the load is complete
	}

	if(mjIsConnected() && mjIsModel())
	{
		mjSetMode(2);
		mjReset();

		// size containts model dimensions
		mjSize size = mjGetSize();

		// number of arm degress of freedom (DOFs) and controls
		// (does not include target object degrees of freedom)
		int dimtheta = size.nu;

		// identity matrix (useful for implementing Jacobian pseudoinverse methods)
		Matrix I(dimtheta, dimtheta);
		I.setIdentity();

		// target arm DOFs
		Vector thetahat(dimtheta);
		thetahat.setConstant(0.0);

		for(;;) // run simulation forever
		{
			// simulation advance substep
			mjStep1();

			mjState state = mjGetState();
			// current arm degrees of freedom
			Vector theta(dimtheta);
			// state.qpos contains DOFs for the whole system. Here extract
			// only DOFs for the arm into theta			
			for(int e=0; e<dimtheta; e++)
			{
				theta[e] = state.qpos[e];
			}

			mjCartesian geoms = mjGetGeoms();
			// current hand position
			Vector x(3, geoms.pos + 3*HAND_GEOM_INDEX);
			// target hand position
			Vector xhat(3, geoms.pos + 3*TARGET_GEOM_INDEX);
			Vector xhat_object(3, geoms.pos + 3*OBJECT_GEOM_INDEX);
			// current hand orientation
			Quat r(geoms.quat + 4*HAND_GEOM_INDEX);
			// target hand orientation
			Quat rhat(geoms.quat + 4*TARGET_GEOM_INDEX);
			Quat rhat_object(geoms.quat + 4*OBJECT_GEOM_INDEX);

			mjJacobian jacobians = mjJacGeom(HAND_GEOM_INDEX);
			// current hand position Jacobian
			Matrix Jpos(3,jacobians.nv, jacobians.jacpos);
			// current hand orientation Jacobian
			Matrix Jrot(3,jacobians.nv, jacobians.jacrot);
			// extract only columns of the Jacobian that relate to arm DOFs
			Jpos = Jpos.getBlock(0,3, 0, dimtheta);
			Jrot = Jrot.getBlock(0,3, 0, dimtheta);

			// -- your code goes here --
			// set thetahat through Jacobian control methods here
			// for part 4 of assignment, you may need to call setGrip(amount, thetahat) here
			// thetahat should be filled by this point
			const bool fJacobian = true;
			const double alpha = 0.01;
			const bool fPart4StateMachine = true;
			static int iEnabledControlMethods = 0x3;
			static double dGrip = 0;
			Quat rhat_offset(1, 0, 0, 0);

			// Part 4 (Picking Up an Object)
			if (fPart4StateMachine)
			{
				static PART4STATE eState = STATE_INIT;

				switch (eState)
				{
				case STATE_INIT:
					// To start things off, let's disable movement and open the gripper
					iEnabledControlMethods = 0x0;
					dGrip = 1;
					eState = STATE_MOVETOOBJECT;
					break;

				case STATE_MOVETOOBJECT:
					// Change rhat/xhat to point to the object instead of the target
					iEnabledControlMethods = 0x3;
					xhat = xhat_object;
					rhat = rhat_object;

					// Apply a 90 degree rotation about the x axis, to align grippers with object
					rhat_offset = Quat(cos(M_PI_4), sin(M_PI_4), 0, 0);

					// Notably, we do not want our wrist to try to position itself in the center of
					// the object, just in front of it a little bit.  So, offset xhat a little bit!
					xhat[0] -= 0.05;

					// Are we there yet?
					StateMachineCheckEndCondition(eState, STATE_CLOSEGRIP, (xhat-x), 0.000005);
					break;

				case STATE_CLOSEGRIP:
					// Now, close the grip on the object!
					iEnabledControlMethods = 0x0;
					dGrip = -1;
					eState = STATE_MOVETOTARGET;
					break;

				case STATE_MOVETOTARGET:
					// Now move the object to the target!
					iEnabledControlMethods = 0x3;

					// Apply a 90 degree rotation about the x axis, to align grippers with target
					rhat_offset = Quat(cos(M_PI_4), sin(M_PI_4), 0, 0);

					// Just like when we moved to the object, we need to offset xhat the same amount so that
					// it ends up in the expected spot!
					xhat[0] -= 0.05;

					// Are we there yet?
					StateMachineCheckEndCondition(eState, STATE_DONE, (xhat-x), 0.000005);
					break;

				case STATE_DONE:
					// Done!  Stop moving, and open the gripper
					iEnabledControlMethods = 0x0;
					dGrip = 1;
					eState = STATE_DONEANIM1;
					break;

				case STATE_DONEANIM1:
					// Have some fun, and slowly close grip to make a waving animation
					dGrip -= 0.0005;

					if (dGrip <= 0.25)
					{
						eState = STATE_DONEANIM2;
					}
					break;

				case STATE_DONEANIM2:
					// Have some fun, and slowly open grip to make a waving animation 
					dGrip += 0.0005;

					if (dGrip >= 1)
					{
						eState = STATE_DONEANIM1;
					}
					break;
				}
			}
			
			// Create our (appropriately sized!) targets to compose into
			const bool fComposingBoth = ((iEnabledControlMethods & 0x3) == 0x3);
			Matrix Jcomposed(Jpos.getNumRows() * (fComposingBoth ? 2 : 1), Jpos.getNumCols());
			Vector deltaXcomposed(Jpos.getNumRows() * (fComposingBoth ? 2 : 1));

			// Part 1 (Position Control)
			if (iEnabledControlMethods & 0x1)
			{
				const Vector delta_x = (xhat - x);

				assert(delta_x.getSize() == Jpos.getNumRows());
				for (int i = 0; i < Jpos.getNumRows(); i++)
				{
					Jcomposed.setRow(i, Jpos.getRow(i));
					deltaXcomposed[i] = delta_x[i];
				}
			}

			// Part 2 (Orientation Control)
			if (iEnabledControlMethods & 0x2)
			{
				Vector delta_r = quatdiff(r, rhat*rhat_offset);
				
				const int rowOffset = (fComposingBoth ? Jpos.getNumRows() : 0);
				assert(delta_r.getSize() == Jrot.getNumRows());
				for (int i = 0; i < Jrot.getNumRows(); i++)
				{
					Jcomposed.setRow(rowOffset+i, Jrot.getRow(i));
					deltaXcomposed[rowOffset+i] = delta_r[i];
				}
			}

			// Part 3 (Combined Position and Orientation Control)
			// thetaHat = theta + delta_theta
			if (iEnabledControlMethods != 0x0)
			{
				// Only try to compute the transforms if there's something to do!
				if (fJacobian)
				{
					thetahat = theta + computeJacobianTranspose(alpha, Jcomposed, deltaXcomposed);
				}
				else
				{
					thetahat = theta + computePseudoInverse(alpha, Jcomposed, deltaXcomposed, theta, I);
				}
			}
			setGrip(dGrip, thetahat);

			// set target DOFs to thetahat and advance simulation
			mjSetControl(dimtheta, thetahat);
			mjStep2();
		}

		mjDisconnect();
	}
	mjClear();
}
