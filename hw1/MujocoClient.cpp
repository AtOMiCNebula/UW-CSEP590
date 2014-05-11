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

// sets thetahat such that gripper fingers open or close by gripAmount
// gripAmount = 0: default gipper posture
// gripAmount > 0: gripper fingers opened
// gripAmount < 0: gripper fingers closed
// between -1 and +1 are acceptable gripAmount values
void setGrip(double gripAmount, Vector &thetahat)
{
	// indices for joints of two gripper fingers
	static const int L_GRIP_JOINT_INDEX = 7;
	static const int R_GRIP_JOINT_INDEX = 8;

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
			// current hand orientation
			Quat r(geoms.quat + 4*HAND_GEOM_INDEX);
			// target hand orientation
			Quat rhat(geoms.quat + 4*TARGET_GEOM_INDEX);

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
			
			// Part 1 (Position Control)
			// thetaHat = theta + delta_theta
			Vector delta_theta1(dimtheta);
			if (fJacobian)
			{
				// Jacobian Transpose method
				// delta_theta = alpha * J^t(theta) * (xHat - x)

				delta_theta1 = computeJacobianTranspose(alpha, Jpos, (xhat - x));
			}
			else
			{
				// Pseudo Inverse method
				// delta_theta = alpha * J^# * delta_x + (I - J^# * J)(theta_0 - theta)

				delta_theta1 = computePseudoInverse(alpha, Jpos, (xhat - x), theta, I);
			}

			// Part 2 (Orientation Control)
			const Quat Qrot90(cos(M_PI_4), sin(M_PI_4), 0, 0);
			Vector delta_theta2(dimtheta);
			if (fJacobian)
			{
				// Jacobian Transpose method

				delta_theta2 = computeJacobianTranspose(alpha, Jrot, quatdiff(r, rhat*Qrot90));
			}
			else
			{
				// Pseudo Inverse method
			}

			thetahat = theta + delta_theta1 + delta_theta2;

			// set target DOFs to thetahat and advance simulation
			mjSetControl(dimtheta, thetahat);
			mjStep2();
		}

		mjDisconnect();
	}
	mjClear();
}
