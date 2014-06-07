// mujoco communication interface
#include "comm.h"

// algebra classes
#include "GMatrix.h"
#include "GVector.h"
#include "Quaternion.h"
#include "windows.h"

typedef GVector<double> Vector;
typedef GMatrix<double> Matrix;
typedef Quaternion<double> Quat;

// paddle geom ID
static const int PADDLE_GEOM = 4;
// number of trial keyframes in the model
static const int NUM_TRIAL = 10;
// simulation timestep size
static const double dt = 1e-3;

// model dynamics function
// given x{k-1} returns x{k} = f(x{k-1}) and its Jacobian J_f(x{k-1})
// J_f is |x|*|x| matrix
void f(const Vector &xprev, Vector *x, Matrix *Jfxprev)
{
	// acceleration due to gravity
	static const double a[3] = { 0.0, 0.0, -2.0 };

	// generate linear dynamics matrices
	Matrix A(6, 6);
	for (int r = 0; r < 3; r++)
		for (int c = 0; c < 3; c++)
		{
		A(0 + r, 0 + c) = (r == c) ? 1.0 : 0.0;
		A(3 + r, 3 + c) = (r == c) ? 1.0 : 0.0;
		A(0 + r, 3 + c) = (r == c) ? dt : 0.0;
		A(3 + r, 0 + c) = 0.0;
		}
	Vector b(6);
	for (int r = 0; r < 3; r++)
	{
		b[0 + r] = 0.0;
		b[3 + r] = dt * a[r];
	}

	if (x)
	{
		*x = A * xprev + b;
	}
	if (Jfxprev)
	{
		*Jfxprev = A;
	}
}

// observation function
// given x{k} return z{k} = h(x{k}) and its Jacobian J_h(x{k})
// J_h is |z|*|x| matrix 
void h(const Vector &x, Vector *z, Matrix *Jhx)
{
	double k1 = x[0] * x[0] + x[1] * x[1] + x[2] * x[2];
	double k2 = x[0] * x[0] + x[1] * x[1];

	if (z)
	{
		*z = Vector(3);
		(*z)[0] = atan2(x[1], x[0]);
		(*z)[1] = atan2(x[2], sqrt(k2));
		(*z)[2] = sqrt(k1);
	}
	if (Jhx)
	{
		*Jhx = Matrix(3, 6);
		Jhx->setConstant(0.0);

		// dz[0]/dx
		(*Jhx)(0, 0) = -x[1] / k2;
		(*Jhx)(0, 1) = +x[0] / k2;
		(*Jhx)(0, 2) = 0.0;

		// dz[1]/dx
		(*Jhx)(1, 0) = -x[0] * x[2] / (sqrt(k2) * k1);
		(*Jhx)(1, 1) = -x[1] * x[2] / (sqrt(k2) * k1);
		(*Jhx)(1, 2) = sqrt(k2) / k1;

		// dz[2]/dx
		(*Jhx)(2, 0) = x[0] / sqrt(k1);
		(*Jhx)(2, 1) = x[1] / sqrt(k1);
		(*Jhx)(2, 2) = x[2] / sqrt(k1);
	}
}

void main(void)
{
	const int iEKFIterations = 1;

	// dynamics noise covariance matrix
	// dynamics are deterministic in this homework, so Q = 0
	Matrix Q(6, 6);
	Q.setConstant(0.0);

	// observation noise covariance matrix
	// change the diagonal entries of this if you change noise field in xml model
	Matrix R(3, 3);
	R.setIdentity();
	R(0, 0) = 0.05;
	R(1, 1) = 0.05;
	R(2, 2) = 0.30;

	// connect to mujoco server
	mjInit();
	mjConnect(10000);

	// load hand model
	if (mjIsConnected())
	{
		mjLoad("estimator.xml");
		Sleep(1000);  // wait until the load is complete
	}

	if (mjIsConnected() && mjIsModel())
	{
		mjSetMode(2);

		// loops over a series of simulation trials (state is reset between trials)
		int numHit = 0;
		int numMiss = 0;
		for (int trial = 0; trial < NUM_TRIAL; trial++)
		{
			// reset to trial's keyframe
			mjReset(trial);

			// estimate of initial state
			// set xa to something non-informative, but non-zero to prevent singularities
			Vector xa(6);
			xa.setConstant(0.0); xa[1] = 1.0;

			// noise covarience in initial state estimate
			Matrix P(6, 6);
			// TODO: play with this parameter to see how it affects results
			double Pdiag = 1e-0;
			P.setIdentity();
			P = P * Pdiag;

			Matrix I(6, 6);
			I.setIdentity();

			// run simulation until ball hits something
			for (int k = 0;; k++)
			{
				// noisy sensor measurement
				Vector z(3, mjGetSensor());

				// EKF Predictor (Model Forecast)
				Vector x_fk;
				Matrix Jf_x_akprev;
				f(xa, &x_fk, &Jf_x_akprev);
				Matrix P_fk = (Jf_x_akprev * P * Jf_x_akprev.transpose()) + Q;

				// EKF Corrector (Data Assimilation)
				Vector x_aki = x_fk;
				Matrix K_ki;
				Matrix Jh_x_aki;
				for (int i = 0; i < iEKFIterations; i++)
				{
					Vector z_ki;
					h(x_aki, &z_ki, &Jh_x_aki);
					K_ki = P_fk * Jh_x_aki.transpose() * (Jh_x_aki * P_fk * Jh_x_aki.transpose() + R).inverse();
					x_aki = x_fk + K_ki * (z - z_ki);
				}
				Matrix P_k = (I - K_ki * Jh_x_aki) * P_fk;
				
				// Store our computed values for the next iteration
				xa = x_aki;
				P = P_k;

				// set estimator location for visualization
				mjSetEstimator(xa);

				// fill xtarget vector here with desired horizonal (entry 0) and
				// vectical (entry 1) position of the paddle
				Vector xtarget(2);
				xtarget[0] = xa[0];
				xtarget[1] = xa[2];
				mjSetControl(2, xtarget);

				// step simulation forward
				mjStep();
				Sleep(1);

				// check for paddle of wall contact to determine scoring
				// do not modify this
				mjContact contact = mjGetContacts();
				if (contact.nc > 0)
				{
					printf("trial %02d: %02d ", trial, int(contact.geom1[0]));
					if (contact.geom1[0] == PADDLE_GEOM)
					{
						printf("hit\n");
						numHit++;
					}
					else
					{
						printf("miss\n");
						numMiss++;
					}

					// run the simulation for one more second to show post-contact result
					for (int kpost = 0; kpost<int(1.0 / dt); kpost++)
					{
						mjStep();
						Sleep(1);
					}

					break;
				}
			}
		}
		// we will use this to tally your results. Do not remove.
		printf("hit: %02d miss: %02d\n", numHit, numMiss);

		mjDisconnect();
	}
	mjClear();
}
