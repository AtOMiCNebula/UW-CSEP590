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

// useful for neighbor storage and sorting
#include <vector>
#include <algorithm>

// you can use this constant to denote infinitely large values
static const double INFINITY = DBL_MAX;

// hardcoded number of model joints
static const int NUM_JOINT = 7;
// joint limits for the model
static const double jointmin[NUM_JOINT] = {-40, -50, -50, -60, -90, -30, -60};
static const double jointmax[NUM_JOINT] = {+40, +50, +50, +60, +90, +30, +60};
// initial poses
static const double keyinit[3][NUM_JOINT] = 
{
	{0.35, -0.22, 0.83, -0.42, 0.84, 0, 0.35},
	{0.7, 0.87, 0.87, 1, -0.38, 0.17, 0},
	{-0.64, -0.21, -0.47, -0.1, -0.12, 0, 0}
};
// goal pose
static const double keygoal[NUM_JOINT] = {0, 0, 0, 0, 0, 0, 0};


// generate a position vector for a random arm configuration (within joint limits)
// returns a qpos vector suitable for use in mjSetState()
Vector generateSample()
{
	static const double DEG_TO_RAD = 0.017453292519943295769236907684886;

	Vector qpos(NUM_JOINT);
	for(int i=0; i<NUM_JOINT; i++)
	{
		qpos[i] = double(rand()) / double(RAND_MAX);
		qpos[i] = (jointmax[i] - jointmin[i]) * qpos[i] + jointmin[i];
		qpos[i] *= DEG_TO_RAD;
	}
	return qpos;
}

// test for a collision-free body configuration vector
// return true if body configuration q is collision-free
// return false otherwise
// do not change this function
bool isValidState(const Vector &q, const mjSize &size)
{
	Vector qvel(size.nv);
	Vector act(size.na);
	qvel.setConstant(0.0);
	act.setConstant(0.0);

	mjSetState(size.nq, size.nv, size.na, 0.0, (Vector&)q, qvel, act);
	mjContact contact = mjGetContacts();
	return (contact.nc == 0);
}

// test for collision-free path between two body configuration vectors
// return true if there exists a collision-free path between body configuration qa and qb
// return false otherwise
bool isValidPath(const Vector &qa, const Vector &qb, const mjSize &size)
{
	// number of interpolated steps between qa and qb to test for validity
	static const int numstep = 10;

	Vector q(qa);
	const Vector deltaq = (qb - qa) / (numstep+1);
	for (int i = 0; i < numstep; i++)
	{
		q = q + deltaq;
		if (!isValidState(q, size))
		{
			return false;
		}
	}
	return true;
}

bool neighborSort(std::pair<double, int> &left, std::pair<double, int> &right)
{
	return (left.first < right.first);
}

void main(void)
{
	// three initial poses vectors (must be included in the list of samples)
	static const Vector qinit[3] = { Vector(NUM_JOINT, keyinit[0]), 
		Vector(NUM_JOINT, keyinit[1]), Vector(NUM_JOINT, keyinit[2]) };

	// goal pose vector (must be included in the list of samples)
	static const Vector qgoal = Vector(NUM_JOINT, keygoal);

	static const int N = 1001;
	static const int K = 10;

	// connect to mujoco server
	mjInit();
	mjConnect(10000);

	// load hand model
	if(mjIsConnected())
	{
		mjLoad("hand-HW2.xml");
		Sleep(1000);  // wait till the load is complete
	}

	if(mjIsConnected() && mjIsModel())
	{
		mjSetMode(1);
		mjReset();

		// size containts model dimensions
		mjSize size = mjGetSize();

		// your implemention:

		// generate a collection of valid samples here
		// make sure qinit and qgoal are included in this collection
		const int numSamples = 500;
		Matrix validStates(numSamples+4, NUM_JOINT);
		for (int i = 0; i < numSamples; i++)
		{
			Vector qpos = generateSample();
			if (isValidState(qpos, size))
			{
				// Got one!
				validStates.setRow(i, qpos);
			}
			else
			{
				// Try again!
				i--;
			}
		}
		validStates.setRow(numSamples+0, qinit[0]);
		validStates.setRow(numSamples+1, qinit[1]);
		validStates.setRow(numSamples+2, qinit[2]);
		validStates.setRow(numSamples+3, qgoal);

		// once valid, collision-free samples are generated, create a set of
		// nearest neighbors for each sample
		GMatrix<int> neighbors(validStates.getNumRows(), K);
		for (int i = 0; i < neighbors.getNumRows(); i++)
		{
			const Vector qa = validStates.getRow(i);

			// Calculate distances
			std::vector<std::pair<double, int>> distances;
			for (int j = i+1; j < neighbors.getNumRows(); j++)
			{
				const Vector qb = validStates.getRow(j);
				if (!isValidPath(qa, qb, size))
				{
					continue;
				}

				const double distance = (qb - qa).length();
				distances.push_back(std::pair<double, int>(distance, j));
			}

			// Sort distances, and trim the list
			std::sort(distances.begin(), distances.end(), neighborSort);
			for (int j = 0; j < neighbors.getNumCols(); j++)
			{
				neighbors[i][j] = ((static_cast<size_t>(j) < distances.size()) ? distances[j].second : 0);
			}
		}

		// find three shortest paths among the available samples from 
		// each of three qinit vectors to qgoal vector (using nearest
		// neighbors above)

		// animate three shortest paths found above by interpolating their
		// positions and setting this interpolated state by mjSetState()

		// done!

		mjDisconnect();
	}
	mjClear();
}
