// mujoco communication interface
#include "comm.h"

// algebra classes
#include "GMatrix.h"
#include "GVector.h"
#include "Quaternion.h"
#include "windows.h"
#include <iostream>

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

struct neighbor_t
{
	double cost;
	int other;

	neighbor_t()
		: cost(INFINITY), other(0)
	{}
	neighbor_t(double cost, int other)
		: cost(cost), other(other)
	{}
};

struct node_t
{
	Vector state;
	std::vector<neighbor_t> neighbors;

	double dijkstra_cost;
	int dijkstra_from;
	bool dijkstra_visited;
	bool dijkstra_seen;

	node_t(const Vector &state)
		: state(state)
	{}

	static void dijkstra_reset(node_t &node)
	{
		node.dijkstra_cost = INFINITY;
		node.dijkstra_from = 0;
		node.dijkstra_visited = false;
		node.dijkstra_seen = false;
	}
};

bool neighborSort(const neighbor_t &left, const neighbor_t &right)
{
	return (left.cost < right.cost);
}

class dijkstraSorter_t
{
public:
	dijkstraSorter_t(const std::vector<node_t> &nodes)
		: nodes(nodes)
	{}

	bool operator() (int left, int right)
	{
		return (nodes[left].dijkstra_cost < nodes[right].dijkstra_cost);
	}

private:
	const std::vector<node_t> &nodes;
};

void main(void)
{
	// three initial poses vectors (must be included in the list of samples)
	static const Vector qinit[3] = { Vector(NUM_JOINT, keyinit[0]), 
		Vector(NUM_JOINT, keyinit[1]), Vector(NUM_JOINT, keyinit[2]) };

	// goal pose vector (must be included in the list of samples)
	static const Vector qgoal = Vector(NUM_JOINT, keygoal);

	static const int N = 100;
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
		std::vector<node_t> nodes;
		for (int i = 0; i < N; i++)
		{
			Vector qpos = generateSample();
			if (isValidState(qpos, size))
			{
				// Got one!
				nodes.push_back(node_t(qpos));
			}
			else
			{
				// Try again!
				i--;
			}
		}
		nodes.push_back(node_t(qinit[0]));
		nodes.push_back(node_t(qinit[1]));
		nodes.push_back(node_t(qinit[2]));
		nodes.push_back(node_t(qgoal));

		// once valid, collision-free samples are generated, create a set of
		// nearest neighbors for each sample
		for (size_t i = 0; i < nodes.size(); i++)
		{
			node_t &node = nodes[i];
			const Vector &qa = node.state;

			// Calculate distances
			std::vector<neighbor_t> distances;
			for (size_t j = i+1; j < nodes.size(); j++)
			{
				const Vector &qb = nodes[j].state;
				if (!isValidPath(qa, qb, size))
				{
					continue;
				}

				const double distance = (qb - qa).length();
				distances.push_back(neighbor_t(distance, j));
			}

			// Sort distances, and trim the list
			std::sort(distances.begin(), distances.end(), neighborSort);
			for (size_t j = 0; (j < distances.size() && j < K); j++)
			{
				node.neighbors.push_back(distances[j]);

				// Handle backwards link too
				nodes[distances[j].other].neighbors.push_back(neighbor_t(distances[j].cost, i));
			}

			std::cout << "Processed neighbors for node #" << i << std::endl;
		}

		// Sort and shrink our list of neighbors as needed (the addition of
		// backwards links may have grown over the limit and unsorted)
		for (size_t i = 0; i < nodes.size(); i++)
		{
			std::vector<neighbor_t> &neighbors = nodes[i].neighbors;
			std::sort(neighbors.begin(), neighbors.end(), neighborSort);
			if (neighbors.size() > K)
			{
				neighbors.resize(K);
			}
		}

		// find three shortest paths among the available samples from 
		// each of three qinit vectors to qgoal vector (using nearest
		// neighbors above)
		std::vector<std::vector<int>> solutionPaths(ARRAYSIZE(qinit));
		for (size_t i = 0; i < solutionPaths.size(); i++)
		{
			// Reset dijkstra values to prepare for a new invocation
			std::for_each(nodes.begin(), nodes.end(), &node_t::dijkstra_reset);

			std::vector<int> toVisit;
			toVisit.push_back(N+i); // prime our search space with qinit[i]
			nodes[N+i].dijkstra_cost = 0;
			nodes[N+i].dijkstra_from = N+i;

			dijkstraSorter_t toVisitSort(nodes);
			while (!toVisit.empty() && !nodes[nodes.size()-1].dijkstra_visited)
			{
				// Find the next node to visit with the shortest path
				std::sort(toVisit.begin(), toVisit.end(), toVisitSort);
				int nodeIdx = toVisit.front();
				toVisit.erase(toVisit.begin());
				node_t &node = nodes[nodeIdx];

				for (size_t j = 0; j < node.neighbors.size(); j++)
				{
					neighbor_t &neighbor = node.neighbors[j];
					node_t &neighborNode = nodes[neighbor.other];

					// No need to evaluate nodes we've already visited
					if (neighborNode.dijkstra_visited)
					{
						continue;
					}
					
					if (!neighborNode.dijkstra_seen)
					{
						toVisit.push_back(neighbor.other);
						neighborNode.dijkstra_seen = true;
					}

					double newCost = (node.dijkstra_cost + neighbor.cost);
					if (newCost < neighborNode.dijkstra_cost)
					{
						neighborNode.dijkstra_cost = newCost;
						neighborNode.dijkstra_from = nodeIdx;
					}
				}

				node.dijkstra_visited = true;
			}

			if (nodes[nodes.size()-1].dijkstra_cost != INFINITY)
			{
				// Solution found!
				std::vector<int> &solutionPath = solutionPaths[i];
				int n = (nodes.size() - 1);
				solutionPath.push_back(n);
				do
				{
					if (nodes[n].dijkstra_from != n)
					{
						n = nodes[n].dijkstra_from;
					}
					solutionPath.insert(solutionPath.begin(), n);
				} while (nodes[n].dijkstra_from != n);

				std::cout << "Solution for qinit[" << i << "]: ";
				for (size_t j = 0; j < solutionPath.size(); j++)
				{
					if (j != 0)
					{
						std::cout << " --> ";
					}
					std::cout << solutionPath[j];
				}
				std::cout << std::endl;
			}
			else
			{
				// No solution found...
				std::cout << "No solution found for qinit[" << i << "]..." << std::endl;
			}
		}

		// animate three shortest paths found above by interpolating their
		// positions and setting this interpolated state by mjSetState()
		int i = 0;
		while (true)
		{
			std::vector<int> &solutionPath = solutionPaths[i];
			
			for (size_t j = 0; j < solutionPath.size(); j++)
			{
				if (j == 0)
				{
					std::cout << "Animating path #" << (i+1) << "..." << std::endl;
				}

				if (j != (solutionPath.size() - 1))
				{
					const int animSteps = 10000;

					const Vector &qa = nodes[solutionPath[j]].state;
					const Vector &qb = nodes[solutionPath[j+1]].state;
					const Vector deltaq = (qb-qa) / animSteps;
					Vector q(qa);

					for (int k = 0; k < animSteps; k++)
					{
						Vector qvel(size.nv);
						Vector act(size.na);
						qvel.setConstant(0.0);
						act.setConstant(0.0);
						mjSetState(size.nq, size.nv, size.na, 0.0, q, qvel, act);

						if (j == 0 && k == 0)
						{
							// If this is the first animation step for the initial
							// segment, pause for a moment in the initial position
							Sleep(500);
						}

						q = (q + deltaq);
					}
				}
				else
				{
					Vector qvel(size.nv);
					Vector act(size.na);
					qvel.setConstant(0.0);
					act.setConstant(0.0);
					mjSetState(size.nq, size.nv, size.na, 0.0, nodes[solutionPath[j]].state, qvel, act);

					// Nothing to interpolate for the last one, so just pause for
					// a moment and enjoy basking in success!
					Sleep(1500);
				}
			}

			i = ((i + 1) % solutionPaths.size());
		}

		// done!

		std::cout << std::endl << "Done..." << std::endl;
		int x;
		std::cin >> x;

		mjDisconnect();
	}
	mjClear();
}
