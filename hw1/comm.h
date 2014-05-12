/*******************************************
This file is part of the MuJoCo software.
(C) 2014 Emo Todorov. All rights reserved.
*******************************************/

#ifndef _MJ_COM_H_
#define _MJ_COM_H_

#define MJDLL __declspec(dllimport)


//------------------------- Type definitions --------------------------------------------

typedef enum _mjtComm					// communication messages
{
	// error codes (server to client)
	mjCOMM_OK			= 0,			// success
	mjCOMM_BADSIZE		= -1,			// data has invalid size
	mjCOMM_BADINDEX		= -2,			// object has invalid index
	mjCOMM_BADMODE		= -3,			// command incompatible with mode
	mjCOMM_BADCOMMAND	= -4,			// unknown command
	mjCOMM_NOMODEL		= -5,			// model has not been loaded
	mjCOMM_CANNOTSEND	= -6,			// could not send data
	mjCOMM_CANNOTRECV	= -7,			// could not receive data
	mjCOMM_TIMEOUT		= -8,			// receive timeout
} mjtComm;


typedef struct _mjSize					// size data
{
	int nq;								// number of generalized positions
	int nv;								// number of generalized velocities
	int na;								// number of actuator activations
	int nu;								// number of controls (= actuators)
	int nbody;							// number of bodies (including world)
	int njnt;							// number of joints
	int ngeom;							// number of geoms
	int nsite;							// number of sites
	int ntendon;						// number of tendons
	int neq;							// number of equality constraints
} mjSize;


typedef struct _mjState					// state data
{
	double* qpos;						// generalized positions			(nq x 1)
	double* qvel;						// generalized velocities			(nv x 1)
	double* act;						// actuator activations				(na x 1)
	double time;						// simulation time
	int nq;								// number of generalized positions
	int nv;								// number of generalized velocities
	int na;								// number of actuator activations
} mjState;


typedef struct _mjCartesian				// Cartesian data for bodies, geoms, sites
{
	double* pos;						// positions						(nobj x 3)
	double* quat;						// orientations						(nobj x 4)
	double* linvel;						// linear velocities				(nobj x 3)
	double* angvel;						// angular velocities				(nobj x 3)
	int nobj;							// number of objects
} mjCartesian;


typedef struct _mjJoint					// joint data
{
	double* anchor;						// joint anchor positions			(njnt x 3)
	double* axis;						// joint axes						(njnt x 3)
	int njnt;							// number of joints
} mjJoint;


typedef struct _mjLinear				// data for tendons and actuators
{
	double* length;						// lengths							(nobj x 1)
	double* velocity;					// veocities						(nobj x 1)
	int nobj;							// number of objects
} mjLinear;


typedef struct _mjContact				// contact data
{
	double* pos;						// contact positions				(nc x 3)
	double* norm;						// contact normals					(nc x 3)
	double* dist;						// normal distances					(nc x 1)
	int* geom1;							// 1st contacting geom indices		(nc x 1)
	int* geom2;							// 2nd contacting geom indices		(nc x 1)
	int nc;								// number of contacts
} mjContact;


typedef struct _mjJacobian				// Jacobian data for one frame
{
	double* jacpos;						// translation Jacobian				(3 x nv)
	double* jacrot;						// rotation Jacobian				(3 x nv)
	int nv;								// number of generalized velocities
} mjJacobian;



//------------------------- API ---------------------------------------------------------

// initalization of socket library
MJDLL void mjInit(void);
MJDLL void mjClear(void);

// connection with MuJoCo
MJDLL bool mjConnect(int timeout, const char* host = 0);
MJDLL bool mjIsConnected(void);
MJDLL void mjDisconnect(void);

// commands to MuJoCo
MJDLL void mjStep(void);
MJDLL void mjStep1(void);
MJDLL void mjStep2(void);
MJDLL void mjReset(int key = -1);
MJDLL void mjLoad(const char* filename);
MJDLL void mjQuit(void);

// getting data
MJDLL bool mjIsModel(void);
MJDLL int mjGetMode(void);
MJDLL mjSize mjGetSize(void);
MJDLL mjState mjGetState(void);
MJDLL mjCartesian mjGetBodies(void);
MJDLL mjCartesian mjGetGeoms(void);
MJDLL mjCartesian mjGetSites(void);
MJDLL mjJoint mjGetJoints(void);
MJDLL mjLinear mjGetTendons(void);
MJDLL mjLinear mjGetActuators(void);
MJDLL mjContact mjGetContacts(void);

// getting Jacobians
MJDLL mjJacobian mjJacBody(int index);
MJDLL mjJacobian mjJacGeom(int index);
MJDLL mjJacobian mjJacSite(int index);

// setting data
MJDLL void mjSetMode(int mode);
MJDLL void mjSetState(int nq, int nv, int na, 
				double* qpos, double* qvel, double* act);
MJDLL void mjSetControl(int nu, double* ctrl);

// error handling
MJDLL int mjError(void);
MJDLL void mjSetErrorHandler(void(*)(int));


#endif