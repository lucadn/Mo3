
//
#ifndef MOMO_H
#define MOMO_H

#include <omnetpp.h>
#include "costants.h"
#include "math.h"
#include "physic.h"
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace std;

using namespace omnetpp;
#define SELECT 50
#define UPDATE 55
#define SWITCH_BEHAVIOR 60
#define SWITCH_MODALITY 70
#define displayS "p=%d,%d,e;b=12,12,oval;o=%s,,1"
#define sgn(x) ((x)==0 ? 0 : ((x)<0 ? -1 : 1))
extern position database[NMAX];
extern double speedVector[NMAX];
extern double directionVector[NMAX];
extern int groupIDVector[NMAX];
extern bool selectNewMobilityParameters[NMAX];
extern int rotationViolation[NMAX];
extern int speedViolation[NMAX];
class MoMo : public cSimpleModule
{

	//Macro that contains the costructor,destructor
	//and other Omnett++ stuff


	virtual void initialize();
	virtual void handleMessage(cMessage* );
	virtual void finish();
	//

   private:
	//implement the mvement that sims a torus
	bool torus(double&, double&);
	char str[50];
	//implement the rebound movement
	bool rebound(double&, double&);

	//returns the time intervall
	//to the next move
	void loadGroupMembers();
	double momoMobility(double&, double&);
	void checkCollisions(double&, double&);
	int checkGroupingFactor(double x , double y );
	void recordMobilityEffect(double x, double y, double alpha, double time);
	double distance(int, int);
	//quoantum of time between to moves
	//e.g. 1s if the speed is in m/sec
	cPar* moveInterval;
	cPar* pauseTime;
	cPar* moveKind;
	cPar* minSpeed;
	cPar* maxSpeed;
	//cPar* distance;
	cPar* mobilitySelectPeriod;
	simsignal_t collisionConflictSignal;
	simsignal_t collisionAvoidedSignal;
	double amax;
	double gmax;
	bool mobilityFlag;
	double mobilePerc;
	bool group;
	bool groupBoundUpdate;
	bool forcedStatus;
	bool checkUpdateFlag;
	bool graphicsFlag;
	bool localGroupVector[NMAX];
	double localMoMoProximityBound[NMAX][3];
	int groupModalities;
	int numGroups;
	int hostNum;
	int groupSize;
	string groupsFileName;
	int groupModality;
	double rho;
	double momoProximityBound;
	double groupBoundUpdatePeriod;
	double checkCollisionDistance;
	double minDistanceCA;
	double minReactionTime;
	bool correctedDirection;
	bool collisionAvoidance;
	bool caDebugFlag;
	//pointer of the physic module that
	//list of the neighbours
	//store the actual <x,y> position
	Physic* physic;
	cModule *mh;
	//size of the movement field
	double minX,maxX,minY,maxY;

	//direction flag
	double dX, dY;

	//direction angle
	double alfa;

	double speed;
	//number of steps to reach
	//the destination
	int steps;
	int myID; //mobilenode ID
	//statistics vars
	int stepsNum;
	double partial;
};

#endif
