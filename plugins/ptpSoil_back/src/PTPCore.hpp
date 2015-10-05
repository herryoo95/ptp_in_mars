#ifndef __PTPCORE_H__
#define __PTPCORE_H__
#include <stdio.h>
#include "PTPUtil.hpp"
#include <math.h>
#include <stdlib.h>
#include <Eigen/Core>


#define MM2M 0.001
#define M2MM 1000
#define SPHERE 0
#define CYLINDER 1
#define CUBE 2
#define NOT 0
#define GOLINE 1024
#define FEATURE_POINT 512
#define ERR -1
#define FINE 1
#define FAST 2
#define START 1
#define END 2
#define PTP_START 4
#define PTP_END 8

#define MOST_UPPER 0
#define MOST_CONTAIN 1

///SPHARE
#define IS_ON 1
#define IS_IN 256

///CYLINDER
#define IS_ON_UPPER 2
#define IS_ON_UNDER 4
#define IS_ON_SIDE 8

///BOX
#define IS_ON_TOP 16
#define IS_ON_BOTTOM 32
#define IS_ON_LEFT 64
#define IS_ON_RIGHT 128

///COLLISION
#define NOT_COLLISION	0
#define COLLISION		-1


namespace mars {
  namespace plugins {
	namespace ptpSoil {
	
typedef Eigen::Matrix<double, 3, 1, Eigen::DontAlign> Vector;

typedef struct FOOT{
	vecD loc;					///[mm*MAG]
	//	vecD acc;					///[mm*MAG/s^2]
	vecD vel;					///[mm*MAG/s]
	//	vecD avel;					///[rad/s]
			
	int type;					///SPHERE,CYLINDER
	double r;					///[mm*MAG]
	double l;					///[mm*MAG]
	double d;					///[mm*MAG]
	double mass;
	//	vecD moi;					
} FOOT;

typedef struct PTP{
	vecD start;					///[mm*MAG]
	vecD end;					///[mm*MAG]
	vecN n;						///[]
	vecD cellScale;
	vecD test;
	vecD start_loc;				/// to calculate dip. of lateral	
	int lateral;
	
} PTP;

typedef struct CELL {
	vecD start;					///[mm*MAG]
	vecD end;					///[mm*MAG]
	vecD center;
	vecD averageLoc;			/** average location of particles collided with object */

#ifdef USE_FORCE_DEBUG
	vecD fZ;
	vecD fL;

#endif
//	int CollisionType;
//	int CellVertex;
	unsigned int CollisionParticleNum;

#ifdef PRICISE_HIGH
	int CellLine;
	int CellPlane;
#endif

unsigned int particleNum;
} CELL;

typedef struct PARTICLE {
	vecD loc;					///[mm*MAG]
	vecN cloc;					///[]
	int isCollision;
} PARTICLE;
#ifdef USE_SOIL_VISUALIZATION
typedef struct SOIL {
vecD loc;					///[mm*MAG]
} SOIL;
#endif


#ifdef USE_COLLISION_DEBUG
typedef struct DEBUG_DATA{
	vecN boundCellStart;
	vecN boundCellEnd;
	vecD boundPointStart;
	vecD boundPointEnd;

}DEBUG_DATA;
#endif

class PTPCore{

public:
		
	PTPCore();
	~PTPCore();
	
	#ifdef USE_SOIL_VISUALIZATION
	vecD soilMovement(const vecD* point,const FOOT* foot);
	int soilWorks(const vecN* soilN,const FOOT* foot,const FOOT* lastFootLoc, const int pLocData);
//	vecD getOriginalSoilPosition(const vecN n);
	SOIL SOIL_GLOBAL[MAX_X+1][MAX_Y+1][MAX_Z+1];
	int soilCollisionDetect(const FOOT* foot, const SOIL* soil);
	#endif

	bool AABBCollision(CELL* pAABB1, CELL* pAABB2);

	bool calcForce(const FOOT* foot, const int type, vecD* forceR, vecD* forceT);
//	int initializePTP(const int p);
	int initializePTP(const vecN *n,const vecD *s,const vecD *e);	
	int particleWorks(const vecN* particleN,const FOOT* foot, const FOOT* lastFootLoc, const int pLocData);


	int ptpWorks(const FOOT* foot,const FOOT* lastFoot);
	vecD particleMovement(const vecD* point,const FOOT* foot);
	
	vecN getComputeNeededCellNum(const FOOT* foot, const int type);

	int getSumOfParticlesNum();
	int particleCollisionDetect(const FOOT* foot, const PARTICLE* particle);
	bool PTPCollisionDetectOne(const FOOT* foot);
	int cellCollisionDetectVertex(const FOOT* foot, const vecN* cellN);
	int cellCollisionDetectLine(const FOOT* foot, const vecN* cellN);
	double lookupTable(const CELL* cell);
	double shearStress(const double disp, const double pressure);  
	double getArea(const CELL* cell,const int direction);
	vecD getCellSize();

	double determinePlanePointLoc(const vecD n, const double d, const vecD p);

	vecD getFeaturePoint(const FOOT* foot, const vecD cellcenter);

	int isInABox(const vecD* p,const vecD* start,const vecD* end);
	int isInACube(const FOOT* foot, const vecD* point);
	int isInASphere(const FOOT* foot, const vecD* point);
	int isInACylinder(const FOOT* foot, const vecD* point);
//	int getCellProj(const FOOT* foot, const int type, const int mode);

	vecD getOriginalParticlePosition(const vecN n);
	int getIntersectionNumLineSphere(const vecD* LineStart, const vecD* LineEnd, const vecD* SphereCenter, const double* SphereRadius);
	int cellCollisionFeaturePoint(const FOOT* foot, const vecN* cellN);
	//int CellWorks(const FOOT* foot, const vecN* cellN);

	unsigned int PROJ_Y_M[MAX_X][MAX_Z];
	unsigned int PROJ_Y_P[MAX_X][MAX_Z];
	unsigned int PROJ_X_M[MAX_Y][MAX_Z];
	unsigned int PROJ_X_P[MAX_Y][MAX_Z];
	unsigned int PROJ_Z_M[MAX_X][MAX_Y];
	unsigned int PROJ_Z_P[MAX_X][MAX_Y];

	#ifdef USE_COLLISION_DEBUG
	DEBUG_DATA DEBUG_GLOBAL;
	#endif

	vecD getFootAABBBound(const FOOT* foot, const int type);

	vecN judgeWhichCell(const vecD* loc);

	//vecD getCellSize(const PTP* ptp);
	//double getArea(const CELL* cell,const int direction);

	double getVolume(const CELL* cell);

	double getLength(const CELL* cell, const int type);

	#ifdef USE_SOIL_VISUALIZATION
	vecD getOriginalSoilPosition(const vecN n);
	#endif

	double radianShifter(const double* inp, const double* standard);
	double radianRestore(const double* inp, const double* standard);

	double radianShrink(const double* inp);
	

	PTP* ptp; //PTP_GLOBAL;

	PTPUtil* ptputil; //PTP_UTIL;
	PARTICLE particle[MAX_X][MAX_Y][MAX_Z];; //PARTICLE_GLOBAL[MAX_X][MAX_Y][MAX_Z];
	CELL cell[MAX_X][MAX_Y][MAX_Z];  //CELL_GLOBAL[MAX_X][MAX_Y][MAX_Z];
	char filenameCALC[150];
	
	vecN LOOPSTART;
	vecN LOOPEND;
	
	FILE *outCALC;


	bool isPTPInitialized;
private:


};	
	
    } // end of namespace ptpSoil
  } // end of namespace plugins
} // end of namespace mars

#endif // __PTPCORE_H__
