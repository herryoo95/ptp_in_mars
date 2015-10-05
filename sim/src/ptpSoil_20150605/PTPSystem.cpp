#include "PTPSystem.h"


namespace mars {
  namespace sim {

//PUBLIC
PTPSystem::PTPSystem(){
}
PTPSystem::~PTPSystem()
{
}

void PTPSystem::endPTP(){
	//	free(CELLP);
	//	free(PARTICLEP);
}
char* PTPSystem::printPTP(){
	static char buf[500];
	snprintf(buf,sizeof(buf),"PTP: s(%.2f,%.2f,%.2f)mm e(%.2f,%.2f,%.2f)mm n(%d,%d,%d)\n", ptp->start.x/MAG,ptp->start.y/MAG,ptp->start.z/MAG,ptp->end.x/MAG,ptp->end.y/MAG,ptp->end.z/MAG,ptp->n.x,ptp->n.y,ptp->n.z);

	return buf; 
}
char* PTPSystem::printCELL(const vecN n){
	static char buf[500];
	snprintf(buf,sizeof(buf),"cell[%d][%d][%d] s(%.3f,%.3f,%.3f)mm e(%.3f,%.3f,%.3f)mm\n",n.x,n.y,n.z,cell[n.x][n.y][n.z].start.x/MAG,cell[n.x][n.y][n.z].start.y/MAG,cell[n.x][n.y][n.z].start.z/MAG,cell[n.x][n.y][n.z].end.x/MAG,cell[n.x][n.y][n.z].end.y/MAG,cell[n.x][n.y][n.z].end.z/MAG);

	return buf; 
}
char* PTPSystem::printPARTICLE(const vecN n){
	static char buf[500];
	snprintf(buf,sizeof(buf),"particle[%d][%d][%d]:(%.3f,%.3f,%.3f)mm\n",n.x,n.y,n.z,particle[n.x][n.y][n.z].loc.x/MAG,particle[n.x][n.y][n.z].loc.y/MAG,particle[n.x][n.y][n.z].loc.z/MAG);
	return buf; 
}

bool PTPSystem::getPTPForce(FOOT (&foot)[6], vecD (&force)[6], vecD (&forceT)[6]){
//	static FOOT foot_last;
	static FOOT foot_last[6];
	static int last_flag=0;
	double big_rSQR;
	static vecD fT[6];
	static vecD f[6];
	

	/////ADAPT MAG
	//foot.l = foot.l*MAG;
	//foot.loc.x = foot.loc.x*MAG;
	//foot.loc.y = foot.loc.y*MAG;
	//foot.loc.z = foot.loc.z*MAG;

	//foot.r = foot.r*MAG;
	//foot.vel.x = foot.vel.x*MAG;
	//foot.vel.y = foot.vel.y*MAG;
	//foot.vel.z = foot.vel.z*MAG;

	///ADAPT MAG
	for(int i=0;i<6;i++){
	foot[i].l = foot[i].l*MAG;
	foot[i].loc.x = foot[i].loc.x*MAG;
	foot[i].loc.y = foot[i].loc.y*MAG;
	foot[i].loc.z = foot[i].loc.z*MAG;

	foot[i].r = foot[i].r*MAG;
	foot[i].vel.x = foot[i].vel.x*MAG;
	foot[i].vel.y = foot[i].vel.y*MAG;
	foot[i].vel.z = foot[i].vel.z*MAG;
	}

	//if(last_flag && (!FLOAT_EQ(foot.r,foot_last.r) || !FLOAT_EQ(foot.type,foot_last.type) || !FLOAT_EQ(foot.l,foot_last.l))){
		//printf("INPUT ERROR\n");		
		//foot_last = foot;
		//return false;


	//}else{
		ptpWorks(foot,foot_last);
	for(int i=0;i<6;i++){
		calcForce(&foot[i], FAST,&f[i],&fT[i]);	//have a look of FINE

		foot_last[i] = foot[i];
		last_flag=1;
		forceT[i].x = fT[i].x;
		forceT[i].y = fT[i].y;
		forceT[i].z = fT[i].z;
		force[i].x = f[i].x;
		force[i].y = f[i].y;
		force[i].z = f[i].z;
		
//		  	  printf(" for(%f %f %f)\n",force[i].x,force[i].y,force[i].z);
		
	}		
		return true;
//	}

}

  } // end of namespace sim
} // end of namespace mars
