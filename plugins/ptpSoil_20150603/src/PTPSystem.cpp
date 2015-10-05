#include "PTPSystem.hpp"


namespace mars {
  namespace plugins {
    namespace ptpSoil {

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

//bool PTPSystem::getPTPForce(FOOT foot, vecD* force, vecD* forceT){
	//static FOOT foot_last;
	//static int last_flag=0;
	//double big_rSQR;
	//static vecD fT;
	//static vecD f;

	/////ADAPT MAG
	//foot.l = foot.l*MAG;
	//foot.loc.x = foot.loc.x*MAG;
	//foot.loc.y = foot.loc.y*MAG;
	//foot.loc.z = foot.loc.z*MAG;

	//foot.r = foot.r*MAG;
	//foot.vel.x = foot.vel.x*MAG;
	//foot.vel.y = foot.vel.y*MAG;
	//foot.vel.z = foot.vel.z*MAG;


	//if(last_flag && (!FLOAT_EQ(foot.r,foot_last.r) || !FLOAT_EQ(foot.type,foot_last.type) || !FLOAT_EQ(foot.l,foot_last.l))){
		//printf("INPUT ERROR\n");		
		//foot_last = foot;
		//return false;
	//}else if(last_flag && foot.loc.x==foot_last.loc.x && 
		//foot.loc.y==foot_last.loc.y &&
		//foot.loc.z==foot_last.loc.z &&
		//foot.vel.x==foot_last.vel.x &&
		//foot.vel.y==foot_last.vel.y &&
		//foot.vel.z==foot_last.vel.z){

			//printf("SAME FOOT INPUT\n");
			//*forceT=fT;
			//*force=f;
			//return true; 

	//}else{
		//ptpWorks(&foot,&foot_last);
		//if(foot.type==SPHERE){
			//big_rSQR = foot.r*foot.r;
		//}

		//if(foot.type==SPHERE){

			//if(last_flag){
				//if(ptputil->getLengthSQR(&foot.loc,&foot_last.loc,ALL)>big_rSQR){
					//calcForce(&foot, FINE,&f,&fT);	
				//}else{			
//#ifdef FAST_CALC
					//calcForce(&foot, FAST,&f,&fT);	
//#endif
//#ifndef FAST_CALC
					//calcForce(&foot, FINE,&f,&fT);	
//#endif
				//}
			//}else{	
				//calcForce(&foot, FINE,&f,&fT);	
			//}	


		//}

		//foot_last = foot;
		//last_flag=1;
		//forceT->x = fT.x;
		//forceT->y = fT.y;
		//forceT->z = fT.z;
		//force->x = f.x;
		//force->y = f.y;
		//force->z = f.z;
		//return true;
	//}

//}

bool PTPSystem::getPTPForce(FOOT foot, vecD* force, vecD* forceT){
//	static FOOT foot_last;
	static FOOT foot_last;
	static int last_flag=0;
	double big_rSQR;
	static vecD fT;
	static vecD f;
	

	///ADAPT MAG
	foot.l = foot.l*MAG;
	foot.loc.x = foot.loc.x*MAG;
	foot.loc.y = foot.loc.y*MAG;
	foot.loc.z = foot.loc.z*MAG;

	foot.r = foot.r*MAG;
	foot.vel.x = foot.vel.x*MAG;
	foot.vel.y = foot.vel.y*MAG;
	foot.vel.z = foot.vel.z*MAG;


	//if(last_flag && (!FLOAT_EQ(foot.r,foot_last.r) || !FLOAT_EQ(foot.type,foot_last.type) || !FLOAT_EQ(foot.l,foot_last.l))){
		//printf("INPUT ERROR\n");		
		//foot_last = foot;
		//return false;


	//}else{
		ptpWorks(&foot,&foot_last);
	//for(int i=0;i<6;i++){
		calcForce(&foot, FAST,&f,&fT);	//have a look of FINE

		//foot_last[i] = foot[i];
		//last_flag=1;
		//forceT[i].x = fT[i].x;
		//forceT[i].y = fT[i].y;
		//forceT[i].z = fT[i].z;
		//force[i].x = f[i].x;
		//force[i].y = f[i].y;
		//force[i].z = f[i].z;
		
		foot_last = foot;
		last_flag=1;
		forceT->x = fT.x;
		forceT->y = fT.y;
		forceT->z = fT.z;
		force->x = f.x;
		force->y = f.y;
		force->z = f.z;
		
//		  	  printf(" for(%f %f %f)\n",force[i].x,force[i].y,force[i].z);
		
	//}		
		return true;
//	}

}
 
    } // end of namespace ptpSoil
  } // end of namespace plugins
} // end of namespace mars
