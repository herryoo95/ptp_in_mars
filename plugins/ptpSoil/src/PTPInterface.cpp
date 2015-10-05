/*
 *  Copyright 2015, DFKI GmbH Robotics Innovation Center
 *
 *  This file is part of the PTP(Plastic Terramechanics Particle).
 *
 *  Author: Dr. -Ing. Yong-Ho Yoo 
 *
 */

#include "PTPInterface.hpp"


namespace mars {
  namespace plugins {
    namespace ptpSoil {

PTPInterface::PTPInterface(int x, int y, int z) : PTPCore(x,y,z){
testlist = 123;
}
PTPInterface::~PTPInterface()
{

}

bool PTPInterface::makePTP(const VectorN *num,const Vector *start,const Vector *end){
	vecN n;
	vecD s,e;
	n.x = num->x();
	n.y = num->y();
	n.z = num->z();
	s.x = start->x();
	s.y = start->y();
	s.z = start->z();
	e.x = end->x();
	e.y = end->y();
	e.z = end->z();	
				
	return 	initializePTP(&n,&s,&e);	
}	

bool PTPInterface::getPTPForce(vFOOT vfoot, Vector* contactForce){
	
	static FOOT foot, foot_last;
	//static int last_flag=0;
	static vecD f;
	
	//foot <- vfoot
	foot.loc.x = vfoot.loc.x();
	foot.loc.y = vfoot.loc.y();	
	foot.loc.z = vfoot.loc.z();
	foot.vel.x = vfoot.vel.x();
	foot.vel.y = vfoot.vel.y();	
	foot.vel.z = vfoot.vel.z();
		
	foot.type = vfoot.type;
	foot.r 	  = vfoot.r;	
	foot.contact = vfoot.contact;	
	
	ptpWorks(&foot,&foot_last);
	calcForce(&foot, FAST,&f);	//have a look of FINE
		
	foot_last = foot;
		//last_flag=1;
	contactForce->x() = f.x;
	contactForce->y() = f.y;
	contactForce->z() = f.z;
		
	return true;

}
 
    } // end of namespace ptpSoil
  } // end of namespace plugins
} // end of namespace mars
