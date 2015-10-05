/*
 *  Copyright 2013, DFKI GmbH Robotics Innovation Center
 *
 *  This file is part of the MARS simulation framework.
 *
 *  MARS is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License
 *  as published by the Free Software Foundation, either version 3
 *  of the License, or (at your option) any later version.
 *
 *  MARS is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with MARS.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * \file PtpSoil.cpp
 * \author Yong-Ho (yong-ho.yoo@dfki.de)
 * \brief soil
 *
 * Version 0.1
 */


#include "PtpSoil.h"
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/data_broker/DataBrokerInterface.h>
#include <mars/data_broker/DataPackage.h>
#include <mars/utils/Vector.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <time.h>


namespace mars {
  namespace sim {

      using namespace mars::utils;
      using namespace mars::interfaces;
      
void PtpSoil::soil_init(){
	vecN num;
	vecD start;
	vecD end;
	
	//Number of Cells
	num.x = 200; 
	num.y = 140; 
	num.z = 100; 

	//start.x = 600;//-2000; //  mm
	//start.y = -700;
	//start.z = -1350;//-350.0;

	//end.x = 2600;//2000;
	//end.y = 700;    
	//end.z = -1220;//-250;   
	
	
	start.x = -50; //  mm
	start.y = -50;
	start.z = -8300;

	end.x = 50;
	end.y = 50;    
	end.z = -8000;  

	//Initialize
	for(int i=0;i<6;i++){
	foot[i].r=25;    //mm
	foot[i].l=0;
	foot[i].d=0;
	foot[i].type = SPHERE;	
	}

	ptpSys->initializePTP(&num,&start,&end);
	  printf("soil surface has been added!\n"); 
}

PtpSoil::PtpSoil() {
	test = 1;
		ptpSys =new PTPSystem();
}

PtpSoil::~PtpSoil() {
		delete ptpSys;
}
  
void PtpSoil::dCollideSoil(utils::Vector *obj_pos, utils::Vector *obj_vel) {  //vecD* obj_pos) {

	
		  for(int i=0;i<6;i++){
		//foot_pos[i] = control->nodes->getPosition(num_foot[i]);
		foot[i].loc.x =0.f;
		foot[i].loc.y =0.f;
		foot[i].loc.z =0.f;	
		
		//foot_vel[i] = control->nodes->getLinearVelocity(num_foot[i]);   //going down in z-axis is minus velocity
		foot[i].vel.x =0.f;
		foot[i].vel.y =0.f;
		foot[i].vel.z =0.f;	
		

	//	printf(" %d (%f %f %f) %f\n",num_foot[i], foot[i].vel.x,foot[i].vel.y,foot[i].vel.z,(foot_pos[i].z()-last_pos[i].z()));	
	//	last_pos[i] = foot_pos[i];
	}
	
		//printf(" ------------\n");	
	foot[0].loc.x =obj_pos->x()*M2MM;
	foot[0].loc.y =obj_pos->y()*M2MM;	
	foot[0].loc.z =obj_pos->z()*M2MM;
	
	foot[0].loc.x =obj_vel->x()*M2MM;
	foot[0].loc.y =obj_vel->y()*M2MM;	
	foot[0].loc.z =obj_vel->z()*M2MM;
	
	
	//foot[0].type = SPHERE;	

	if(!ptpSys->getPTPForce(foot,forceR,forceT)){printf("ERRRROR");}else{	}
	
	if(forceR[0].z > 0.001f) { 
		printf(" --(%f %f %f))--forceR[0] = (%f %f %f)-----\n", foot[0].loc.x, foot[0].loc.y, foot[0].loc.z, 
		forceR[0].x, forceR[0].y, forceR[0].z);	
	}
    
}

  } // end of namespace sim
} // end of namespace mars

