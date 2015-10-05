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


#include "PtpSoil.hpp"
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
  namespace plugins {
    namespace ptpSoil {

      using namespace mars::utils;
      using namespace mars::interfaces;
      
void PtpSoil::configSim()
{
	Value = 2;  //if value=1 foot has vertical movement and if value =2 foot has lateral movement 
	
	step=0;

	t_J=0;
	t_K=0;
	t_debug=0;
	#ifdef SPHERE_EXPERIMENT2
		t_p=0;
		t_c=0;
		t_f=1;
		t_b=1;
		t_a=0; 
		t_x=1;
		t_d=1;
		t_t=0;
		t_s=0;
		t_j=0;
		t_k=0;
		t_m=5;
		t_v=0;
	#endif
	#ifndef SPHERE_EXPERIMENT2
		t_p=0;
		t_c=0;
		t_f=2;
		t_b=1;
		t_a=0;
		t_x=1;
		t_d=1;
		t_t=0;
		t_s=0;
		t_j=0;
		t_k=0;
		t_m=5;
		t_v=0;
	#endif
	v_x=0;
	v_y=0;
	v_z=0;

	step=0;
	step_size=0.01f;  //sec
	msec=0.f;
	vel_set = 1.f;

}

void PtpSoil::setSoil(){
	vecN num;
	vecD start;
	vecD end;
	
//	foot.mass = 0.456;
	
#ifdef SPHERE_EXPERIMENT
	//Number of Cells
	num.x = 100; 
	num.y = 50; 
	num.z = 5; 

	start.x = 0; //  mm
	start.y = 0;
	start.z = 0.0;

	end.x = 1;
	end.y = 0.5;    
	end.z = 0.05;   

	//Initialize
	for(int i=0;i<6;i++){
	foot[i].r=0.025;    //mm
	foot[i].l=0;
	foot[i].d=0;
	foot[i].type = SPHERE;	
	}
	//foot.loc.x =0;
	//foot.loc.y =0;
	//foot.loc.z =25.1;


#endif

//#ifdef SPHERE_EXPERIMENT2
	////Number of Cells
	//num.x = 10;
	//num.y = 5;
	//num.z = 10;

	//foot.mass = 0;

	//start.x = -30;
	//start.y = -60;
	//start.z = -30;

	//end.x = 30;
	//end.y = 0;
	//end.z = 30;

	////Initialize
	//foot.r=25;
	//foot.l=0;
	//foot.d=0;

	////foot.loc.x =0;
	////foot.loc.y =26;
	////foot.loc.z =0;
	

	//foot.type = SPHERE; //
	////foot.type = CYLINDER  
//#endif

	ptpSys->initializePTP(&num,&start,&end);
	
	//time_t timer;
	//struct tm *t;

	//timer = time(NULL); 
	//t = localtime(&timer); 	
//	snprintf(filename,sizeof(filename),"result%d_%d_%d-%d_%d_%d.csv", t->tm_year + 1900,t->tm_mon + 1,t->tm_mday,t->tm_hour,t->tm_min,t->tm_sec);
//	snprintf(filename_cell,sizeof(filename),"result%d_%d_%d-%d_%d_%d_cell.csv", t->tm_year + 1900,t->tm_mon + 1,t->tm_mday,t->tm_hour,t->tm_min,t->tm_sec);
//	out = fopen(filename, "w");

//	fprintf(out,"msec,force_x,force_y,force_z,force_T_x,force_T_y,force_T_z,foot_loc_x,foot_loc_y,foot_loc_z\n");
//	fclose(out); 
}

void PtpSoil::genMovement()  //SPHARE, generated movement
{

	if(!ptpSys->getPTPForce(foot,forceR,forceT)){printf("ERRRROR");}else{
		//out = fopen(filename, "a+");
		//msec += step_size*100;
		//fprintf(out,"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",msec*10,forceR.x,forceR.y,forceR.z,forceT.x,forceT.y,forceT.z,foot.loc.x,foot.loc.y,foot.loc.z);
		//fclose(out); 
	}
//	footLast[i] = foot[i];
	
//	printf("status =%d\n", status);

}



PtpSoil::PtpSoil(lib_manager::LibManager *theManager)
        : MarsPluginTemplate(theManager, "PtpSoil") {
		ptpSys =new PTPSystem();
		kf=new KalmanFilter();
		ID =2;

}

   
 void PtpSoil::showptp(){

//	 osg::ref_ptr<osg::Group> root (new osg::Group());
     osg::ref_ptr<osg::Geode> geode (new osg::Geode());
     geode_pt = geode;
     osg::ref_ptr<osg::Geometry> geom (new osg::Geometry());
     osg::ref_ptr<osg::Vec3Array> vertices (new osg::Vec3Array());
     osg::ref_ptr<osg::Vec4Array> colors (new osg::Vec4Array());

     Vector ptp_pos;
  
       for(int i = 0;i<ptpSys->ptp->n.x;i++){
		for(int j = 0;j<ptpSys->ptp->n.y;j++){
		  for(int k = 0;k<ptpSys->ptp->n.z;k++){
			  
       	////Only Collision Particle
		//if(ptpSys->particle[i][j][k].isCollision){
		  //ptp_pos.x() = ptpSys->particle[i][j][k].loc.x;
		  //ptp_pos.y() = ptpSys->particle[i][j][k].loc.y;
		  //ptp_pos.z() = ptpSys->particle[i][j][k].loc.z; 			
	   
	   //vertices->push_back (osg::Vec3 (ptp_pos.x(), ptp_pos.y(), ptp_pos.z()));
       //colors->push_back (osg::Vec4f (1.0f,0.0f, 0.0f,1.0f));
		//}else{			  
		  ptp_pos.x() = ptpSys->particle[i][j][k].loc.x;
		  ptp_pos.y() = ptpSys->particle[i][j][k].loc.y;
		  ptp_pos.z() = ptpSys->particle[i][j][k].loc.z; 
  
       vertices->push_back (osg::Vec3 (ptp_pos.x(), ptp_pos.y(), ptp_pos.z()));
       colors->push_back (osg::Vec4f (1.0f,1.0f, 1.0f,1.0f));
		//}
       
		  }
		}
	  }
	  
	    
	  
    geom->setVertexArray(vertices.get());
    geom->setColorArray(colors.get());

	geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,vertices->size()));
    geode->addDrawable(geom.get());
 //   root->addChild(geode.get());

    control->graphics->addOSGNode(geode.get());  

}


PtpSoil::~PtpSoil() {
		delete ptpSys;
		delete kf;

}
  
void PtpSoil::init() {
		//char file[64];

		char *conPath = getenv("ROCK_CONFIGURATION_PATH");
		if(conPath) {
				configurationPath = conPath;
		} else {
				configurationPath = "..";
		}
		
	  	
      //foot ID (f0,f1,f2,f3,f4,f5) -> (34,48,61,73,87,99) 
      configSim();
      setSoil();
	  const Vector f0_pos = control->nodes->getPosition(34);
	  foot[0].loc.x =f0_pos.x();
	  foot[0].loc.y =f0_pos.y();
	  foot[0].loc.z =f0_pos.z();	
	  
	  const Vector f2_pos = control->nodes->getPosition(61);
	  foot[2].loc.x =f2_pos.x();
	  foot[2].loc.y =f2_pos.y();
	  foot[2].loc.z =f2_pos.z();
	  foot[1].loc.x=1;	
	  foot[3].loc.x=3;
	  foot[4].loc.x=4;
	  foot[5].loc.x=5;
	  
	
      showptp();
         
      }

void PtpSoil::reset() {
}

void PtpSoil::update(sReal time_ms) {
			
        // control->motors->setMotorValue(id, value);
        
	  genMovement();
        
	  const Vector f0_pos = control->nodes->getPosition(34);
	  foot[0].loc.x =f0_pos.x();
	  foot[0].loc.y =f0_pos.y();
	  foot[0].loc.z =f0_pos.z();	
	  
	  const Vector f2_pos = control->nodes->getPosition(61);
	  foot[2].loc.x =f2_pos.x();
	  foot[2].loc.y =f2_pos.y();
	  foot[2].loc.z =f2_pos.z();	   

      
      disp_time += time_ms;
      if(disp_time > 100.0f){
		control->graphics->removeOSGNode(geode_pt.get()); 
		showptp();
			  printf(" (%f %f %f)\n",foot[2].loc.x,foot[2].loc.y,foot[2].loc.z);
	  disp_time = 0;
	  }
	  
	  //reaction forces to the feet
	   
	  
}

void PtpSoil::receiveData(const data_broker::DataInfo& info,
                          const data_broker::DataPackage& package,
                                    int id) {
        // package.get("force1/x", force);
}
  
void PtpSoil::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) {

        if(_property.paramId == example.paramId) {
          example.dValue = _property.dValue;
        }
}

    } // end of namespace ptpSoil
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::ptpSoil::PtpSoil);
CREATE_LIB(mars::plugins::ptpSoil::PtpSoil);
