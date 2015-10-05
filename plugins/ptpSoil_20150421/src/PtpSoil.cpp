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
	num.x = 200; 
	num.y = 140; 
	num.z = 10; 

	start.x = 600;//-2000; //  mm
	start.y = -700;
	start.z = -350;//-350.0;

	end.x = 2600;//2000;
	end.y = 700;    
	end.z = -220;//-250;   

	//Initialize
	for(int i=0;i<6;i++){
	foot[i].r=25;    //mm
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

PtpSoil::PtpSoil(lib_manager::LibManager *theManager)
        : MarsPluginTemplate(theManager, "PtpSoil") {
		ptpSys =new PTPSystem();
		kf=new KalmanFilter();
		ID =2;

}

 
void PtpSoil::drawCell(Vector* cellPos, Vector* cellSize){
	osg::ref_ptr<osg::Geometry> cellGeom (new osg::Geometry());
 	osg::ref_ptr<osg::Vec3Array> cellVertices (new osg::Vec3Array());
    osg::ref_ptr<osg::Vec4Array> cellColors (new osg::Vec4Array());
    
    cellVertices->push_back( osg::Vec3(cellPos->x(), cellPos->y(), cellPos->z()));
	cellVertices->push_back( osg::Vec3(cellPos->x()+cellSize->x(), cellPos->y(), cellPos->z()));
	cellVertices->push_back( osg::Vec3(cellPos->x()+cellSize->x(), cellPos->y()+cellSize->y(), cellPos->z()));
	cellVertices->push_back( osg::Vec3(cellPos->x(), cellPos->y()+cellSize->y(), cellPos->z()));
	cellVertices->push_back( osg::Vec3(cellPos->x(), cellPos->y(), cellPos->z()+cellSize->z()));
	cellVertices->push_back( osg::Vec3(cellPos->x()+cellSize->x(), cellPos->y(), cellPos->z()+cellSize->z()));
	cellVertices->push_back( osg::Vec3(cellPos->x()+cellSize->x(), cellPos->y()+cellSize->y(), cellPos->z()+cellSize->z()));
	cellVertices->push_back( osg::Vec3(cellPos->x(), cellPos->y()+cellSize->y(), cellPos->z()+cellSize->z()));
    
	//cellVertices->push_back( osg::Vec3(0, 0, 0));
	//cellVertices->push_back( osg::Vec3(1, 0, 0));
	//cellVertices->push_back( osg::Vec3(1, 1, 0));
	//cellVertices->push_back( osg::Vec3(0,1,0));
	//cellVertices->push_back( osg::Vec3(0, 0, 1));
	//cellVertices->push_back( osg::Vec3(1, 0, 1));
	//cellVertices->push_back( osg::Vec3(1, 1, 1));
	//cellVertices->push_back( osg::Vec3(0,1,1));
	
	cellGeom->setVertexArray(cellVertices.get());

	osg::ref_ptr<osg::DrawElementsUInt> cellBottom (new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0));
	cellBottom->push_back(3);
	cellBottom->push_back(2);
	cellBottom->push_back(1);
	cellBottom->push_back(0);
	cellGeom->addPrimitiveSet(cellBottom.get());
	
	osg::ref_ptr<osg::DrawElementsUInt> cellFront (new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0));
	cellFront->push_back(4);
	cellFront->push_back(5);
	cellFront->push_back(1);
	cellFront->push_back(0);
	cellGeom->addPrimitiveSet(cellFront.get());
	
	osg::ref_ptr<osg::DrawElementsUInt> cellLeft (new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0));	
	cellLeft->push_back(6);
	cellLeft->push_back(2);
	cellLeft->push_back(1);
	cellLeft->push_back(5);
	cellGeom->addPrimitiveSet(cellLeft.get());
	
	osg::ref_ptr<osg::DrawElementsUInt> cellRight (new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0));
	cellRight->push_back(4);
	cellRight->push_back(7);
	cellRight->push_back(3);
	cellRight->push_back(0);
	cellGeom->addPrimitiveSet(cellRight.get());
	
	osg::ref_ptr<osg::DrawElementsUInt> cellBack (new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0));
	cellBack->push_back(7);
	cellBack->push_back(6);
	cellBack->push_back(2);
	cellBack->push_back(3);
	cellGeom->addPrimitiveSet(cellBack.get());
	
	osg::ref_ptr<osg::DrawElementsUInt> cellTop (new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0));
	cellTop->push_back(7);
	cellTop->push_back(6);
	cellTop->push_back(5);
	cellTop->push_back(4);
	cellGeom->addPrimitiveSet(cellTop.get());
	
    cellColors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f) ); //index 0 red
    cellColors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f) ); //index 0 red
    cellColors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f) ); //index 0 red
    cellColors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f) ); //index 0 red
    cellColors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f) ); //index 0 red
    cellColors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f) ); //index 0 red
    
    cellGeom->setColorArray(cellColors.get());
    cellGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
    //cellGeom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    geode_pt->addDrawable(cellGeom.get());
    
}

  
void PtpSoil::drawSoil(){

     osg::ref_ptr<osg::Geode> geode (new osg::Geode());
     geode_pt = geode;
     
     osg::ref_ptr<osg::Geometry> particleGeom (new osg::Geometry());
     osg::ref_ptr<osg::Vec3Array> particleVertices (new osg::Vec3Array());
     osg::ref_ptr<osg::Vec4Array> particleColors (new osg::Vec4Array());
     
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
		  ptp_pos.x() = ptpSys->particle[i][j][k].loc.x*MM2M;
		  ptp_pos.y() = ptpSys->particle[i][j][k].loc.y*MM2M;
		  ptp_pos.z() = ptpSys->particle[i][j][k].loc.z*MM2M; 
  
       particleVertices->push_back (osg::Vec3 (ptp_pos.x(), ptp_pos.y(), ptp_pos.z()));
       particleColors->push_back (osg::Vec4f (1.0f,1.0f, 1.0f,1.0f));
		//}
	     if(ptpSys->cell[i][j][k].CollisionParticleNum){
			Vector cellPos,cellSize; 
			cellPos.x() = ptpSys->cell[i][j][k].start.x*MM2M;
			cellPos.y() = ptpSys->cell[i][j][k].start.y*MM2M;
			cellPos.z() = ptpSys->cell[i][j][k].start.z*MM2M;
			 
			cellSize.x() = (ptpSys->cell[i][j][k].start.x-ptpSys->cell[i][j][k].end.x)*MM2M/MAG;
			cellSize.y() = (ptpSys->cell[i][j][k].start.y-ptpSys->cell[i][j][k].end.y)*MM2M/MAG;
			cellSize.z() = (ptpSys->cell[i][j][k].start.z-ptpSys->cell[i][j][k].end.z)*MM2M/MAG;

            drawCell(&cellPos, &cellSize);
		}
       
		  }
		}
	  }
	  

	  
    particleGeom->setVertexArray(particleVertices.get());
    particleGeom->setColorArray(particleColors.get());

	particleGeom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    particleGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,particleVertices->size()));
    geode_pt->addDrawable(particleGeom.get());


    control->graphics->addOSGNode(geode.get());  

}


PtpSoil::~PtpSoil() {
		delete ptpSys;
		delete kf;

}
  
void PtpSoil::init() {
		//char file[64];
		
		time_t timer;
		struct tm *t;

		char *conPath = getenv("ROCK_CONFIGURATION_PATH");
		if(conPath) {
				configurationPath = conPath;
		} else {
				configurationPath = "..";
		}
		
	  	
      //foot ID (f0,f1,f2,f3,f4,f5) -> (34,48,61,73,87,99) 
	  const int num_foot[6]={34,48,61,73,87,99};
	  Vector foot_pos[6];
      configSim();
      setSoil();

	  for(int i=0;i<6;i++){
		foot_pos[i] = control->nodes->getPosition(num_foot[i]);
		foot[i].loc.x =foot_pos[i].x()*M2MM;
		foot[i].loc.y =foot_pos[i].y()*M2MM;
		foot[i].loc.z =foot_pos[i].z()*M2MM;	
	
//		printf(" %d (%f %f %f)\n",num_foot[i], foot[i].loc.x,foot[i].loc.y,foot[i].loc.z);	
	  }

      drawSoil();
 
	timer = time(NULL); 
	t = localtime(&timer); 	
	snprintf(filename,sizeof(filename),"result%d_%d_%d-%d_%d_%d.csv", t->tm_year + 1900,t->tm_mon + 1,t->tm_mday,t->tm_hour,t->tm_min,t->tm_sec);
	snprintf(filename_cell,sizeof(filename),"result%d_%d_%d-%d_%d_%d_cell.csv", t->tm_year + 1900,t->tm_mon + 1,t->tm_mday,t->tm_hour,t->tm_min,t->tm_sec);
	out = fopen(filename, "w");
	fprintf(out,"msec,force_x,force_y,force_z,force_T_x,force_T_y,force_T_z,foot_loc_x,foot_loc_y,foot_loc_z\n");
	fclose(out); 
         
      }

void PtpSoil::reset() {
}

void PtpSoil::update(sReal time_ms) {
	
		const int num_foot[6]={34,48,61,73,87,99};
	    Vector foot_pos[6], foot_vel[6];
	    vecD filter[6];

	  for(int i=0;i<6;i++){
		foot_pos[i] = control->nodes->getPosition(num_foot[i]);
		foot[i].loc.x =foot_pos[i].x()*M2MM;
		foot[i].loc.y =foot_pos[i].y()*M2MM;
		foot[i].loc.z =foot_pos[i].z()*M2MM;	
		
		foot_vel[i] = control->nodes->getLinearVelocity(num_foot[i]);   //going down in z-axis is minus velocity
		foot[i].vel.x =foot_vel[i].x()*M2MM;
		foot[i].vel.y =foot_vel[i].y()*M2MM;
		foot[i].vel.z =foot_vel[i].z()*M2MM;	
		
	//	printf(" %d (%f %f %f) %f\n",num_foot[i], foot[i].vel.x,foot[i].vel.y,foot[i].vel.z,(foot_pos[i].z()-last_pos[i].z()));	
		last_pos[i] = foot_pos[i];
	}
		record_time += time_ms;
	if(!ptpSys->getPTPForce(foot,forceR,forceT)){printf("ERRRROR");}else{
		//out = fopen(filename, "a+");
		
		//Adapting Force
  	  Vector footForce[6];
  	  
  	  for(int i=0;i<6;i++){
		double dampX = 10;
		double dampY = 10;
		double dampZ = 30;

		footForce[i].x() = -forceR[i].x;// + dampX*(foot_pos[i].x()-last_pos[i].x());
		footForce[i].y() = -forceR[i].y;// + dampY*(foot_pos[i].y()-last_pos[i].y());
		footForce[i].z() = -forceR[i].z;// + dampZ*(foot_pos[i].z()-last_pos[i].z());
		
		if(footForce[i].z() < 620) control->nodes->applyForce(num_foot[i], footForce[i]);
		//if(footForce[i].z() < 320) control->nodes->applyForce(num_foot[i], footForce[i]);		
	//	printf(" (%d = %f %f)\n",i, foot_pos[0].z(),footForce[0].z());
		
	  }
	  
	  //double f_all = forceR[0].z + forceR[1].z +forceR[2].z +forceR[3].z +forceR[4].z +forceR[5].z;
	  
  	 // if(record_time > 200){
  	 
  	 	//vecD filtered[6];
  	
		//for(int i=0; i<6;i++) {
			 ////filtered[i].x = footForce[i].x();
			 ////filtered[i].y = footForce[i].y();
			 ////filtered[i].z = footForce[i].z();
			 
			 ////filtered[i] = kf->kalman(filtered[i],1,ptpSys->ptputil);
			 
//////			 footForce[i].x() = filtered[i].x; 
//////			 footForce[i].y() = filtered[i].y; 
			 ////footForce[i].z() = filtered[i].z; 		 

		////control->nodes->applyForce(num_foot[i], footForce[i]);
		//}
	  
      disp_time += time_ms;
      if(disp_time > 100.0f){
		control->graphics->removeOSGNode(geode_pt.get()); 
		drawSoil();
		//	  printf(" (%f %f %f)\n",foot[2].loc.x,foot[2].loc.y,foot[2].loc.z);
	  disp_time = 0;
	  }
		
		//fprintf(out,"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",record_time,forceR[1].x,forceR[1].y,forceR[1].z,forceT[1].x,forceT[1].y,forceT[1].z,
		//							foot[1].loc.x,foot[1].loc.y,foot[1].loc.z,foot[1].vel.x,foot[1].vel.y,foot[1].vel.z);
		//fclose(out); 
	}
  	  
 
	  
  
}

void PtpSoil::receiveData(const data_broker::DataInfo& info,
                          const data_broker::DataPackage& package,
                                    int id) {
        // package.get("force1/x", force);

        //package.get("00087_collision_leg4_foot/position/x", &tempX);
        //printf("---%f\n", tempX);
        		//std::cout << "Vert" << " : " << tempX << "\n";
        
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
