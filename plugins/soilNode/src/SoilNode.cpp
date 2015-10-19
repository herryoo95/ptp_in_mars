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
 * \file SoilNode.cpp
 * \author Yoo, ()
 * \brief PTP
 *
 * Version 0.1
 */


#include "SoilNode.h"
#include <mars/data_broker/DataBrokerInterface.h>
#include <mars/data_broker/DataPackage.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/interfaces/graphics/GraphicsManagerInterface.h>
#include <mars/utils/Vector.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <time.h>

namespace mars {
  namespace plugins {
    namespace soilNode {

      using namespace mars::utils;
      using namespace mars::interfaces;
      
      SoilNode::SoilNode(lib_manager::LibManager *theManager)
        : MarsPluginTemplate(theManager, "SoilNode") {
      }
  
      void SoilNode::init() {
		  

		
// heightmap for mls 
//unvisual surface, it has be be just to call mls height func from DataHeigtht.txt
	//	control->sim->loadScene("box.scn");  
	//	printf("+++box_id = %lu\n", control->nodes->getID("box"));
  //    control->sim->loadScene("dlr.scn");  
 //     control->sim->loadScene("terrain_0_0.scn");        
 //     terrain_id[0] = control->nodes->getID("terrain_0_0") + 1;  //heightmap ID = 102, then it has normal collision
     
 //     printf("..........%lu.....\n", terrain_id[0]);		     //verify that terrain id is 102 

//rotation test of heightmap
		char file[64];
		std::string path = configurationPath;
		
		Vector pos;		
		
      sprintf(file, "../rimres_env/heightmaps/terrain_%d_%d.scn", 0, 0);
      path = configurationPath;	
      path.append(file);
      control->sim->loadScene(path);  
    
      terrain_id[0] = control->nodes->getID("terrain_0_0");
      printf("..........%lu.....\n", terrain_id[0]);
      pos = control->nodes->getPosition(terrain_id[0]);
      printf("heightmap pos (%f %f %f)\n", pos.x(), pos.y(), pos.z());  
      
      sprintf(file, "../rimres_env/heightmaps/terrain_%d_%d.scn", 0, 0);
      path = configurationPath;
      path.append(file);
      control->sim->loadScene(path); 
      terrain_id[1] = control->nodes->getID("terrain_0_0") + 1;      
	  control->nodes->setPosition(terrain_id[1], pos); 
      printf("..........%lu.....\n", terrain_id[1]);	  
      pos = control->nodes->getPosition(terrain_id[1]);	  
      printf("heightmap pos (%f %f %f)\n", pos.x(), pos.y(), pos.z()); 
      
       sprintf(file, "../rimres_env/heightmaps/terrain_%d_%d.scn", 0, 0);
      path = configurationPath;
      path.append(file);
      control->sim->loadScene(path); 
      terrain_id[1] = control->nodes->getID("terrain_0_0") + 2;      
	  control->nodes->setPosition(terrain_id[1], pos); 
      printf("..........%lu.....\n", terrain_id[1]);	  
      pos = control->nodes->getPosition(terrain_id[1]);	  
      printf("heightmap pos (%f %f %f)\n", pos.x(), pos.y(), pos.z());                  
      
/*  
  //heightmap for soil model     
		char file[64];
		std::string path = configurationPath;
		
		Vector pos;		
		
      sprintf(file, "../rimres_env/heightmaps/terrain_%d_%d.scn", 0, 0);
      path = configurationPath;
      path.append(file);
      control->sim->loadScene(path);  
    
      terrain_id[0] = control->nodes->getID("terrain_0_0");
      printf("..........%lu.....\n", terrain_id[0]);
      pos = control->nodes->getPosition(terrain_id[0]);
      printf("heightmap pos (%f %f %f)\n", pos.x(), pos.y(), pos.z());    
  //    pos.z() = pos.z() - 0.1f;

     
      sprintf(file, "../rimres_env_bottom/heightmaps/terrain_%d_%d.scn", 0, 0);
      path = configurationPath;
      path.append(file);
      control->sim->loadScene(path); 
      terrain_id[1] = control->nodes->getID("terrain_0_0") + 1;      
	  control->nodes->setPosition(terrain_id[1], pos); 
      printf("..........%lu.....\n", terrain_id[1]);	  
      pos = control->nodes->getPosition(terrain_id[1]);	  
      printf("heightmap pos (%f %f %f)\n", pos.x(), pos.y(), pos.z());  
      
      control->sim->switchPluginUpdateMode(PLUGIN_GUI_MODE, this);
      	 
      terraingraph_id[0] = control->nodes->getDrawID(terrain_id[0]);//control->nodes->getID("terrain_0_0"));
      foot_id[0] = control->nodes->getID("collision_leg0_foot");     
      foot_id[1] = control->nodes->getID("collision_leg1_foot");  
      foot_id[2] = control->nodes->getID("collision_leg2_foot");  
      foot_id[3] = control->nodes->getID("collision_leg3_foot");  
      foot_id[4] = control->nodes->getID("collision_leg4_foot");  
      foot_id[5] = control->nodes->getID("collision_leg5_foot");      
      
      for(int i=0;i<6;i++) printf("**** foot_id[%lu]\n", foot_id[i]);
                                
  //for(int i=0; i<1; ++i) {
    //for(int k=0; k<1; ++k) {
      //sprintf(file, "../rimres_env/heightmaps/terrain_%d_%d.scn", i, k);
      //path2 = configurationPath;
      //path2.append(file);
      //control->sim->loadScene(path2);   
    //}
  //}	  
*/		
	
      }

      void SoilNode::reset() {
      }

      SoilNode::~SoilNode() {
      }


      void SoilNode::update(sReal time_ms) {
        static sReal s = 0;

	  //Vector pos[i];
      //for(int i=0;i<6;i++) pos[i] = control->nodes->getPosition(foot_id[i]);

      //for(int i=0;i<6;i++) control->graphics->collideSphere(terraingraph_id[0], 
			//control->nodes->getPosition(foot_id[i]), 0.025);

	  
      }

      void SoilNode::receiveData(const data_broker::DataInfo& info,
                                    const data_broker::DataPackage& package,
                                    int id) {
        // package.get("force1/x", force);
      }
  
      void SoilNode::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) {

        if(_property.paramId == example.paramId) {
          example.dValue = _property.dValue;
        }
      }

    } // end of namespace soilNode
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::soilNode::SoilNode);
CREATE_LIB(mars::plugins::soilNode::SoilNode);
