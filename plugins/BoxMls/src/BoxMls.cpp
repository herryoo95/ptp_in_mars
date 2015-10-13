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
 * \file BoxMls.cpp
 * \author Yong-Ho (yong-ho.yoo@dfki.de)
 * \brief mls
 *
 * Version 0.1
 */


#include "BoxMls.h"
#include <mars/data_broker/DataBrokerInterface.h>
#include <mars/data_broker/DataPackage.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <boost/lexical_cast.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>

namespace mars {
  namespace plugins {
    namespace BoxMls {

      using namespace mars::utils;
      using namespace mars::interfaces;

      using namespace mars::sim;
      using namespace std;      

      BoxMls::BoxMls(lib_manager::LibManager *theManager)
        : MarsPluginTemplate(theManager, "BoxMls") {
      }
  
      void BoxMls::init() {
		  
		  
       NodeData nodeS;
       NodeData *node = &nodeS;
	char *ptr;
	char *err;
	double f;
	int i,j=0;

	fp=fopen("DataHeight.txt", "r"); 
	while(!feof(fp)) 		
	{
			fgets(buf, 50, fp);
			i=0;
			ptr = strtok( buf, ",");
			do{
				f = strtod( ptr, &err);
				data[i] = f;
				i++;
			}while(ptr = strtok(NULL, ","));
			mls_mean[j++] = data[2];
			//printf("....mls_mean = %f\n", data[2]);
	}
	  fclose(fp);						
    
      int count = 0;  
      int x, y;
      int width =140;
      int height =140;
   
  for(x=0; x<height; x++) {
  for(y=0; y<width; y++) {

		  
		//printf("....mls_mean = %f\n", node->pos.z());

        const std::string name = "box" + boost::lexical_cast<string>(x*width+y);
          
		node->pos.x() = (double)x*0.1;
		node->pos.y() = (double)y*0.1;		
        node->pos.z() = (double)mls_mean[x*width+y]-(double)2;
        
       node->name = name;
       node->movable = true;
       node->physicMode = NODE_TYPE_BOX;
       node->index=0;
       node->noPhysical = false;
       node->inertia_set=true;
		node->ext.x() = (double)0.1;       
		node->ext.y() = (double)0.1;
		node->ext.z() = (double)0.1;  
       NodeType type = node->physicMode;
       node->mass = 1.0f;
       node->initPrimitive(type, node->ext, node->mass);   
       control->nodes->addNode(node,false);

  }  
  }


      }

      void BoxMls::reset() {
      }

      BoxMls::~BoxMls() {
      }


      void BoxMls::update(sReal time_ms) {

        // control->motors->setMotorValue(id, value);
      }

      void BoxMls::receiveData(const data_broker::DataInfo& info,
                                    const data_broker::DataPackage& package,
                                    int id) {
        // package.get("force1/x", force);
      }
  
      void BoxMls::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) {

        if(_property.paramId == example.paramId) {
          example.dValue = _property.dValue;
        }
      }

    } // end of namespace BoxMls
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::BoxMls::BoxMls);
CREATE_LIB(mars::plugins::BoxMls::BoxMls);
