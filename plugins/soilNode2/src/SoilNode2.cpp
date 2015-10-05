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
 * \file SoilNode2.cpp
 * \author yoo ()
 * \brief plane
 *
 * Version 0.1
 */


#include "SoilNode2.h"
#include <mars/data_broker/DataBrokerInterface.h>
#include <mars/data_broker/DataPackage.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/utils/Vector.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <time.h>

namespace mars {
  namespace plugins {
    namespace soilNode2 {

      using namespace mars::utils;
      using namespace mars::interfaces;

      SoilNode2::SoilNode2(lib_manager::LibManager *theManager)
        : MarsPluginTemplate(theManager, "SoilNode2") {
      }
  
      void SoilNode2::init() {
	
		Vector pos;
		unsigned long obj_id[6];		

      control->sim->loadScene("dlr.scn");  
      obj_id[0] = control->nodes->getID("terrain");
      pos = control->nodes->getPosition(obj_id[0]);
      printf("%lu terrain pos (%f %f %f)\n", obj_id[0], pos.x(), pos.y(), pos.z());    
      pos.z() = pos.z() - 0.10f;
      
      control->sim->loadScene("dlr.scn"); 
      obj_id[1] = control->nodes->getID("terrain") + 1;      
	  control->nodes->setPosition(obj_id[1], pos);  
      pos = control->nodes->getPosition(obj_id[1]);
      printf("%lu terrain pos  (%f %f %f)\n", obj_id[1], pos.x(), pos.y(), pos.z());  	    
      
      //control->sim->loadScene("sphere.scn"); 
      //pos.z() = 0.10f;   
      //control->nodes->setPosition(3, pos);  
      //pos = control->nodes->getPosition(3);
      //printf("sphere pos (%f %f %f)\n", pos.x(), pos.y(), pos.z());  	   
      

      }

      void SoilNode2::reset() {
      }

      SoilNode2::~SoilNode2() {
      }


      void SoilNode2::update(sReal time_ms) {

        // control->motors->setMotorValue(id, value);
      }

      void SoilNode2::receiveData(const data_broker::DataInfo& info,
                                    const data_broker::DataPackage& package,
                                    int id) {
        // package.get("force1/x", force);
      }
  
      void SoilNode2::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) {

        if(_property.paramId == example.paramId) {
          example.dValue = _property.dValue;
        }
      }

    } // end of namespace soilNode2
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::soilNode2::SoilNode2);
CREATE_LIB(mars::plugins::soilNode2::SoilNode2);
