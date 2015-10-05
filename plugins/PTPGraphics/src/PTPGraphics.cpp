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
 * \file PTPGraphics.cpp
 * \author Yoo, ()
 * \brief shows
 *
 * Version 0.1
 */

#include <mars/interfaces/sim/NodeManagerInterface.h>
#include "PTPGraphics.h"
#include <mars/data_broker/DataBrokerInterface.h>
#include <mars/data_broker/DataPackage.h>
#include <mars/utils/Vector.h>

namespace mars {
  namespace plugins {
    namespace PTPGraphics {

      using namespace mars::utils;
      using namespace mars::interfaces;
//      using namespace mars::sim;      

      PTPGraphics::PTPGraphics(lib_manager::LibManager *theManager)
        : MarsPluginTemplate(theManager, "PTPGraphics") {
			
				prev_drawing = false;	
				disp_time =0;
      }
  
  void PTPGraphics::drawSoil(int foot_id){

	 if(prev_drawing) control->graphics->removeOSGNode(geode_pt.get()); 
		  
     osg::ref_ptr<osg::Geode> geode (new osg::Geode());
     geode_pt = geode;
     
     osg::ref_ptr<osg::Geometry> particleGeom (new osg::Geometry());
     osg::ref_ptr<osg::Vec3Array> particleVertices (new osg::Vec3Array());
     osg::ref_ptr<osg::Vec4Array> particleColors (new osg::Vec4Array());
     
     Vector ptp_pos;
     VectorN ptpsize;
  for(int id=0;id<6;id++){
	  
     ptpsize = control->nodes->getParticleSize(id);
     
       for(int i = 0;i<ptpsize.x();i++){
		for(int j = 0;j<ptpsize.y();j++){
		  for(int k =0;k<ptpsize.z();k++){
		  //for(int k =ptpsize.z()-1;k<ptpsize.z();k++){			  
			  
  
		  ptp_pos = control->nodes->getParticlePosition(id, i,j,k);
		  ptp_pos = ptp_pos*MM2M;
		  
       particleVertices->push_back (osg::Vec3 (ptp_pos.x(), ptp_pos.y(), ptp_pos.z()));
       particleColors->push_back (osg::Vec4f (1.0f,0.0f, 0.0f,1.0f));
    
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
    prev_drawing = true;

}
  
  
      void PTPGraphics::init() {
	
      }

      void PTPGraphics::reset() {
      }

      PTPGraphics::~PTPGraphics() {
      }


      void PTPGraphics::update(sReal time_ms) {
			//	drawSoil(control, soil);
				
		disp_time += time_ms;
	    if(disp_time > 500.0f)  {
		drawSoil(0);
		disp_time = 0;
		}
		

	}
      void PTPGraphics::receiveData(const data_broker::DataInfo& info,
                                    const data_broker::DataPackage& package,
                                    int id) {
        // package.get("force1/x", force);
      }
  
      void PTPGraphics::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) {

        if(_property.paramId == example.paramId) {
          example.dValue = _property.dValue;
        }
      }

    } // end of namespace PTPGraphics
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::PTPGraphics::PTPGraphics);
CREATE_LIB(mars::plugins::PTPGraphics::PTPGraphics);
