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
 * \file MlsDraw.cpp
 * \author Yong-Ho (yong-ho.yoo@dfki.de)
 * \brief Draw
 *
 * Version 0.1
 */


#include "MlsDraw.h"
#include <mars/data_broker/DataBrokerInterface.h>
#include <mars/data_broker/DataPackage.h>

namespace mars {
  namespace plugins {
    namespace MlsDraw {

      using namespace mars::utils;
      using namespace mars::interfaces;

      MlsDraw::MlsDraw(lib_manager::LibManager *theManager)
        : MarsPluginTemplate(theManager, "MlsDraw") {
      }
 
 
  
      void MlsDraw::init() {
		  
    	FILE *fp; 
		double data[3];
		char buf[50000];
		double mls_mean[20000];
			
	char *ptr;
	char *err;
	double f;
	int i,j,x,y=0;

	fp=fopen("DataHeight.txt", "r"); 
	while(!feof(fp)) 		
	{
			fgets(buf, 50, fp);
			//if(buf[0]=='%' || buf[0] == 10) continue;
			i=0;
			ptr = strtok( buf, ",");
			
			do{

				f = strtod( ptr, &err);
				data[i] = f;
				//printf("%f %s\t",f,err);
				i++;
			}while(ptr = strtok(NULL, ","));

			mls_mean[j++] = data[2];
			//printf("(%d,%d,%f)\n", (int)data[0],(int)data[1],mls_mean[j++]);			
		
	}

		fclose(fp);				

      int count = 0;

      int height = 140;
      int width =140;      
		  

//	 if(prev_drawing) control->graphics->removeOSGNode(geode_pt.get()); 
		  
     osg::ref_ptr<osg::Geode> geode (new osg::Geode());
     geode_pt = geode;
     
     osg::ref_ptr<osg::Geometry> particleGeom (new osg::Geometry());
     osg::ref_ptr<osg::Vec3Array> particleVertices (new osg::Vec3Array());
     osg::ref_ptr<osg::Vec4Array> particleColors (new osg::Vec4Array());
     
     Vector pos;

       
       for(x=0; x<height; x++) {
        for(y=0; y<width; y++) {
          
          pos.x() = (double)x*0.1f;
          pos.y() = (double)y*0.1f;
          pos.z() = (double)mls_mean[count++];
        //  printf("(%d %d %f),",x,y,mls_mean[count++]);
		  //count++;
 
      
	   //should exchange x and y like height_dat array	  
       particleVertices->push_back (osg::Vec3 (pos.y()-10.f, pos.x()-7.f, pos.z()-1.0));   

       
       particleColors->push_back (osg::Vec4f (1.0f,0.0f, 0.0f,1.0f));
       
              }
      }  
    
	  
    particleGeom->setVertexArray(particleVertices.get());
    particleGeom->setColorArray(particleColors.get());

	particleGeom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    particleGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,particleVertices->size()));
    geode_pt->addDrawable(particleGeom.get());


    control->graphics->addOSGNode(geode.get());  
  //  prev_drawing = true;





      }

      void MlsDraw::reset() {
      }

      MlsDraw::~MlsDraw() {
      }


      void MlsDraw::update(sReal time_ms) {

        // control->motors->setMotorValue(id, value);
      }

      void MlsDraw::receiveData(const data_broker::DataInfo& info,
                                    const data_broker::DataPackage& package,
                                    int id) {
        // package.get("force1/x", force);
      }
  
      void MlsDraw::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) {

        if(_property.paramId == example.paramId) {
          example.dValue = _property.dValue;
        }
      }

    } // end of namespace MlsDraw
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::MlsDraw::MlsDraw);
CREATE_LIB(mars::plugins::MlsDraw::MlsDraw);
