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
 * \file PTPGraphics.h
 * \author Yoo, ()
 * \brief shows
 *
 * Version 0.1
 */

#ifndef MARS_PLUGINS_PTPGRAPHICS_H
#define MARS_PLUGINS_PTPGRAPHICS_H

#ifdef _PRINT_HEADER_
  #warning "PTPGraphics.h"
#endif

// set define if you want to extend the gui
//#define PLUGIN_WITH_MARS_GUI
#include <mars/interfaces/sim/MarsPluginTemplate.h>
#include <mars/interfaces/graphics/GraphicsManagerInterface.h>
#include <mars/interfaces/MARSDefs.h>
#include <mars/data_broker/ReceiverInterface.h>
#include <mars/cfg_manager/CFGManagerInterface.h>

#include "../../../sim/src/ptpSoil/PTPInterface.hpp"
#include <osg/Node>
#include <osg/Group>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Texture2D>
#include <osgDB/ReadFile> 
#include <osgViewer/Viewer>
#include <osg/PositionAttitudeTransform>
#include <osgGA/TrackballManipulator>


#include <iostream>
#include <list>

#include <string>

namespace mars {

  namespace plugins {
    namespace PTPGraphics {
		
	  using namespace mars::utils;
      using namespace mars::interfaces;
      using namespace std;
		
      // inherit from MarsPluginTemplateGUI for extending the gui
      class PTPGraphics: public mars::interfaces::MarsPluginTemplate,
        public mars::data_broker::ReceiverInterface,
        // for gui
        // public mars::main_gui::MenuInterface,
        public mars::cfg_manager::CFGClient {

      public:
        PTPGraphics(lib_manager::LibManager *theManager);
        ~PTPGraphics();

        // LibInterface methods
        int getLibVersion() const
        { return 1; }
        const std::string getLibName() const
        { return std::string("PTPGraphics"); }
        CREATE_MODULE_INFO();

        // MarsPlugin methods
        void init();
        void reset();
        void update(mars::interfaces::sReal time_ms);
 void drawSoil(int foot_id);        

        // DataBrokerReceiver methods
        virtual void receiveData(const data_broker::DataInfo &info,
                                 const data_broker::DataPackage &package,
                                 int callbackParam);
        // CFGClient methods
        virtual void cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property);

        // MenuInterface methods
        //void menuAction(int action, bool checked = false);

        // PTPGraphics methods
        osg::ref_ptr<osg::Geode> geode_pt;
		bool prev_drawing;
		
        sReal disp_time;
        
        //sim::PTPInterface* test; 
        

      private:
        cfg_manager::cfgPropertyStruct example;

      }; // end of class definition PTPGraphics

    } // end of namespace PTPGraphics
  } // end of namespace plugins
} // end of namespace mars

#endif // MARS_PLUGINS_PTPGRAPHICS_H
