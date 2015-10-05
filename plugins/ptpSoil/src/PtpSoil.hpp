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
 * \file PtpSoil.h
 * \author Yong-Ho (yong-ho.yoo@dfki.de)
 * \brief soil
 *
 * Version 0.1
 */

#ifndef MARS_PLUGINS_PTPSOIL_H
#define MARS_PLUGINS_PTPSOIL_H

#ifdef _PRINT_HEADER_
  #warning "PtpSoil.h"
#endif

// set define if you want to extend the gui
//#define PLUGIN_WITH_MARS_GUI
#include <mars/interfaces/sim/MarsPluginTemplate.h>
#include <mars/interfaces/graphics/GraphicsManagerInterface.h>
//#include <mars/interfaces/MARSDefs.h>
#include <mars/data_broker/ReceiverInterface.h>
#include <mars/cfg_manager/CFGManagerInterface.h>
#include <string>
#include "PTPInterface.hpp"
#include <time.h>

#include <iostream>
#include <list>

#include <osg/Node>
#include <osg/Group>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Texture2D>
#include <osgDB/ReadFile> 
#include <osgViewer/Viewer>
#include <osg/PositionAttitudeTransform>
#include <osgGA/TrackballManipulator>

#define I_NON 0
#define I_TRS 1
#define I_ROT 2
#define I_MAG 3
#define BUF_SIZE 800

namespace mars {

  namespace plugins {
    namespace ptpSoil {
		
	  using namespace mars::utils;
      using namespace mars::interfaces;
      using namespace std;

      // inherit from MarsPluginTemplateGUI for extending the gui
      class PtpSoil: public mars::interfaces::MarsPluginTemplate,
        public mars::data_broker::ReceiverInterface,
        // for gui
        // public mars::main_gui::MenuInterface,
        public mars::cfg_manager::CFGClient {

      public:
        PtpSoil(lib_manager::LibManager *theManager);
        ~PtpSoil();

        // LibInterface methods
        int getLibVersion() const
        { return 1; }
        const std::string getLibName() const
        { return std::string("ptpSoil"); }
        CREATE_MODULE_INFO();

        // MarsPlugin methods
        void init();
        void reset();
        void update(mars::interfaces::sReal time_ms);
        

void drawCell(Vector* cellPos, Vector* cellSize);
void drawSoil(PTPInterface* soil);

        // DataBrokerReceiver methods
        virtual void receiveData(const data_broker::DataInfo &info,
                                 const data_broker::DataPackage &package,
                                 int callbackParam);
        // CFGClient methods
        virtual void cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property);

        // MenuInterface methods
        //void menuAction(int action, bool checked = false);

        // PtpSoil methods
        
        sReal disp_time;
        sReal record_time;
        
     osg::ref_ptr<osg::Geode> geode_pt;

vFOOT foot[6];
vFOOT footLast[6];
Vector forceR[6];
Vector last_outF[6];
Vector last_pos[6];

int step;
double step_size;
double msec;
double vel_set;
FILE *out;
//FILE *out_cell;

double tempX;


float q,w,e;

char filename[500];
char filename_cell[153];

PTPInterface* soil[6];
bool readyNextPlane;
bool nextPlane;

        list< PTPInterface > ptplist;

      private:
        cfg_manager::cfgPropertyStruct example;
       
          std::string configurationPath;

      }; // end of class definition PtpSoil

    } // end of namespace ptpSoil
  } // end of namespace plugins
} // end of namespace mars

#endif // MARS_PLUGINS_PTPSOIL_H
