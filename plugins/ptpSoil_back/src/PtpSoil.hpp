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
#include "PTPSystem.hpp"
#include "KalmanFilter.hpp"

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
    
	typedef struct EXPDATA{
		int msec; 
		double Joint0; 
		double Joint1; 
		double Joint2; 
		double Joint3; 
		double Joint4; 
		double Joint5; 
		double Joint6; 
		double 	Fx; 
		double Fy; 
		double	Fz; 
		double Tx; 
		double Ty; 
		double Tz; 
		int FTStatus; 
		int RTDSeq; 
		int FTSeq; 
		int Marker; 
		int currentPointID; 
		double X; 
		double Y; 
		double Z; 
		double Rx; 
		double Ry; 
		double Rz;
	}EXPDATA;
	
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
        
        void setSoil();
        void configSim();
        int ID;

void genMovement();
 void showptp();

        // DataBrokerReceiver methods
        virtual void receiveData(const data_broker::DataInfo &info,
                                 const data_broker::DataPackage &package,
                                 int callbackParam);
        // CFGClient methods
        virtual void cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property);

        // MenuInterface methods
        //void menuAction(int action, bool checked = false);

        // PtpSoil methods
        
        int Value;   // which foot?
        sReal disp_time;
        
 //          osgViewer::Viewer viewer;
   //osg::Group* root;
   //osg::Geode* geode;
   //osg::Geometry* geometry;
   //osg::Vec3Array* vertices;
   //osg::Vec4Array* colors;
   
     //osg::ref_ptr<osg::Group> root;
     osg::ref_ptr<osg::Geode> geode_pt;
     //osg::ref_ptr<osg::Geometry> geom;
  
     //osg::ref_ptr<osg::Vec3Array> vertices;
     //osg::ref_ptr<osg::Vec4Array> colors;
         //osg::ref_ptr<osg::DrawArrays> drawArrays;
   ////          osg::ref_ptr<osg::MatrixTransform> scaleTransform;

int Projection;
int Object;
	int _InputState;
int t_J, t_K, t_debug;

#ifdef SPHERE_EXPERIMENT2
int t_p, t_c, t_f, t_b, t_a, t_x, t_d, t_t, t_s, t_j, t_k, t_m, t_v;
#endif
#ifndef SPHERE_EXPERIMENT2
int t_p, t_c, t_f, t_b, t_a, t_x, t_d, t_t, t_s, t_j, t_k, t_m, t_v;
#endif

int v_x,v_y,v_z;
FOOT foot[6];
FOOT footLast[6];
vecD forceR[6];
vecD forceT[6];

int step;
double step_size;
double msec;
double vel_set;
FILE *out;
//FILE *out_cell;


float q,w,e;
EXPDATA edata[2500];     // loading lookup all file data. EXPDATA is defined in PTPopenGL.hpp


char filename[150];
char filename_cell[153];

KalmanFilter* kf;
PTPSystem* ptpSys;

      private:
        cfg_manager::cfgPropertyStruct example;
       
          std::string configurationPath;

      }; // end of class definition PtpSoil

    } // end of namespace ptpSoil
  } // end of namespace plugins
} // end of namespace mars

#endif // MARS_PLUGINS_PTPSOIL_H
