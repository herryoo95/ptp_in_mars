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
 * \file BoxMls.h
 * \author Yong-Ho (yong-ho.yoo@dfki.de)
 * \brief mls
 *
 * Version 0.1
 */

#ifndef MARS_PLUGINS_BOXMLS_H
#define MARS_PLUGINS_BOXMLS_H

#ifdef _PRINT_HEADER_
  #warning "BoxMls.h"
#endif

// set define if you want to extend the gui
//#define PLUGIN_WITH_MARS_GUI
#include <mars/interfaces/sim/MarsPluginTemplate.h>
#include <mars/interfaces/MARSDefs.h>
#include <mars/data_broker/ReceiverInterface.h>
#include <mars/cfg_manager/CFGManagerInterface.h>

#include <string>

namespace mars {

  namespace plugins {
    namespace BoxMls {

      // inherit from MarsPluginTemplateGUI for extending the gui
      class BoxMls: public mars::interfaces::MarsPluginTemplate,
        public mars::data_broker::ReceiverInterface,
        // for gui
        // public mars::main_gui::MenuInterface,
        public mars::cfg_manager::CFGClient {

      public:
        BoxMls(lib_manager::LibManager *theManager);
        ~BoxMls();

        // LibInterface methods
        int getLibVersion() const
        { return 1; }
        const std::string getLibName() const
        { return std::string("BoxMls"); }
        CREATE_MODULE_INFO();

        // MarsPlugin methods
        void init();
        void reset();
        void update(mars::interfaces::sReal time_ms);

        // DataBrokerReceiver methods
        virtual void receiveData(const data_broker::DataInfo &info,
                                 const data_broker::DataPackage &package,
                                 int callbackParam);
        // CFGClient methods
        virtual void cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property);

        // MenuInterface methods
        //void menuAction(int action, bool checked = false);

        // BoxMls methods

      private:
        cfg_manager::cfgPropertyStruct example;
        
            	FILE *fp; 
		double data[3];
		char buf[500];   		//to test mls to height map
		double mls_mean[25000]; //to test mls to height map	
        

      }; // end of class definition BoxMls

    } // end of namespace BoxMls
  } // end of namespace plugins
} // end of namespace mars

#endif // MARS_PLUGINS_BOXMLS_H
