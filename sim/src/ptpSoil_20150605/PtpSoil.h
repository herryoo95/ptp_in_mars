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

#ifndef MARS_PTPSOIL_H
#define MARS_PTPSOIL_H

#ifdef _PRINT_HEADER_
  #warning "PtpSoil.h"
#endif

#include <string>
#include <mars/utils/Vector.h>
#include "PTPSystem.h"

#define I_NON 0
#define I_TRS 1
#define I_ROT 2
#define I_MAG 3
#define BUF_SIZE 800

namespace mars {
  namespace sim {
   
      class PtpSoil {
		      //class NodePhysics : public interfaces::NodeInterface {

      public:
        PtpSoil();
        ~PtpSoil();

        virtual void soil_init();
        virtual void dCollideSoil(utils::Vector *obj_pos, utils::Vector *obj_vel);//vecD* obj_pos);
       
       	FOOT foot[6];
		FOOT footLast[6];
		vecD forceR[6];
		vecD forceT[6];
		Vector last_pos[6];
		int test;

		PTPSystem* ptpSys;

      private:

      }; // end of class definition PtpSoil

  } // end of namespace sim
} // end of namespace mars

#endif // MARS_PTPSOIL_H
