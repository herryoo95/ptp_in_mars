/*
 *  Copyright 2011, 2012, DFKI GmbH Robotics Innovation Center
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
 * \file WorldPhysics.cpp
 * \author Malte Roemmermann
 * \brief "WorldPhysics" includes the methods to handle the physically world.
 *
 * Conditions:
 *           - The state of the private variable world_init is
 *             aquivalent to the initialization state of the world,
 *             space and the contactgroup variables
 *
 * ToDo:
 *               - get and set the standard physical parameters
 *               - get and set the special ODE parameters via
 *                 a generic component through the Simulator class
 *               - handle sensor data 
 *
 */

#include "WorldPhysics.h"
#include "NodePhysics.h"


#include <mars/utils/MutexLocker.h>
#include <mars/interfaces/graphics/draw_structs.h>
#include <mars/interfaces/graphics/GraphicsManagerInterface.h>
#include <mars/interfaces/sim/SimulatorInterface.h>
#include <mars/interfaces/Logging.hpp>

#include "../../../plugins/PTPGraphics/src/PTPGraphics.h"

namespace mars {
  namespace sim {

    using namespace utils;
    using namespace interfaces;

    PhysicsError WorldPhysics::error = PHYSICS_NO_ERROR;

    void myMessageFunction(int errnum, const char *msg, va_list ap) {
      CPP_UNUSED(errnum);
      LOG_INFO(msg, ap);
    }

    void myDebugFunction(int errnum, const char *msg, va_list ap) {
      CPP_UNUSED(errnum);
      LOG_DEBUG(msg, ap);
      WorldPhysics::error = PHYSICS_DEBUG;
    }

    void myErrorFunction(int errnum, const char *msg, va_list ap) {
      CPP_UNUSED(errnum);
      LOG_ERROR(msg, ap);
      WorldPhysics::error = PHYSICS_ERROR;
    }

    /**
     *  \brief The constructor for the physical world.
     *
     *  pre:
     *      - none
     *
     *  post:
     *      - all private variables should be initialized correct
     *        should correct be spezified?
     *      - world, space, contactgroup and world_init to false (0)
     */
    WorldPhysics::WorldPhysics(ControlCenter *control) {
      this->control = control;
      draw_contact_points = 0;
      fast_step = 0;
      world_cfm = 1e-10;
      world_erp = 0.1;
      world_gravity = Vector(0.0, 0.0, -9.81);
      ground_friction = 20;
      ground_cfm = 0.00000001;
      ground_erp = 0.1;
      world = 0;
      space = 0;
      contactgroup = 0;
      world_init = 0;
      num_contacts = 0;
      create_contacts = 1;
      log_contacts = 0;
      
      for(int i=0;i<6;i++) first_colliding[i] = true;
      for(int i=0;i<6;i++) heightMap_collision[i] = false;    
         
      collideCount = 0;
      prev_collideCount = 0;
      foot_radius = 0.025958;
      onCalCollision = false;

      // the step size in seconds
      step_size = 0.01;
      // dInitODE is relevant for using trimesh objects as correct as
      // possible in the ode implementation
      MutexLocker locker(&iMutex);
#ifdef ODE11
      // for ode-0.11
      dInitODE2(0);
      dAllocateODEDataForThread(dAllocateMaskAll);
#else
      dInitODE();
#endif
      dSetErrorHandler (myErrorFunction);
      dSetDebugHandler (myDebugFunction);
      dSetMessageHandler (myMessageFunction);
    }

    /**
     * \brief Close ODE environment
     *
     * pre:
     *     - none
     *
     * post:
     *     - everthing that was created should be destroyed
     *
     */
    WorldPhysics::~WorldPhysics(void) {
      // free the ode objects
      freeTheWorld();
      // and close the ODE ...
      MutexLocker locker(&iMutex);
      dCloseODE();
    }

    /**
     *  \brief This function initializes the ode world.
     *
     * pre:
     *     - world_init = false
     *
     * post:
     *     - world, space and contactgroup should be created
     *     - the ODE world parameters should be set
     *     - at the end world_init have to become true
     */
    void WorldPhysics::initTheWorld(void) {
      MutexLocker locker(&iMutex);
  
      // if world_init = true debug something
      if (!world_init) {
        //LOG_DEBUG("init physics world");
        world = dWorldCreate();
        space = dHashSpaceCreate(0);
        contactgroup = dJointGroupCreate(0);

        old_gravity = world_gravity;
        old_cfm = world_cfm;
        old_erp = world_erp;

        dWorldSetGravity(world, world_gravity.x(), world_gravity.y(), world_gravity.z()); 
        dWorldSetCFM(world, (dReal)world_cfm);
        dWorldSetERP (world, (dReal)world_erp);

        dWorldSetAutoDisableFlag (world,0);
        // if usefull for some tests a ground can be created here
        plane = 0; //dCreatePlane (space,0,0,1,0);
        world_init = 1;
        drawStruct draw;
        draw.ptr_draw = (DrawInterface*)this;
        if(control->graphics)
          control->graphics->addDrawItems(&draw);
      }
    }

    /**
     * \brief This functions destroys the ode world.
     *
     * pre:
     *     - world_init = true
     *
     * post:
     *     - world, space and contactgroup have to be destroyed here
     *     - afte that, world_init have to become false
     */
    void WorldPhysics::freeTheWorld(void) {
      MutexLocker locker(&iMutex);
      if(world_init) {
        //LOG_DEBUG("free physics world");
        dJointGroupDestroy(contactgroup);
        dSpaceDestroy(space);
        dWorldDestroy(world);
        world_init = 0;
      }
      // else debug something
    }

    /**
     * \brief Returns if a world exists.
     *
     * pre:
     *     - none
     *
     * post:
     *     - return state of world_init
     */
    bool WorldPhysics::existsWorld(void) const {
      return world_init;
    }

    /**
     * \brief This function handles the calculation of a step in the world.
     *
     * pre:
     *     - world_init = true
     *     - step_size > 0
     *
     * post:
     *     - handled the collisions
     *     - step the world for step_size seconds
     *     - the contactgroup should be empty
     */
    void WorldPhysics::stepTheWorld(void) {
      MutexLocker locker(&iMutex);
      std::vector<dJointFeedback*>::iterator iter;
      geom_data* data;
      int i;

      // if world_init = false or step_size <= 0 debug something
      if(world_init && step_size > 0) {
        if(old_gravity != world_gravity) {
          old_gravity = world_gravity;
          dWorldSetGravity(world, world_gravity.x(),
                           world_gravity.y(), world_gravity.z());
        }

        if(old_cfm != world_cfm) {
          old_cfm = world_cfm;
          dWorldSetCFM(world, (dReal)world_cfm);
        }

        if(old_erp != world_erp) {
          old_erp = world_erp;
          dWorldSetERP(world, (dReal)world_erp);
        }

        /// first clear the collision counters of all geoms
        for(i=0; i<dSpaceGetNumGeoms(space); i++) {
          data = (geom_data*)dGeomGetData(dSpaceGetGeom(space, i));
          data->num_ground_collisions = 0;
          data->contact_ids.clear();
          data->contact_points.clear();
          data->ground_feedbacks.clear();
        }
        for(iter = contact_feedback_list.begin();
            iter != contact_feedback_list.end(); iter++) {
          free((*iter));
        }
        contact_feedback_list.clear();
        draw_intern.clear();
        /// then we have to clear the contacts
        dJointGroupEmpty(contactgroup);
        /// first check for collisions
        num_contacts = log_contacts = 0;
        create_contacts = 1;
        dSpaceCollide(space,this, &WorldPhysics::callbackForward);
                
        drawLock.lock();
        draw_extern.swap(draw_intern);
        drawLock.unlock();

        /// then calculate the next state for a time of step_size seconds
        try {
          if(fast_step) dWorldQuickStep(world, step_size);
          else dWorldStep(world, step_size);
        } catch (...) {
          control->sim->handleError(PHYSICS_UNKNOWN);
        }
	if(WorldPhysics::error) {
          control->sim->handleError(WorldPhysics::error);
          WorldPhysics::error = PHYSICS_NO_ERROR;
	}
      }
    }

    /**
     * \brief Returns the ode ID of the world object.
     *
     * pre:
     *     - none
     *
     * post:
     *     - world ID returned
     */
    dWorldID WorldPhysics::getWorld(void) const {
      return world;
    }

    /**
     * \brief Returns the ode ID of the main space object.
     *
     * pre:
     *     - none
     *
     * post:
     *     - space ID returned
     */
    dSpaceID WorldPhysics::getSpace(void) const {
      return space;
    }

    /**
     * \brief Sets the body pointer param to the body for the comp_group_id
     *
     * The functions sets the body pointer to the body ID that
     * represents the composite object. If no body is aviable
     * for the composite group a body will be created.
     * The functions return if an body allready exists or if one
     * had been created.
     *
     * Careful with this function, bad implementation. This function should be
     * only called if a new geom will be conected to the given body.
     *
     * pre:
     *     - comp_group > 0
     *
     * post:
     *     - body pointer should be set to a regular body
     *       that is in the vector comp_body_list
     *     - retruned true if a body was created, otherwise retruned false
     */
    bool WorldPhysics::getCompositeBody(int comp_group, dBodyID* body,
                                        NodePhysics *node) {
      body_nbr_tupel tmp_tupel;

      // if comp_group is bad, debug something
      if(comp_group > 0) {
        std::vector<body_nbr_tupel>::iterator iter;
        for( iter = comp_body_list.begin(); iter != comp_body_list.end(); iter++ ) {
          if((*iter).comp_group == comp_group) {
            (*iter).connected_geoms++;
            *body = (*iter).body;
            (*iter).comp_nodes.push_back(node);
            return 0;
          }
        }
        tmp_tupel.body = *body = dBodyCreate(world);
        tmp_tupel.comp_group = comp_group;
        tmp_tupel.connected_geoms = 1;
        tmp_tupel.comp_nodes.push_back(node);
        comp_body_list.push_back(tmp_tupel);
        return 1;
      }
      return 0;
    }

    /**
     * \brief Destroyes a body from a node:
     *
     * This function checks if not more than one geom is connected to the
     * body and destroyes the body in that case. In the other case the counter
     * of the connected geoms is decreased.
     *
     * pre:
     *     - the body exists in the physical world
     *
     * post:
     *     - if more than one geoms are connected to the body, decrease the
     *       counter of connected geoms
     *     - otherwise, if only one geom is connected to the body, destroy the body
     */
    void WorldPhysics::destroyBody(dBodyID theBody, NodePhysics* node) {
      std::vector<body_nbr_tupel>::iterator iter;
      std::vector<NodePhysics*>::iterator jter;

      for(iter = comp_body_list.begin(); iter != comp_body_list.end(); iter++) {
        if((*iter).body == theBody) {
          if((*iter).connected_geoms > 1) {
            (*iter).connected_geoms--;
            for(jter = (*iter).comp_nodes.begin();
                jter != (*iter).comp_nodes.end(); jter++) {
              if((*jter) == node) {
                (*iter).comp_nodes.erase(jter);
                break;
              }
            }
            resetCompositeMass(theBody);
            return;
          }
          else {
            dBodyDestroy(theBody);
            comp_body_list.erase(iter);
            return;
          }
        }
      }
      // if we get here in the code, the body is not in the list and can
      // be removed from the world
      dBodyDestroy(theBody);
    }

    /**
     * \brief Returns the stepsize for calculating a world step
     *
     * pre:
     *     - none
     *
     * post:
     *     - the step_size value should be returned
     */
    dReal WorldPhysics::getWorldStep(void) {
      return step_size;
    }


//////
//void WorldPhysics::drawSoil(ControlCenter *control, PTPInterface* soil){

	// if(prev_drawing) control->graphics->removeOSGNode(geode_pt.get()); 
		  
     //osg::ref_ptr<osg::Geode> geode (new osg::Geode());
     //geode_pt = geode;
     
   ////  osg::ref_ptr<osg::Geometry> particleGeom (new osg::Geometry());
   ////  osg::ref_ptr<osg::Vec3Array> particleVertices (new osg::Vec3Array());
   ////  osg::ref_ptr<osg::Vec4Array> particleColors (new osg::Vec4Array());
     
     
        //osg::Group* root = new osg::Group();
   //osg::Geode* geode = new osg::Geode();
   //osg::Geometry* geometry = new osg::Geometry();
   //osg::Vec3Array* vertices = new osg::Vec3Array;
   //osg::Vec4Array* colors = new osg::Vec4Array;
   
     //osg::ref_ptr<osg::Group> root (new osg::Group());
     //osg::ref_ptr<osg::Geode> geode (new osg::Geode());
     //osg::ref_ptr<osg::Geometry> geometry (new osg::Geometry());
  
     //osg::ref_ptr<osg::Vec3Array> vertices (new osg::Vec3Array());
     //osg::ref_ptr<osg::Vec4Array> colors (new osg::Vec4Array());
     
     
     
     //Vector ptp_pos;
  
       //for(int i = 0;i<soil->ptp.n.x;i++){
		//for(int j = 0;j<soil->ptp.n.y;j++){
		  //for(int k = 0;k<soil->ptp.n.z;k++){
			  
       	//////Only Collision Particle
		////if(soil->particle[i][j][k].isCollision){
		  ////ptp_pos.x() = soil->particle[i][j][k].loc.x;
		  ////ptp_pos.y() = soil->particle[i][j][k].loc.y;
		  ////ptp_pos.z() = soil->particle[i][j][k].loc.z; 			
	   
	   ////vertices->push_back (osg::Vec3 (ptp_pos.x(), ptp_pos.y(), ptp_pos.z()));
       ////colors->push_back (osg::Vec4f (1.0f,0.0f, 0.0f,1.0f));
		////}else{			  
		  //ptp_pos.x() = soil->particle[i][j][k].loc.x*MM2M;
		  //ptp_pos.y() = soil->particle[i][j][k].loc.y*MM2M;
		  //ptp_pos.z() = soil->particle[i][j][k].loc.z*MM2M; 
  
       //particleVertices->push_back (osg::Vec3 (ptp_pos.x(), ptp_pos.y(), ptp_pos.z()));
       //particleColors->push_back (osg::Vec4f (1.0f,1.0f, 1.0f,1.0f));
		////}
	     //if(soil->cell[i][j][k].CollisionParticleNum){
			//Vector cellPos,cellSize; 
			//cellPos.x() = soil->cell[i][j][k].start.x*MM2M;
			//cellPos.y() = soil->cell[i][j][k].start.y*MM2M;
			//cellPos.z() = soil->cell[i][j][k].start.z*MM2M;
			 
			//cellSize.x() = (soil->cell[i][j][k].start.x-soil->cell[i][j][k].end.x)*MM2M/MAG;
			//cellSize.y() = (soil->cell[i][j][k].start.y-soil->cell[i][j][k].end.y)*MM2M/MAG;
			//cellSize.z() = (soil->cell[i][j][k].start.z-soil->cell[i][j][k].end.z)*MM2M/MAG;

          ////  drawCell(&cellPos, &cellSize);
		//}
       
		  //}
		//}
	  //}
	  

	  
    //particleGeom->setVertexArray(particleVertices.get());
    //particleGeom->setColorArray(particleColors.get());

	//particleGeom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    //particleGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,particleVertices->size()));
    //geode_pt->addDrawable(particleGeom.get());


    //control->graphics->addOSGNode(geode.get());  
    //prev_drawing = true;

//}
///


void WorldPhysics::buildNewSoil(const int foot_id, const Vector obj_pos, const Vector obj_vel, dBodyID body){
	    //dBodyID body1 = body;
		printf("############footID = %d %d new soil\n",foot_id, first_colliding[foot_id]); 		
	if(first_colliding[foot_id] && obj_pos.x() > 0.6) {
		printf("################################################## footID = %d new soil\n",foot_id); 		
		first_colliding[foot_id] = false;
		heightMap_collision[foot_id] = true;
		
		VectorN num;  //number of particles in a soil
		Vector start;
		Vector end;
		num.x() = 50;
		num.y() = 50;
		num.z() = 20;
		double soildepth = foot_radius*4;
		start.x() = (obj_pos.x() - foot_radius*4)*M2MM;
		start.y() = (obj_pos.y() - foot_radius*4)*M2MM;
		start.z() = (obj_pos.z() - foot_radius*2 - soildepth)*M2MM;//-310;
		end.x() = (obj_pos.x() + foot_radius*4)*M2MM;
		end.y() = (obj_pos.y() + foot_radius*4)*M2MM;
		end.z() = (obj_pos.z() - foot_radius*2)*M2MM;//-210;
		end_soil[foot_id] = end.z()*MM2M;
		PTPInterface* soil = new PTPInterface(num.x(),num.y(),num.z());
		soil->buildPTP(&num,&start,&end);
		soil->vfoot.type = SPHERE;
		control->nodes->pushPTPlist(foot_id,soil);
		//delete soil;
	
	}
	if(heightMap_collision[foot_id] && obj_pos.x() > 0.6){
		control->nodes->setFootPosition(foot_id, obj_pos);
		control->nodes->setFootVelocity(foot_id, obj_vel);	    
		control->nodes->setFootRadius(foot_id, foot_radius);	    
		 if(!control->nodes->collideOnSoil(foot_id)){printf("ERRRROR\n");}
		 else {
			 Vector outF;
			 
			control->nodes->getSoilContactForce(foot_id, outF);	
			outF.z() = -outF.z();
			//outF.x() = 0;
			//outF.y() = 0;			
			
			Vector maxForce(100.f, 100.f, 230.f), minForce(-100.f,-100.f,-230.f);
			if(ABS(outF.z()) <= maxForce.z()) outF.z() = outF.z();
			else if(outF.z() > maxForce.z()) outF.z() = maxForce.z();
			else if(outF.z() < minForce.z()) outF.z() = minForce.z();	
			
			if(ABS(outF.x()) <= maxForce.x()) outF.x() = outF.x();
			else if(outF.x() > maxForce.x()) outF.x() = maxForce.x();
			else if(outF.x() < minForce.x()) outF.x() = minForce.x();
			
			if(ABS(outF.y()) <= maxForce.y()) outF.y() = outF.y();
			else if(outF.y() > maxForce.y()) outF.y() = maxForce.y();
			else if(outF.y() < minForce.y()) outF.y() = minForce.y();	
										 			
			dBodyAddForce(body,(dReal)outF.x(), (dReal)outF.y(), (dReal)outF.z());	
		}
	 }	
	
} 

    /**
     * \brief In this function the collision handling from ode is performed.
     *
     * pre:
     *     - world_init = true
     *     - o1 and o2 are regular geoms
     *
     * post:
     *     - if o1 or o2 was a Space, called SpaceCollide and exit
     *     - otherwise tested if the geoms collide and created a contact
     *       joint if so.
     *
     * A lot of the code is uncommented in this function. This
     * code maybe used later to handle sensors or other special cases
     * in the simulation.
     */
    void WorldPhysics::nearCallback (dGeomID o1, dGeomID o2) {
      int i;
      int numc;
      //up to MAX_CONTACTS contact per Box-box
      //dContact contact[MAX_CONTACTS];
      dVector3 v1, v;
      //dMatrix3 R;
      dReal dot;
  
      if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {
        /// test if a space is colliding with something
        dSpaceCollide2(o1,o2,this,& WorldPhysics::callbackForward);
        return;
      }
  
      /// exit without doing anything if the two bodies are connected by a joint 
      dBodyID b1=dGeomGetBody(o1);
      dBodyID b2=dGeomGetBody(o2);

      geom_data* geom_data1 = (geom_data*)dGeomGetData(o1);
      geom_data* geom_data2 = (geom_data*)dGeomGetData(o2);
      
	//if(first_colliding[0] == false && geom_data1->id == 34){   	// i = foot_id; PTP exists
		//Vector footpos = control->nodes->getPosition(34);
		//if((footpos.z()-0.05f) > end_soil[0]) {
		//first_colliding[0] = true;
		//heightMap_collision[0] = false;			
		//printf("+++++++++++++++++deleted+++++++++++++++++++foot %d %d (%f %f)..over..over\n", 
			//0, first_colliding[0], footpos.z()-0.05, end_soil[0]);		
		//control->nodes->popPTPlist(0);

		//}
		////printf("%lu.. first_colliding....reset (%f %f)\n",geom_data1->id, footpos.z()-0.05, end_soil[0]);
	//}
	
		Vector footpos;
				switch (geom_data1->id){							
					case 34:
							if(first_colliding[0] == false){  
							footpos = control->nodes->getPosition(34);
							if((footpos.z()+THRESHOLD) > end_soil[0]) {
		printf("+++++++++++++++++deleted+++++++++++++++++++foot %d %d (%f %f)..over..over\n", 
			0, first_colliding[0], footpos.z()+THRESHOLD, end_soil[0]);		
							first_colliding[0] = true;
							heightMap_collision[0] = false;
							control->nodes->popPTPlist(0);
		printf("%lu.. first_colliding....reset (%f %f)\n",
		geom_data1->id, footpos.z()+THRESHOLD, end_soil[0]);
							}}
							break;
					case 48: 
							if(first_colliding[1] == false){  
							footpos = control->nodes->getPosition(48);
							if((footpos.z()+THRESHOLD) > end_soil[1]) {
		printf("+++++++++++++++++deleted+++++++++++++++++++foot %d %d (%f %f)..over..over\n", 
			1, first_colliding[1], footpos.z()+THRESHOLD, end_soil[1]);		
							first_colliding[1] = true;
							heightMap_collision[1] = false;
							control->nodes->popPTPlist(1);
		printf("%lu.. first_colliding....reset (%f %f)\n",
		geom_data1->id, footpos.z()+THRESHOLD, end_soil[1]);
							}}
							break;
					case 61: 
							if(first_colliding[2] == false){  
							footpos = control->nodes->getPosition(61);
							if((footpos.z()+THRESHOLD) > end_soil[2]) {
		printf("+++++++++++++++++deleted+++++++++++++++++++foot %d %d (%f %f)..over..over\n", 
			2, first_colliding[2], footpos.z()+THRESHOLD, end_soil[2]);		
							first_colliding[2] = true;
							heightMap_collision[2] = false;
							control->nodes->popPTPlist(2);
		printf("%lu.. first_colliding....reset (%f %f)\n",
		geom_data1->id, footpos.z()+THRESHOLD, end_soil[2]);
							}}
							break;
					case 73:  
							if(first_colliding[3] == false){  
							footpos = control->nodes->getPosition(73);
							if((footpos.z()+THRESHOLD) > end_soil[3]) {
		printf("+++++++++++++++++deleted+++++++++++++++++++foot %d %d (%f %f)..over..over\n", 
			3, first_colliding[3], footpos.z()+THRESHOLD, end_soil[3]);		
							first_colliding[3] = true;
							heightMap_collision[3] = false;
							control->nodes->popPTPlist(3);
		printf("%lu.. first_colliding....reset (%f %f)\n",
		geom_data1->id, footpos.z()+THRESHOLD, end_soil[3]);
							}}
							break;
					case 87:  
							if(first_colliding[4] == false){  
							footpos = control->nodes->getPosition(87);
							if((footpos.z()+THRESHOLD) > end_soil[4]) {
		printf("+++++++++++++++++deleted+++++++++++++++++++foot %d %d (%f %f)..over..over\n", 
			4, first_colliding[4], footpos.z()+THRESHOLD, end_soil[4]);		
							first_colliding[4] = true;
							heightMap_collision[4] = false;
							control->nodes->popPTPlist(4);
		printf("%lu.. first_colliding....reset (%f %f)\n",
		geom_data1->id, footpos.z()+THRESHOLD, end_soil[4]);
							}}
							break;
					case 99:  
							if(first_colliding[5] == false){  
							footpos = control->nodes->getPosition(99);
							if((footpos.z()+THRESHOLD) > end_soil[5]) {
		printf("+++++++++++++++++deleted+++++++++++++++++++foot %d %d (%f %f)..over..over\n", 
			5, first_colliding[5], footpos.z()+THRESHOLD, end_soil[5]);		
							first_colliding[5] = true;
							heightMap_collision[5] = false;
							control->nodes->popPTPlist(5);
		printf("%lu.. first_colliding....reset (%f %f)\n",
		geom_data1->id, footpos.z()+THRESHOLD, end_soil[5]);
							}}
							break;
					default: break;	
					
				}
		

	  if(geom_data1->ray_sensor) {
        dContact contact;
        if(geom_data1->parent_geom == o2) {
          return;
        }
        
        if(geom_data1->parent_body == dGeomGetBody(o2)) {
          return;
        }
        
        numc = dCollide(o2, o1, 1|CONTACTS_UNIMPORTANT, &(contact.geom), sizeof(dContact));
        if(numc) {
          if(contact.geom.depth < geom_data1->value)
            geom_data1->value = contact.geom.depth;
          ray_collision = 1;
        }
        return;
      }
      else if(geom_data2->ray_sensor) {
        dContact contact;
        if(geom_data2->parent_geom == o1) {
          return;
        }
        if(geom_data2->parent_body == dGeomGetBody(o1)) {
          return;
        }
        numc = dCollide(o2, o1, 1|CONTACTS_UNIMPORTANT, &(contact.geom), sizeof(dContact));
        if(numc) {
          if(contact.geom.depth < geom_data2->value)
            geom_data2->value = contact.geom.depth;
          ray_collision = 1;
        }
        return;
      }
      
      if(b1 && b2 && dAreConnectedExcluding(b1,b2,dJointTypeContact))
        return;

      if(!b1 && !b2 && !geom_data1->ray_sensor && !geom_data2->ray_sensor) return;

      int maxNumContacts = 0;
      if(geom_data1->c_params.max_num_contacts <
         geom_data2->c_params.max_num_contacts) {
        maxNumContacts = geom_data1->c_params.max_num_contacts;
      }
      else {
        maxNumContacts = geom_data2->c_params.max_num_contacts;
      }
      dContact *contact = new dContact[maxNumContacts];


      //for granular test
      //if( (plane != o2) && (plane !=o1)) return ;
  
  
      /*
     /// we use the geomData to handle some special cases
     void* geom_data1 = dGeomGetData(o1);
     void* geom_data2 = dGeomGetData(o2);

     /// one case is, that we don't wont to handle a collision between some special
     /// geoms beweet each other and the ground
     if((geom_data1 && ((robot_geom*)geom_data1)->type & 16)) {
     if(plane == o2) return;
     if((geom_data2 && ((robot_geom*)geom_data2)->type & 16)) return;
     }
     else if((geom_data2 && ((robot_geom*)geom_data2)->type & 16) && (plane == o1)) return;
  
     /// an other case is a ray geom that we use simulate ray sensors
     /// this geom has to be handled in a different way
     if((geom_data1 && ((robot_geom*)geom_data1)->type & 8) ||
     (geom_data2 && ((robot_geom*)geom_data2)->type & 8)) {    
     int n;
     const int N = MAX_CONTACTS;
     dContactGeom contact[N];

     n = dCollide (o2,o1,N,contact,sizeof(dContactGeom));
     if (n > 0) {
     //const dReal ss[3] = {1,0.01,0.01};
     for (i=0; i<n; i++) {
     contact[i].pos[2] += Z_OFFSET;
     if(contact[i].depth > 0.01){
     if(geom_data1 && ((robot_geom*)geom_data1)->type & 8)
     ((robot_geom*)geom_data1)->i_length = contact[0].depth;
     if(geom_data2 && ((robot_geom*)geom_data2)->type & 8)
     ((robot_geom*)geom_data2)->i_length = contact[0].depth;
     }
     }
     }
     return;
     }
      */
  

  
      // frist we set the softness values:
      contact[0].surface.mode = dContactSoftERP | dContactSoftCFM;
      contact[0].surface.soft_cfm = (geom_data1->c_params.cfm +
                                     geom_data2->c_params.cfm)/2;
      contact[0].surface.soft_erp = (geom_data1->c_params.erp +
                                     geom_data2->c_params.erp)/2;
      // then check if one of the geoms want to use the pyramid approximation
      if(geom_data1->c_params.approx_pyramid ||
         geom_data2->c_params.approx_pyramid)
        contact[0].surface.mode |= dContactApprox1;
  
      // Then check the friction for both directions
      contact[0].surface.mu = (geom_data1->c_params.friction1 +
                               geom_data2->c_params.friction1)/2;
      contact[0].surface.mu2 = (geom_data1->c_params.friction2 +
                                geom_data2->c_params.friction2)/2;

      if(contact[0].surface.mu != contact[0].surface.mu2)
        contact[0].surface.mode |= dContactMu2;

      // check if we have to calculate friction direction1
      if(geom_data1->c_params.friction_direction1 ||
         geom_data2->c_params.friction_direction1) {
        // here the calculation becomes more complicated
        // maybe we should make some restrictions
        // so -> we only use friction motion in friction direction 1
        // the friction motion is only set if a local vector for friction
        // direction 1 is given
        // the steps for the calculation:
        // 1. rotate the local vectors to global coordinates
        // 2. scale the vectors to the length of the motion if given
        // 3. vector 3 =  vector 1 - vector 2
        // 4. get the length of vector 3
        // 5. set vector 3 as friction direction 1
        // 6. set motion 1 to the length
        contact[0].surface.mode |= dContactFDir1;
        if(!geom_data2->c_params.friction_direction1) {
          // get the orientation of the geom
          //dGeomGetQuaternion(o1, v);
          //dRfromQ(R, v);
          // copy the friction direction
          v1[0] = geom_data1->c_params.friction_direction1->x();
          v1[1] = geom_data1->c_params.friction_direction1->y();
          v1[2] = geom_data1->c_params.friction_direction1->z();
          // translate the friction direction to global coordinates
          // and set friction direction for contact
          //dMULTIPLY0_331(contact[0].fdir1, R, v1);
          contact[0].fdir1[0] = v1[0];
          contact[0].fdir1[1] = v1[1];
          contact[0].fdir1[2] = v1[2];
          if(geom_data1->c_params.motion1) {
            contact[0].surface.mode |= dContactMotion1;
            contact[0].surface.motion1 = geom_data1->c_params.motion1;
          }
        }
        else if(!geom_data1->c_params.friction_direction1) {
          // get the orientation of the geom
          //dGeomGetQuaternion(o2, v);
          //dRfromQ(R, v);
          // copy the friction direction
          v1[0] = geom_data2->c_params.friction_direction1->x();
          v1[1] = geom_data2->c_params.friction_direction1->y();
          v1[2] = geom_data2->c_params.friction_direction1->z();
          // translate the friction direction to global coordinates
          // and set friction direction for contact
          //dMULTIPLY0_331(contact[0].fdir1, R, v1);
          contact[0].fdir1[0] = v1[0];
          contact[0].fdir1[1] = v1[1];
          contact[0].fdir1[2] = v1[2];
          if(geom_data2->c_params.motion1) {
            contact[0].surface.mode |= dContactMotion1;
            contact[0].surface.motion1 = geom_data2->c_params.motion1;
          }
        }
        else {
          // the calculation steps as mentioned above
          fprintf(stderr, "the calculation for friction directen set for both nodes is not done yet.\n");
        }
      }

      // then check for fds
      if(geom_data1->c_params.fds1 || geom_data2->c_params.fds1) {
        contact[0].surface.mode |= dContactSlip1;
        contact[0].surface.slip1 = (geom_data1->c_params.fds1 +
                                    geom_data2->c_params.fds1);
      }
      if(geom_data1->c_params.fds2 || geom_data2->c_params.fds2) {
        contact[0].surface.mode |= dContactSlip2;
        contact[0].surface.slip2 = (geom_data1->c_params.fds2 +
                                    geom_data2->c_params.fds2);
      }
      if(geom_data1->c_params.bounce || geom_data2->c_params.bounce) {
        contact[0].surface.mode |= dContactBounce;
        contact[0].surface.bounce = (geom_data1->c_params.bounce +
                                     geom_data2->c_params.bounce);
        if(geom_data1->c_params.bounce_vel > geom_data2->c_params.bounce_vel)
          contact[0].surface.bounce_vel = geom_data1->c_params.bounce_vel;
        else
          contact[0].surface.bounce_vel = geom_data2->c_params.bounce_vel;      
      }

      for (i=1;i<maxNumContacts;i++){
        contact[i] = contact[0];
      }

      numc=dCollide(o1,o2, maxNumContacts, &contact[0].geom,sizeof(dContact));
    if(numc){ 
		
	if(geom_data2->height_field && geom_data2->id != 103) {
		//start to detect PTP collision
		dBodyID body1=dGeomGetBody(o1);
		int foot_id = 0;
		
		const dReal* body_pos = dBodyGetPosition(body1);
		const dReal* body_vel = dBodyGetLinearVel(body1);
		Vector obj_pos, obj_vel;
		
		obj_pos.x() = (sReal)body_pos[0];
		obj_pos.y() = (sReal)body_pos[1];
		obj_pos.z() = (sReal)body_pos[2];
		
		obj_vel.x() = (sReal)body_vel[0];
		obj_vel.y() = (sReal)body_vel[1];
		obj_vel.z() = (sReal)body_vel[2];


	//   printf("before switch... geom_data2->id = %lu\n", geom_data1->id);	
				
switch(geom_data1->id){
case 34:	foot_id = 0;
		buildNewSoil(foot_id, obj_pos, obj_vel, body1);
break;
case 48:		foot_id = 1;
	buildNewSoil(foot_id, obj_pos, obj_vel, body1);
break;
case 61:	foot_id = 2;
	if(first_colliding[foot_id] && obj_pos.x() > 0.6) {
	//	   printf("geom_data1->id = %lu\n", geom_data1->id);		
		first_colliding[foot_id] = false;
		heightMap_collision[foot_id] = true;
		
		VectorN num;  //number of particles in a soil
		Vector start;
		Vector end;
		num.x() = 50;
		num.y() = 50;
		num.z() = 20;
		double soildepth = foot_radius*4;
		start.x() = (obj_pos.x() - foot_radius*4)*M2MM;
		start.y() = (obj_pos.y() - foot_radius*4)*M2MM;
		start.z() = (obj_pos.z() - foot_radius - soildepth)*M2MM;//-310;
		end.x() = (obj_pos.x() + foot_radius*4)*M2MM;
		end.y() = (obj_pos.y() + foot_radius*4)*M2MM;
		end.z() = (obj_pos.z() - foot_radius)*M2MM;//-210;
		end_soil[foot_id] = end.z()*MM2M;
		PTPInterface* soil = new PTPInterface(num.x(),num.y(),num.z());
		soil->buildPTP(&num,&start,&end);
		soil->vfoot.type = SPHERE;
		control->nodes->pushPTPlist(foot_id,soil);
		//delete soil;
	} 
	
	if(heightMap_collision[foot_id] && obj_pos.x() > 0.6){
		
		//onCalCollision = true;
		
		control->nodes->setFootPosition(foot_id, obj_pos);
		control->nodes->setFootVelocity(foot_id, obj_vel);	    
		control->nodes->setFootRadius(foot_id, foot_radius);	    
		 if(!control->nodes->collideOnSoil(foot_id)){printf("ERRRROR\n");}
		 else {
			 Vector outF;
			 
			control->nodes->getSoilContactForce(foot_id, outF);	
			outF.z() = -outF.z();
			//outF.x() = 0;
			//outF.y() = 0;						
			
			double maxForce = 245.f;
			if(outF.z() > maxForce) outF.z() = maxForce;
		
			dBodyAddForce(body1,(dReal)outF.x(), (dReal)outF.y(), (dReal)outF.z());	
		}
		//onCalCollision = false;
	 }
break;
case 73:	foot_id = 3;
	if(first_colliding[foot_id] && obj_pos.x() > 0.6) {
	//	   printf("geom_data1->id = %lu\n", geom_data1->id);		
		first_colliding[foot_id] = false;
		heightMap_collision[foot_id] = true;
		
		VectorN num;  //number of particles in a soil
		Vector start;
		Vector end;
		num.x() = 50;
		num.y() = 50;
		num.z() = 20;
		double soildepth = foot_radius*4;
		start.x() = (obj_pos.x() - foot_radius*4)*M2MM;
		start.y() = (obj_pos.y() - foot_radius*4)*M2MM;
		start.z() = (obj_pos.z() - foot_radius*2 - soildepth)*M2MM;//-310;
		end.x() = (obj_pos.x() + foot_radius*4)*M2MM;
		end.y() = (obj_pos.y() + foot_radius*4)*M2MM;
		end.z() = (obj_pos.z() - foot_radius*2)*M2MM;//-210;
		end_soil[foot_id] = end.z()*MM2M;
		PTPInterface* soil = new PTPInterface(num.x(),num.y(),num.z());
		soil->buildPTP(&num,&start,&end);
		soil->vfoot.type = SPHERE;
		control->nodes->pushPTPlist(foot_id,soil);
		//delete soil;
	}
	
	if(heightMap_collision[foot_id] && obj_pos.x() > 0.6){
		
		//onCalCollision = true;
		
		control->nodes->setFootPosition(foot_id, obj_pos);
		control->nodes->setFootVelocity(foot_id, obj_vel);	    
		control->nodes->setFootRadius(foot_id, foot_radius);	    
		 if(!control->nodes->collideOnSoil(foot_id)){printf("ERRRROR\n");}
		 else {
			Vector outF;
			 
			control->nodes->getSoilContactForce(foot_id, outF);	
			outF.z() = -outF.z();
			//outF.x() = 0;
			//outF.y() = 0;						
			
			double maxForce = 245.f;
			if(outF.z() > maxForce) outF.z() = maxForce;
		
			dBodyAddForce(body1,(dReal)outF.x(), (dReal)outF.y(), (dReal)outF.z());	
		}
		//onCalCollision = false;
	 }
case 87:	foot_id = 4;
	if(first_colliding[foot_id] && obj_pos.x() > 0.6) {
	//	   printf("geom_data1->id = %lu\n", geom_data1->id);		
		first_colliding[foot_id] = false;
		heightMap_collision[foot_id] = true;

		VectorN num;  //number of particles in a soil
		Vector start;
		Vector end;
		num.x() = 50;
		num.y() = 50;
		num.z() = 20;
		double soildepth = foot_radius*4;
		start.x() = (obj_pos.x() - foot_radius*4)*M2MM;
		start.y() = (obj_pos.y() - foot_radius*4)*M2MM;
		start.z() = (obj_pos.z() - foot_radius*2 - soildepth)*M2MM;//-310;
		end.x() = (obj_pos.x() + foot_radius*4)*M2MM;
		end.y() = (obj_pos.y() + foot_radius*4)*M2MM;
		end.z() = (obj_pos.z() - foot_radius*2)*M2MM;//-210;
		end_soil[foot_id] = end.z()*MM2M;
		PTPInterface* soil = new PTPInterface(num.x(),num.y(),num.z());
		soil->buildPTP(&num,&start,&end);
		soil->vfoot.type = SPHERE;
		control->nodes->pushPTPlist(foot_id,soil);
		//delete soil;
	} 
	
	if(heightMap_collision[foot_id] && obj_pos.x() > 0.6){
		
		//onCalCollision = true;
		
		control->nodes->setFootPosition(foot_id, obj_pos);
		control->nodes->setFootVelocity(foot_id, obj_vel);	    
		control->nodes->setFootRadius(foot_id, foot_radius);	    
		 if(!control->nodes->collideOnSoil(foot_id)){printf("ERRRROR\n");}
		 else {
			 Vector outF;
			 
			control->nodes->getSoilContactForce(foot_id, outF);	
			outF.z() = -outF.z();
			//outF.x() = 0;
			//outF.y() = 0;						
			
			double maxForce = 245.f;
			if(outF.z() > maxForce) outF.z() = maxForce;
		
			dBodyAddForce(body1,(dReal)outF.x(), (dReal)outF.y(), (dReal)outF.z());	
		}
		//onCalCollision = false;
	 }
case 99:	foot_id = 5;		
	if(first_colliding[foot_id] && obj_pos.x() > 0.6) {
	//	   printf("geom_data1->id = %lu\n", geom_data1->id);		
		first_colliding[foot_id] = false;
		heightMap_collision[foot_id] = true;

		VectorN num;  //number of particles in a soil
		Vector start;
		Vector end;
		num.x() = 50;
		num.y() = 50;
		num.z() = 20;
		double soildepth = foot_radius*4;
		start.x() = (obj_pos.x() - foot_radius*4)*M2MM;
		start.y() = (obj_pos.y() - foot_radius*4)*M2MM;
		start.z() = (obj_pos.z() - foot_radius*2 - soildepth)*M2MM;//-310;
		end.x() = (obj_pos.x() + foot_radius*4)*M2MM;
		end.y() = (obj_pos.y() + foot_radius*4)*M2MM;
		end.z() = (obj_pos.z() - foot_radius*2)*M2MM;//-210;
		end_soil[foot_id] = end.z()*MM2M;
		PTPInterface* soil = new PTPInterface(num.x(),num.y(),num.z());
		soil->buildPTP(&num,&start,&end);
		soil->vfoot.type = SPHERE;
		control->nodes->pushPTPlist(foot_id,soil);
		//delete soil;
	}
	
	if(heightMap_collision[foot_id] && obj_pos.x() > 0.6){
		
		//onCalCollision = true;
		
		control->nodes->setFootPosition(foot_id, obj_pos);
		control->nodes->setFootVelocity(foot_id, obj_vel);	    
		control->nodes->setFootRadius(foot_id, foot_radius);	    
		 if(!control->nodes->collideOnSoil(foot_id)){printf("ERRRROR\n");}
		 else {
			 Vector outF;
			 
			control->nodes->getSoilContactForce(foot_id, outF);	
			outF.z() = -outF.z();
			//outF.x() = 0;
			//outF.y() = 0;						
			
			double maxForce = 245.f;
			if(outF.z() > maxForce) outF.z() = maxForce;
		
			dBodyAddForce(body1,(dReal)outF.x(), (dReal)outF.y(), (dReal)outF.z());	
		}
		//onCalCollision = false;
	 }
default: //printf("Did not find foot_id\n");
break;
}		
  } else { 	   // printf("normal...collided \n");		
   //   if(numc){ 
	  	  
        dJointFeedback *fb;
        draw_item item;
        Vector contact_point;

        num_contacts++;
        if(create_contacts) {
          fb = 0;
          item.id = 0;
          item.type = DRAW_LINE;
          item.draw_state = DRAW_STATE_CREATE;
          item.point_size = 10;
          item.myColor.r = 1;
          item.myColor.g = 0;
          item.myColor.b = 0;
          item.myColor.a = 1;
          item.label = "";
          item.t_width = item.t_height = 0;
          item.texture = "";
          item.get_light = 0;

          for(i=0;i<numc;i++){
            item.start.x() = contact[i].geom.pos[0];
            item.start.y() = contact[i].geom.pos[1];
            item.start.z() = contact[i].geom.pos[2];
            item.end.x() = contact[i].geom.pos[0] + contact[i].geom.normal[0];
            item.end.y() = contact[i].geom.pos[1] + contact[i].geom.normal[1];
            item.end.z() = contact[i].geom.pos[2] + contact[i].geom.normal[2];
            draw_intern.push_back(item);
            if(geom_data1->c_params.friction_direction1 ||
               geom_data2->c_params.friction_direction1) {
              v[0] = contact[i].geom.normal[0];
              v[1] = contact[i].geom.normal[1];
              v[2] = contact[i].geom.normal[2];
              dot = dDOT(v, contact[i].fdir1);
              dOPEC(v, *=, dot);
              contact[i].fdir1[0] -= v[0];
              contact[i].fdir1[1] -= v[1];
              contact[i].fdir1[2] -= v[2];
              dNormalize3(contact[0].fdir1);
            }
            contact[0].geom.depth += (geom_data1->c_params.depth_correction +
                                      geom_data2->c_params.depth_correction);
        
            if(contact[0].geom.depth < 0.0) contact[0].geom.depth = 0.0;
            dJointID c=dJointCreateContact(world,contactgroup,contact+i);
            dJointAttach(c,b1,b2);

            geom_data1->num_ground_collisions += numc;
            geom_data2->num_ground_collisions += numc;

            contact_point.x() = contact[i].geom.pos[0];
            contact_point.y() = contact[i].geom.pos[1];
            contact_point.z() = contact[i].geom.pos[2];

            geom_data1->contact_ids.push_back(geom_data2->id);
            geom_data2->contact_ids.push_back(geom_data1->id);
            geom_data1->contact_points.push_back(contact_point);
            geom_data2->contact_points.push_back(contact_point);
            //if(dGeomGetClass(o1) == dPlaneClass) {
            fb = 0;
            if(geom_data2->sense_contact_force) {
              fb = (dJointFeedback*)malloc(sizeof(dJointFeedback));
              dJointSetFeedback(c, fb);
              contact_feedback_list.push_back(fb);
              geom_data2->ground_feedbacks.push_back(fb);
              geom_data2->node1 = false;
            } 
            //else if(dGeomGetClass(o2) == dPlaneClass) {
            if(geom_data1->sense_contact_force) {
              if(!fb) {
                fb = (dJointFeedback*)malloc(sizeof(dJointFeedback));
                dJointSetFeedback(c, fb);
                contact_feedback_list.push_back(fb);
              }
              geom_data1->ground_feedbacks.push_back(fb);
              geom_data1->node1 = true;
            }
          }
        }
	  }
	}
      delete[] contact;
	}
    /**
     * \brief This static function is used to project a normal function
     *   pointer to a method from a class
     *
     * pre:
     *     - data is a pointer to a correct object from type WorldPhysics
     *
     * post:
     *     - the newCallback method of the data object should be called
     */
    void WorldPhysics::callbackForward(void *data, dGeomID o1, dGeomID o2) {
      WorldPhysics *wp = (WorldPhysics*)data;
      wp->nearCallback(o1, o2);

    }

    /**
     * \brief resets the mass of a composite body
     *
     * pre:
     *     - the body exists in the physical world
     *
     * post:
     */
    void WorldPhysics::resetCompositeMass(dBodyID theBody) {
      std::vector<body_nbr_tupel>::iterator iter;
      std::vector<NodePhysics*>::iterator jter;
      dMass bodyMass, tmpMass;
      //bool first = 1;

      for(iter = comp_body_list.begin(); iter != comp_body_list.end(); iter++) {
        if((*iter).body == theBody) {      
          dMassSetZero(&bodyMass);
          for(jter = (*iter).comp_nodes.begin();
              jter != (*iter).comp_nodes.end(); jter++) {
            (*jter)->addMassToCompositeBody(theBody, &bodyMass);
          }
          dBodySetMass(theBody, &bodyMass);
          break;
        }
      }
    }

    void WorldPhysics::moveCompositeMassCenter(dBodyID theBody, dReal x,
                                               dReal y, dReal z) {
      std::vector<body_nbr_tupel>::iterator iter;
      std::vector<NodePhysics*>::iterator jter;
      const dReal *bpos;

      // first we have to calculate the offset in bodyframe
      // so rotate the vector
      bpos = dBodyGetPosition(theBody);
      dBodySetPosition(theBody, bpos[0]+x, bpos[1]+y, bpos[2]+z);
      for(iter = comp_body_list.begin(); iter != comp_body_list.end(); iter++) {
        if((*iter).body == theBody) {      
          for(jter = (*iter).comp_nodes.begin();
              jter != (*iter).comp_nodes.end(); jter++) {
            (*jter)->addCompositeOffset(-x, -y, -z);
          }
          break;
        }
      }
    }

    const Vector WorldPhysics::getCenterOfMass(const std::vector<NodeInterface*> &nodes) const {
      MutexLocker locker(&iMutex);
      Vector center;
      std::vector<NodeInterface*>::const_iterator iter;
      dMass sumMass;
      dMass tMass;

      dMassSetZero(&sumMass);
      for(iter = nodes.begin(); iter != nodes.end(); iter++) {
        ((NodePhysics*)(*iter))->getAbsMass(&tMass);
        dMassAdd(&sumMass, &tMass);
      }
      center.x() = sumMass.c[0];
      center.y() = sumMass.c[1];
      center.z() = sumMass.c[2];
      return center;
    }

    void WorldPhysics::update(std::vector<draw_item>* drawItems) {
      MutexLocker locker(&drawLock);
      std::vector<draw_item>::iterator iter;
  
      for(iter=drawItems->begin(); iter!=drawItems->end(); iter++) {
        iter->draw_state = DRAW_STATE_ERASE;
      }
      if(draw_contact_points) {
        for(iter=draw_extern.begin(); iter!=draw_extern.end(); iter++) {
          drawItems->push_back(*iter);
        }
      }

		//conforce[0] = control->nodes->getContactForce(34);   //ptp
		//conforce[1] = control->nodes->getContactForce(48);
		//conforce[2] = control->nodes->getContactForce(61);
		//conforce[3] = control->nodes->getContactForce(73);
		//conforce[4] = control->nodes->getContactForce(87);
		//conforce[5] = control->nodes->getContactForce(99);
		//conforce_all =conforce[0]+conforce[1]+conforce[2]+conforce[3]+conforce[4]+conforce[5];
		
		//Vector footpos;
		//for(int i=0;i<6;i++){
			//if(first_colliding[i] == false){   	// i = foot_id; PTP exists
				//switch (geom_data1->id){							
					//case 34: footpos = control->nodes->getPosition(34);
							//if((footpos.z()-0.05f) > end_soil[i]) {
							//printf("foot %d (%f %f)..over..over\n", i, footpos.z()-0.05, end_soil[i]);		
							//if(!control->nodes->popPTPlist(i)) printf("footID= %d pop error\n", i);
							//first_colliding[i] = true;
							//heightMap_collision[i] = false;
		//printf("%lu.. first_colliding....reset (%f %f)\n",geom_data1->id, footpos.z()-0.05, end_soil[0]);
							
							//}
							//break;
					//case 48: footpos = control->nodes->getPosition(48);
							//if((footpos.z()-0.05f) > end_soil[i]) {
							//printf("foot %d (%f %f)..over..over\n", i, footpos.z()-0.05, end_soil[i]);		
							//if(!control->nodes->popPTPlist(i)) printf("footID= %d pop error\n", i);
							//first_colliding[i] = true;							
							//heightMap_collision[i] = false;
		//printf("%lu.. first_colliding....reset (%f %f)\n",geom_data1->id, footpos.z()-0.05, end_soil[0]);
							
							//}
							//break;
					//case 61: footpos = control->nodes->getPosition(61);
							//if((footpos.z()-0.05f) > end_soil[i]) {
							//printf("foot %d (%f %f)..over..over\n", i, footpos.z()-0.05, end_soil[i]);		
							//if(!control->nodes->popPTPlist(i)) printf("footID= %d pop error\n", i);
							//first_colliding[i] = true;							
							//heightMap_collision[i] = false;
		//printf("%lu.. first_colliding....reset (%f %f)\n",geom_data1->id, footpos.z()-0.05, end_soil[0]);
							
							//}
							//break;
					//case 73: footpos = control->nodes->getPosition(73);
							//if((footpos.z()-0.05f) > end_soil[i]) {
							//printf("foot %d (%f %f)..over..over\n", i, footpos.z()-0.05, end_soil[i]);		
							//if(!control->nodes->popPTPlist(i)) printf("footID= %d pop error\n", i);
							//first_colliding[i] = true;							
							//heightMap_collision[i] = false;
		//printf("%lu.. first_colliding....reset (%f %f)\n",geom_data1->id, footpos.z()-0.05, end_soil[0]);
							
							//}
							//break;
					//case 87: footpos = control->nodes->getPosition(87);
							//if((footpos.z()-0.05f) > end_soil[i]) {
							//printf("foot %d (%f %f)..over..over\n", i, footpos.z()-0.05, end_soil[i]);		
							//if(!control->nodes->popPTPlist(i)) printf("footID= %d pop error\n", i);
							//first_colliding[i] = true;							
							//heightMap_collision[i] = false;
		//printf("%lu.. first_colliding....reset (%f %f)\n",geom_data1->id, footpos.z()-0.05, end_soil[0]);
							
							//}
							//break;
					//case 99: footpos = control->nodes->getPosition(99);
							//if((footpos.z()-0.05f) > end_soil[i]) {
							//printf("foot %d (%f %f)..over..over\n", i, footpos.z()-0.05, end_soil[i]);		
							//if(!control->nodes->popPTPlist(i)) printf("footID= %d pop error\n", i);
							//first_colliding[i] = true;							
							//heightMap_collision[i] = false;
		//printf("%lu.. first_colliding....reset (%f %f)\n",geom_data1->id, footpos.z()-0.05, end_soil[0]);
	
							//}
							//break;
					//default: break;	
					
				//}
			//}
		//}		
		
	
    }

    int WorldPhysics::handleCollision(dGeomID theGeom) {
      ray_collision = 0;
      dSpaceCollide2(theGeom, (dGeomID)space, this,
                     &WorldPhysics::callbackForward);
      return ray_collision;
    }

    double WorldPhysics::getCollisionDepth(dGeomID theGeom) {
      dGeomID otherGeom;
      dContact contact[1];
      double depth = 0.0;
      int numc;
      dBodyID b1;
      dBodyID b2;

      for(int i=0; i<dSpaceGetNumGeoms(space); i++) {
        otherGeom = dSpaceGetGeom(space, i);

        if(!(dGeomGetCollideBits(theGeom) & dGeomGetCollideBits(otherGeom)))
          continue;

        b1 = dGeomGetBody(theGeom);
        b2 = dGeomGetBody(otherGeom);

        if(b1 && b2 && dAreConnectedExcluding(b1,b2,dJointTypeContact))
          continue;

        numc = dCollide(theGeom, otherGeom, 1 | CONTACTS_UNIMPORTANT,
                        &(contact[0].geom), sizeof(dContact));
        if(numc) {
          if(contact[0].geom.depth > depth)
            depth = contact[0].geom.depth;
        }
      }

      return depth;
    }

    int WorldPhysics::checkCollisions(void) {
      MutexLocker locker(&iMutex);
      num_contacts = log_contacts = 0;
      create_contacts = 0;
      dSpaceCollide(space,this, &WorldPhysics::callbackForward);	
      return num_contacts;
    }

    double WorldPhysics::getVectorCollision(const Vector &pos, 
                                            const Vector &ray) const {
      MutexLocker locker(&iMutex);
      dGeomID otherGeom;
      dContact contact[1];
      //double depth = ray.length();
      double depth = ray.norm();
      int numc;
  
      dGeomID theGeom = dCreateRay(space, depth);
      dGeomRaySet(theGeom, pos.x(), pos.y(), pos.z(), ray.x(), ray.y(), ray.z()); 

      for(int i=0; i<dSpaceGetNumGeoms(space); i++) {
        otherGeom = dSpaceGetGeom(space, i);

        if(!(dGeomGetCollideBits(theGeom) & dGeomGetCollideBits(otherGeom)))
          continue;
        numc = dCollide(theGeom, otherGeom, 1 | CONTACTS_UNIMPORTANT,
                        &(contact[0].geom), sizeof(dContact));
        if(numc) {
          if(contact[0].geom.depth < depth)
            depth = contact[0].geom.depth;
        }
      }

      dGeomDestroy(theGeom);
      return depth;
    }

  } // end of namespace sim
} // end of namespace mars
