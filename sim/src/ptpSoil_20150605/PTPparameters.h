#ifndef __PTPPARAMETERS_H__
#define __PTPPARAMETERS_H__

///Experiment Option
//#define CYLINDER_EXPERIMENT		 
#define SPHERE_EXPERIMENT			 
//#define SPHERE_EXPERIMENT2			 

//#define CELL_TEST_SPHERE
//#define CELL_TEST_CYLINDER

///Precision
//#define PRECISE
//#define DRAFT

///CALC Option
#define VEL_DIR_EFFECT
//#define FR_BINDING	//!


///Execution Options
//#define USE_SOIL_VISUALIZATION	///SOIL  

/// FOR DEBUG
#define USE_PRESSURE_DEBUG
//#define USE_LOCAL_DEBUG		 
#define USE_LOCAL_TRACE			 
#define USE_FORCE_DEBUG			 
//#define USE_COLLISION_DEBUG		 
//#define USE_AREA_DEBUG
///Essentional Option(FIXED)
#define FAST_CALC

#define USING_FR		//!
//#define FR_INNER_PRODUCT		 
#define USE_MOVEMENT_FUNCTION		 

///Movement Function Option
//#define USE_MOVEMENT_TOP
#define USE_GRAVITY
///Failed Option
//#define USE_PROJ_ARRAY
#ifndef USING_FR		
//#define ACTIVATE_FZ	//!
//#define ACTIVATE_FL	//!
//#define ADAPT_FL_DIRECTION
#endif

#define PI 3.1415926536
#define DEG2RAD (PI/180.0)
#define RAD2DEG (180.0/PI)

#define SPREAD 1  //50
#define FRICTION 0
#define GRAVITY -0.1//-9.80665			 
#define EPSILON_1 0.0001   // Define your own tolerance

#define COHESION 1100   //[Pa]   granular:600-900, fine: 1000-1195
#define INT_ANGLE 40*DEG2RAD  //[Pa]   granular:600-900, fine: 1000-1195
#define DEFORMATION_MODULE 0.05

#define MAG 1.0
#define MAX_X 200					/// X�� �ִ� �� ����
#define MAX_Y 200					/// Y�� �ִ� �� ����
#define MAX_Z 200					/// Z�� �ִ� �� ����

///FILER
#define FILTER_Q 0.022
#define FILTER_R 0.617

///Laplace Equation Parameters
#define PANEL_Y	(-8e-6)				///Gravity
#define PANEL_ZX (3e-3)				///

///Soil Visualization Parameters
#define SOIL_PANEL_ZX 8e-4			///�ҵ�¡ ����Ʈ �Ķ����(����κ����� �����ϴ���) �ӵ��� ����������
#define SOIL_DEGREE	1.5				///�ҵ�¡ ����Ʈ �Ķ����(��ŭ �����ϴ���..) XZ �� �������
#define SOIL_DEGREE_Y	0.8

///Lookup Table Parameters
#define LOOKUP_K	895980
#define LOOKUP_N	0.785

#endif
