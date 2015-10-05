#ifndef __PTPSYSTEM_H__
#define __PTPSYSTEM_H__
#include "PTPCore.h"
//#include "PTPUtil.h"

namespace mars {
  namespace sim {


class PTPSystem : public PTPCore{
public:
	
	PTPSystem();
    ~PTPSystem();
    void  endPTP();
    char* printPTP();
    char* printCELL(const vecN n);
    char* printPARTICLE(const vecN n);
    bool getPTPForce(FOOT (&foot)[6], vecD (&force)[6], vecD (&forceT)[6]);  //bool (&a)[10]
private:

    
};

  } // end of namespace sim
} // end of namespace mars

#endif // __PTPSYSTEM_H__
