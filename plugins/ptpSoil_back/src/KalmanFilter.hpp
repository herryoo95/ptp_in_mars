#include "PTPparameters.hpp"
#include "PTPUtil.hpp"

namespace mars {
  namespace plugins {
    namespace ptpSoil {

class KalmanFilter{
	
public:
	KalmanFilter();
	~KalmanFilter();
	
	vecD kalman(vecD input, int s, PTPUtil* ut);
	double kalmanD(double input, int s);
	
};

    } // end of namespace ptpSoil
  } // end of namespace plugins
} // end of namespace mars
