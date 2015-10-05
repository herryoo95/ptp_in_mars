/** A simple kalman filter example by Adrian Boeing 
 www.adrianboeing.com 
 */ 
#include "KalmanFilter.hpp"

namespace mars {
  namespace plugins {
    namespace ptpSoil {

KalmanFilter::KalmanFilter(){}
KalmanFilter::~KalmanFilter(){}

vecD KalmanFilter::kalman(vecD input, int s, PTPUtil* ut) {

    //initial values for the kalman filter
    static vecD x_est_last;// = setVecD(0.f,0.f,0.f);
    static vecD P_last;// = setVecD(0.f,0.f,0.f);
    //the noise in the system
    vecD Q = ut->setVecD(FILTER_Q,FILTER_Q,FILTER_Q);
    vecD R = ut->setVecD(FILTER_R,FILTER_R,FILTER_R);
    
    vecD K;
    vecD P;

    vecD P_temp;
    vecD x_temp_est;

	vecD x_est;
    vecD z_measured; //the 'noisy' value we measured
    
	static int state=1;
    
	if(state==0 || s==0){
		state = 1;
    //initialize with a measurement
	    x_est_last = input;
	}
    
//    for (int i=0;i<3000;i++) {
        //do a prediction
        x_temp_est = x_est_last;
        P_temp = ut->plusVecD(&P_last,&Q);
        //calculate the Kalman gain
//        K = P_temp * (1.0/(plusVecD(&P_temp,&R)));
		K.x = P_temp.x * (1.0/(P_temp.x+R.x));
		K.y = P_temp.y * (1.0/(P_temp.y+R.y));
		K.z = P_temp.z * (1.0/(P_temp.z+R.z));

		//measure
        z_measured = input; //the real measurement plus noise
        //correct
        //x_est = x_temp_est + K * minusVecD(&z_measured,&x_temp_est); 
		x_est.x = x_temp_est.x + K.x * (z_measured.x-x_temp_est.x);
		x_est.y = x_temp_est.y + K.y * (z_measured.y-x_temp_est.y);
		x_est.z = x_temp_est.z + K.z * (z_measured.z-x_temp_est.z);

//		P = (1- K) * P_temp;
		P.x = (1-K.x)*P_temp.x;
		P.y = (1-K.y)*P_temp.y;
		P.z = (1-K.z)*P_temp.z;

		//we have our new system
        
//        printf("Ideal    position: %6.3f \n",z_real);
//        printf("Mesaured position: %6.3f [diff:%.3f]\n",z_measured,fabs(z_real-z_measured));
//        printf("Kalman   position: %6.3f [diff:%.3f]\n",x_est,fabs(z_real - x_est));
        
        //update our last's
        P_last = P;
        x_est_last = x_est;
//    }
    
//    printf("Total error if using raw measured:  %f\n",sum_error_measure);
//    printf("Total error if using kalman filter: %f\n",sum_error_kalman);
//    printf("Reduction in error: %d%% \n",100-(int)((sum_error_kalman/sum_error_measure)*100));
    
    
    return x_est;
}

double KalmanFilter::kalmanD(double input, int s) {

    //initial values for the kalman filter
    static double x_est_last;// = setVecD(0.f,0.f,0.f);
    static double P_last;// = setVecD(0.f,0.f,0.f);
    //the noise in the system
    double Q = FILTER_Q;
    double R = FILTER_R;
    
    double K;
    double P;

    double P_temp;
    double x_temp_est;

	double x_est;
    double z_measured; //the 'noisy' value we measured
    
	static int state=1;
    
	if(state==0 || s==0){
		state = 1;
    //initialize with a measurement
	    x_est_last = input;
	}
    
//    for (int i=0;i<3000;i++) {
        //do a prediction
        x_temp_est = x_est_last;
        P_temp = (P_last+Q);
        //calculate the Kalman gain
        K = P_temp * (1.0/(P_temp+R));

		//measure
        z_measured = input; //the real measurement plus noise
        //correct
        x_est = x_temp_est + K * (z_measured-x_temp_est); 

		P = (1- K) * P_temp;

		//we have our new system
        
//        printf("Ideal    position: %6.3f \n",z_real);
//        printf("Mesaured position: %6.3f [diff:%.3f]\n",z_measured,fabs(z_real-z_measured));
//        printf("Kalman   position: %6.3f [diff:%.3f]\n",x_est,fabs(z_real - x_est));
        
        //update our last's
        P_last = P;
        x_est_last = x_est;
//    }
    
//    printf("Total error if using raw measured:  %f\n",sum_error_measure);
//    printf("Total error if using kalman filter: %f\n",sum_error_kalman);
//    printf("Reduction in error: %d%% \n",100-(int)((sum_error_kalman/sum_error_measure)*100));
    
    
    return x_est;
}

    } // end of namespace ptpSoil
  } // end of namespace plugins
} // end of namespace mars
