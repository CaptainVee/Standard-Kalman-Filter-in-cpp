#include <iostream>
#include <vector>
#include <Eigen/Dense>

using std::cout;
using std::endl;
using std::vector;
using Eigen::VectorXd;
using Eigen::MatrixXd;

// Kalman Filter variables
VectorXd x;	// object state
MatrixXd P;	// object covariance matrix
VectorXd u;	// external motion
MatrixXd F; // state transition matrix
MatrixXd H;	// measurement matrix
MatrixXd R;	// measurement covariance matrix
MatrixXd I; // Identity matrix
MatrixXd Q;	// process covariance matrix

vector<VectorXd> measurements;
void filter(VectorXd &x, MatrixXd &P);


int main() {
  /**
   * Code used as example to work with Eigen matrices
   */
  // design the KF with 1D motion
  x = VectorXd(2);
  x << 0, 0;

  P = MatrixXd(2, 2);
  P << 1000, 0, 0, 1000;

  u = VectorXd(2);
  u << 0, 0;

  F = MatrixXd(2, 2);
  F << 1, 1, 0, 1;

  H = MatrixXd(1, 2);
  H << 1, 0;

  R = MatrixXd(1, 1);
  R << 1;

  I = MatrixXd::Identity(2, 2);

  Q = MatrixXd(2, 2);
  Q << 0, 0, 0, 0;

  // create a list of measurements
  VectorXd single_meas(1);
  single_meas << 1;
  measurements.push_back(single_meas);
  single_meas << 2;
  measurements.push_back(single_meas);
  single_meas << 3;
  measurements.push_back(single_meas);

  // call Kalman filter algorithm
  filter(x, P);

  return 0;
}


void filter(VectorXd &x, MatrixXd &P) {
    
    MatrixXd S = MatrixXd(1, 1); // System uncertainity
    MatrixXd K = MatrixXd(1, 1); // Kalman gain
    MatrixXd y = MatrixXd(1, 1); // residual
	
	/**
	we do the measurement and then the prediction in the loop. 
	Over time, the order of these doesn't have a huge impact, 
	since it is just a cycle from one to the other.
	*/
  for (unsigned int n = 0; n < measurements.size(); ++n) {

    VectorXd z = measurements[n];
    
    // KF Measurement update step
    S = H*P*(H.transpose()) + R;
    K = P*(H.transpose())*(S.inverse());
    y = z - H*x;
    
    // new state
	x = x + K*y;
	P = (I - K*H)*P;
	
    // KF Prediction step
	x = F*x;   // B*u = 0 
	/**
	we assume that there is no way to measure or know the exact acceleration of a tracked object. 
	For example, if we were in an autonomous vehicle tracking a bicycle, pedestrian or another car, 
	we would not be able to model the internal forces of the other object; hence, 
	we do not know for certain what the other object's acceleration is. 
	Instead, we will set B*u = 0
	*/
	P = F*P*(F.transpose()) + Q;
	
		
    cout << "x=" << endl <<  x << endl;
    cout << "P=" << endl <<  P << endl;
  }
}