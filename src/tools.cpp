#include <iostream>
#include "tools.h"
#include <stdexcept>


using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */

	VectorXd rmse(4);
	rmse << 0,0,0,0;
	// check validity
	if (estimations.size() == 0) {
		throw std::invalid_argument("estimations size is zero");
	}
	else if (ground_truth.size() != estimations.size()) {
		throw std::invalid_argument("estimations and ground truth vector sizes do not match");
	}


	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i){
        // ... your code here
        //#rmse += (estimations[i] - ground_truth[i]) * (estimations[i] - ground_truth[i]);
        //cout << (estimations[i] - ground_truth[i]) 
        VectorXd D = (estimations[i] - ground_truth[i]);
        D = D.array() * D.array();
        rmse = rmse + D;
	}

	//calculate the mean
	// ... your code here
    rmse = rmse / estimations.size();
	//calculate the squared root
	// ... your code here
    rmse = rmse.array().sqrt();
	//return the result
	return rmse; 
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);

	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//pre-compute a set of terms to avoid repeated calculation
	float c1 = px*px + py*py;
	float c2 = sqrt(c1);
	float c3 = (c1*c2);

	//check division by zero
	if(fabs(c1) < 0.0001)
	{
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		return Hj;
	}

	//compute the Jacobian matrix
	Hj << (px/c2), (py/c2), 0, 0,
		  -(py/c1), (px/c1), 0, 0,
		  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

	return Hj; 
}
