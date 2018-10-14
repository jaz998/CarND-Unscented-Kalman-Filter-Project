#include <iostream>
#include "tools.h"

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

	// Initializing RMSE
	VectorXd rmse(4);
	rmse << 0.0, 0.0, 0.0, 0.0;

	if (estimations.size() == 0) {
		cout << "ERROR - CalculateRMSE() - The estimations vector is empty" << endl;
		return rmse;
	}

	if (ground_truth.size() == 0) {
		cout << "ERROR - CalculateRMSE() - The ground-truth vector is empty" << endl;
		return rmse;
	}

	unsigned int n = ground_truth.size();
	if (estimations.size() != n) {
		cout << "ERROR - CalculateRMSE() - estimation size is different compared to ground truth size" << endl;
		return rmse;
	}

	for (unsigned int i = 0; i<n; i++) {
		VectorXd difference = estimations[i] - ground_truth[i];
		difference = difference.array()*difference.array();
		rmse = rmse + difference;
	}

	rmse = rmse / n;
	rmse = rmse.array().sqrt();
	return	rmse;


}