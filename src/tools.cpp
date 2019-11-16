#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
{
  /**
   * TODO: Calculate the RMSE here.
   *
   *
   */

  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // TODO: YOUR CODE HERE
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size

  if (!estimations.size() || !ground_truth.size() ||
      (estimations.size() != ground_truth.size()))
    return rmse;

  // TODO: accumulate squared residuals
  for (int i = 0; i < estimations.size(); ++i)
  {
    // ... your code here
    VectorXd error = (estimations[i] - ground_truth[i]);

    error = error.array() * error.array();
    rmse += error;
  }

  // TODO: calculate the mean
  rmse = rmse.array() / estimations.size();
  // TODO: calculate the squared root
  rmse = rmse.array().sqrt();
  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state)
{
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3, 4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE

  // check division by zero
  if (px == 0 && py == 0)
  {
    std::cout << "Error computing jacobian ";
    Hj = MatrixXd::Zero(3, 4);
    return Hj;
  }

  // compute the Jacobian matrix
  float norm_psquare = px * px + py * py;
  float norm = sqrtf(norm_psquare);
  float temp = vx * py - vy * px;
  float temp1 = vy * px - vx * py;

  Hj << px / norm, py / norm, 0, 0, -1 * py / norm_psquare, px / norm_psquare,
      0, 0, py * temp / (norm * norm * norm), px * temp1 / (norm * norm * norm),
      px / norm, py / norm;

  return Hj;
}

void print_vec(const VectorXd &x)
{
  for (int i = 0; i < x.size(); ++i)
  {
    std::cout << x(i);
    if (i != x.size() - 1)
    {
      std::cout << ",";
    }
  }
}

void print_matrix(const MatrixXd &mat, const std::string &id)
{

  std::cout << id << '\n';
  std::cout << "(" << mat.rows() << "," << mat.cols() << ")"
            << "\n";
  std::cout << mat << "\n";
}
