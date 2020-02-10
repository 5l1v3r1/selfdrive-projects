#include <math.h>
#include <iostream>
#include <fstream>
#include "json.hpp"
#include <eigen3/Eigen/Dense>
#include "matplotlibcpp.h"

using json = nlohmann::json;
using namespace std;
using namespace Eigen;
namespace plt = matplotlibcpp;


double wraptopi(double x) {
  if (x > M_PI) {
    x = x - (floor(x / (2 * M_PI)) + 1) * 2 * M_PI;
  } else if (x < -M_PI) {
    x = x + (floor(x / (-2 * M_PI)) + 1) * 2 * M_PI;
  }
  return x;
}


void measurement_update(vector<double> &lk, double rk, double bk, Matrix<double,3,3> &P_check, Matrix<double,3,1> &x, Matrix<double,2,2> cov_y) {

  bk = wraptopi(bk);

  // 1. Compute measurement Jacobian
  Matrix<double,2,1> y;
  double dist = pow(lk[0] - x(0, 0), 2) + pow((lk[1] - x(1, 0)), 2);
  y(0, 0) = sqrt(dist);
  y(1, 0) = atan2(lk[1] - x(1, 0), lk[0] - x(0, 0)) - x(2, 0);
  y(1, 0) = wraptopi(y(1, 0));

  Matrix<double,2,3> H;

  H << (x(0, 0) - lk[0]) / sqrt(dist), (x(1, 0) - lk[1]) / sqrt(dist), 0,
       -(x(1, 0) - lk[1]) / dist, -(-x(0, 0) + lk[0]) / dist, -1;

  // 2. Compute Kalman Gain
  auto K = P_check * (H.transpose()) * ((H*P_check * (H.transpose()) + cov_y).inverse());

  // 3. Correct predicted state
  Matrix<double,2,1> y_m;
  y_m << rk, bk;

  Matrix<double,2,1> y_d;
  y_d = y_m - y;
  y_d(1, 0) = wraptopi(y_d(1, 0));

  x = x + K * y_d;
  x(2, 0) = wraptopi(x(2, 0));

  // 4. Correct covariance
  P_check = (Matrix3d::Identity() - K * H) * P_check;
}


int main() {
  // read a JSON file
  json j;
  ifstream i("data/data.json");
  i >> j;

  //cout << j["b"] << endl;

  vector<double> t = j["t"];
  double x_init = j["x_init"];
  double y_init = j["y_init"];
  double th_init = j["th_init"];


  vector<double> v = j["v"];
  vector<double> om = j["om"];

  vector<vector<double>> b = j["b"];
  vector<vector<double>> r = j["r"];
  vector<vector<double>> l = j["l"];

  double v_var = 0.1;
  double om_var = 0.05;
  double r_var = 0.01;
  double b_var = 0.02;

  Matrix2d Q_km(2, 2);
  Q_km << v_var, 0,
          0, om_var;

  Matrix<double,2,2> cov_y;
  cov_y << r_var, 0,
          0, b_var;

  vector<Matrix<double,3,1> > x_est(v.size());
  vector<Matrix<double,3,3> > P_est(v.size());


  x_est[0](0, 0) = x_init;
  x_est[0](1, 0) = y_init;
  x_est[0](2, 0) = th_init;

  P_est[0](0, 0) = 1.;
  P_est[0](1, 1) = 1.;
  P_est[0](2, 2) = .1;

  cout << x_est[0] << endl;

  // Main Filter Loop
  for (int k=1; k < t.size(); k++) {

    // 1. Update state with odometry readings (remember to wrap the angles to [-pi,pi])
    Matrix<double,3,1> x_check;
    x_check << x_est[k-1](0, 0) + v[k] * cos(x_est[k-1](2, 0)),
               x_est[k-1](1, 0) + v[k] * sin(x_est[k-1](2, 0)),
               x_est[k-1](2, 0) + om[k];
    x_check[2] = wraptopi(x_check[2]);

    // 2. Motion model jacobian with respect to last state
    Matrix<double,3,3> F_km;
    F_km << 1, 0, -v[k] * sin(x_est[k-1](2, 0)),
            0, 1, v[k] * cos(x_est[k-1](2, 0)),
            0, 0, 1;

    // 3. Motion model jacobian with respect to noise
    Matrix<double,3,2> L_km;
    L_km << cos(x_est[k-1](2, 0)), 0,
            sin(x_est[k-1](2, 0)), 0,
            0, 1;

    // 4. Propagate uncertainty
    Matrix<double,3,3> P_check = F_km * P_est[k-1] * F_km.transpose() + L_km * Q_km * L_km.transpose();

    // 5. Update state estimate using available landmark measurements
    for (int i=0; i<r[k].size(); i++) {
      measurement_update(l[i], r[k][i], b[k][i], P_check, x_check, cov_y);
    }

    // Set final state predictions for timestep
    x_est[k](0, 0) = x_check(0, 0);
    x_est[k](1, 0) = x_check(1, 0);
    x_est[k](2, 0) = x_check(2, 0);
    P_est[k] = P_check;
  }

  cout << P_est[1]<<endl;

  // plot
  vector<double> xs,ys, ths;

  for (auto x:x_est) {
    xs.push_back(x[0]);
    ys.push_back(x[1]);
    ths.push_back(x[2]);
  }

  plt::plot(xs, ys);
  plt::show();

  plt::plot(t, ths);
  plt::show();
}
