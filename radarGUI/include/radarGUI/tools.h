#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Dense"
#include "QDebug"

class Tools {
public:
  /**
  *构造函数
  */
  Tools();

  /**
  * 析构函数
  */
  virtual ~Tools();

  /**
  * 用于计算雅克比矩阵的函数
  */
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);
  Eigen::VectorXd RadarCartesianToPolar(const Eigen::VectorXd &x_state);
  Eigen::VectorXd RadarPolarToGUI(const Eigen::VectorXd &resultEKF);
  Eigen::VectorXd Tools::RadarPolarToCartesian(const Eigen::VectorXd &x_measurement);
};

#endif /* TOOLS_H_ */
