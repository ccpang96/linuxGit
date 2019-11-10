#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Dense"


//70多个包的其中1个，里面又有多个目标值
/**
 * @brief The MeasurementPackage class
 * 说明：70多个包中的1个
 * raw_measurements_:
 * 0.径向距离
 * 1.方位角
 * 2.径向速度
 */
class MeasurementPackage {
public:
  Eigen::VectorXd raw_measurements_;
};

#endif /* MEASUREMENT_PACKAGE_H_ */
