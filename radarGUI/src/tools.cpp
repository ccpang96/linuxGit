#include <iostream>
#include "include/radarGUI/tools.h"
#include "include/radarGUI/mainwindow.h"
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

const double PI_2 = M_PI / 2;

/**
 * TODO kRadarMaxDetectionRange
 * 说明：雷达可以探测到的最大距离，记得修改
 */
const int kRadarMaxDetectionRange = 500;  //

//用于计算雅克比矩阵
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

  MatrixXd Hj(3,4);
  Hj << 0,0,0,0,
        0,0,0,0,
        0,0,0,0;

  //取出状态变量
  float px = static_cast<float>(x_state(0));
  float py = static_cast<float>(x_state(1));
  float vx = static_cast<float>(x_state(2));
  float vy = static_cast<float>(x_state(3));

  //先把求雅克比过程中要用到的中间变量求出来
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  //检查除数是否为0
  if(fabs(static_cast<double>(c1)) < 0.0001){
    std::cout << "CalculateJacobian()函数出错: 除数为0" << std::endl;
    return Hj;
  }

  //计算出雅克比矩阵
  Hj << (px/c2),                (py/c2),                0,      0,
        -(py/c1),               (px/c1),                0,      0, 
        py*(vx*py - vy*px)/c3,  px*(px*vy - py*vx)/c3,  px/c2,  py/c2;

  return Hj;

}

/**
 * @brief Tools::RadarPolarToCartesian
 * @param x_measurement
 * @return x_state
 * 说明：将数据从极坐标系变换到笛卡尔坐标系
 */
VectorXd Tools::RadarPolarToCartesian(const VectorXd &x_measurement) {

    double r = x_measurement(0); //径向距离
    double phi = x_measurement(1); //方位角
    double r_dot = x_measurement(2); //速度

    double x;
    double y;
    double vx;
    double vy;

    x = r * cos(phi);
    y = r * sin(phi);
    vx = r_dot * cos(phi);
    vy = r_dot * sin(phi);

    VectorXd x_state = VectorXd(4);
    x_state << x,y,vx,vy;
    return x_state;
}


/**
 * @brief RadarCartesianToPolar
 * @param x_state
 * @return VectorXd
 * @state:将状态变量从笛卡尔坐标系变换到polar坐标系
 */

VectorXd Tools::RadarCartesianToPolar(const VectorXd &x_state){

  /**
   * 将状态变量从笛卡尔坐标系(x,y,vx,vy)变换到极坐标系(rho,phi,rhi_dot)
  */
  double px, py, vx, vy;
  px = x_state[0];  //x
  py = x_state[1];  //y
  vx = x_state[2];  //水平速度
  vy = x_state[3];  //垂直速度

  double rho, phi, rho_dot;
  rho = sqrt(px*px + py*py);

  /**TODO:
   * 避免径向速度为0，除数为0的bug
   */
  if(rho < 0.000001)
    rho = 0.000001;

  rho_dot = (px * vx + py * vy) / rho;

  /**TODO:
   *返回的角度值在-pi到pi之间，在画航迹时采用以下公式转换：
   * if (phi > 0)
   *    phi = 180 * phi / M_PI;
   * else
   *    phi = 360 - 180 * phi / M_PI;
   */
   phi = atan2(py, px);

  VectorXd z_pred = VectorXd(3);
  z_pred << rho, phi, rho_dot;
  return z_pred;
}

/**
 * @brief Tools::RadarPolarToGUI
 * @param resultEKF
 * @return  GUIdata
 * 说明：  写一个函数，将含有径向距离、径向速度、角度的resultEKF值转换成可以打印在radarGUI上的值GUIdata
  要确保此时圆心为坐标原点
  如果角度值resultEKF(1)是大于0的，说明是位于第1和第2象限；如果小于0，则是位于第三和第四象限
  圆的半径是kRoundDiameter / 2  = 350个像素点
 */
VectorXd Tools::RadarPolarToGUI(const VectorXd &resultEKF) {

    double rho;
    double phi;
    double rhoGUI;

    int x; //x坐标值 以当前圆心为坐标原点
    int y; //y坐标值
    double v;
    rho = resultEKF(0);
    phi = resultEKF(1);
    rhoGUI = (rho * kRoundDiameter) / (kRadarMaxDetectionRange * 2);
    //qDebug() << rhoGUI<<endl;

    /**
      @brief:如果角度大于0，说明位于第1和第2象限，则直接打印
    */
    if (phi > 0 && phi <= PI_2)  {// 位于第一象限
        y = -rhoGUI * cos(phi);
        x = rhoGUI * sin(phi);
    } else if  (phi > PI_2 && phi <= PI) {  //第2象限
        y = rhoGUI * cos(PI - phi);
        x = rhoGUI * sin(PI - phi);
    } else if (phi > -PI && phi <= -PI_2)   {//第3象限
        y =   rhoGUI * cos(phi + PI);
        x =   -rhoGUI * sin(phi + PI);
    }
    else if (phi > -PI_2 && phi  <= 0) { // -pi/2 < phi < 0
        y =  -rhoGUI * cos(-phi);
        x =  -rhoGUI *sin(-phi);
    }
    else {
        qDebug() << "角度不对啊！！" << "\t"<< phi <<endl;

    }

    //qDebug() << x<< "\t"<<y<<endl;
    v = resultEKF(2);
    VectorXd GUIdata = VectorXd(3);
    GUIdata << x,y,v;
    return GUIdata;
}













