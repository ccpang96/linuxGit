// Copyright (c) 2019 The radarGUI Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "include/radarGUI/mainwindow.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
extern Tools tools1;


/**
 * @brief MainWindow::MainWindow
 * @param parent
 * 说明：主窗口的构造函数：
 */
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

    ui->setupUi(this);

    SetGUIStyle();
    //setWindowFlags(Qt::FramelessWindowHint);  //隐藏窗口的标题栏
    //object_track(); //跟踪部分


    QTimer *qtime = new QTimer(this);
    connect(qtime,SIGNAL(timeout()),this,SLOT(DrawScanningGUI())); //用中断实现每隔5ms显示一次界面
    qtime->start(5);//5ms 更新一次
}

/**
 * @brief MainWindow::~MainWindow
 * 说明：析构函数
 */
MainWindow::~MainWindow()
{
    delete ui;
}

/*************************第一部分：目标跟踪**********************/


/*
void objectTrack(vector<vector<MeasurementPackage>> &data_list) {


    int number_frame; //第几帧(批数据)
    double dt = 0.021; //每两批数据之间的间隔 单位s

    track(measurement_pack_list,track_data_output_pack_list,temp_point_pack_list,trust_track_pack_list,
                number_frame, number_of_track,dt,time_accumulate);

    track(vector<MeasurementPackage> &measurement_pack_list,
            vector<TrackDataOutputPackage> &track_data_output_pack_list,
            vector<TempPointPackage>&temp_point_pack_list,
            vector<TrustTrackPackage>&trust_track_pack_list,
            int number_frame,int number_of_track,double dt,double time_accumulate);
}
*/

/*
int object_track() {


   string in_file_name_ = "Z:\\sister\\radarGUI\\data\\input_radar.txt";
   ifstream in_file_(in_file_name_.c_str(), ifstream::in);
   string out_file_name_ = "Z:\\sister\\radarGUI\\data\\output.txt";
   ofstream out_file_(out_file_name_.c_str(), ofstream::out);

   //检查文件打开是否正确
   check_files(in_file_, in_file_name_, out_file_, out_file_name_);

  vector<MeasurementPackage> measurement_pack_list;  //存放测量值的vector
  vector<GroundTruthPackage> gt_pack_list;          //存放真实值的vector

  string line;

  while (getline(in_file_, line)) {


    istringstream iss(line);

    long long timestamp;  //用于存放时刻值
    iss >> timestamp;

    //读入测量值
    MeasurementPackage meas_package;
    meas_package.timestamp_ = timestamp;

    meas_package.raw_measurements_ = VectorXd(3);  //用于存放点目标信息 为了验证跟踪效果，修改为5
    float ro;
    float phi;
    float ro_dot;
    iss >> ro;            //读入径向距离
    iss >> phi;           //读入角度
    iss >> ro_dot;        //读入径向速度
    meas_package.raw_measurements_ << ro, phi, ro_dot;  //将读取的值存入meas_package中
    measurement_pack_list.push_back(meas_package);



    GroundTruthPackage gt_package;
    gt_package.gt_values_ = VectorXd(2); //用于存放真实点目标信息
    float gt_x;
    float gt_y;
    iss >> gt_x;
    iss >> gt_y;
    gt_package.gt_values_ << gt_x, gt_y;
    gt_pack_list.push_back(gt_package);

  }

  //开始进行EKF滤波
  FusionEKF fusionEKF;
  VectorXd  resultEKF;  //这个里面放的就是最终的卡尔曼跟踪结果

  size_t N = measurement_pack_list.size();
  for (size_t k = 0; k < N; ++k) {

      //从第二frame开始滤波，因为第一帧的速度未知
      fusionEKF.ProcessMeasurement(measurement_pack_list[k]);

      // 输出估计值(x,y,vx,vy)
      out_file_ << fusionEKF.ekf_.x_(0) << "\t";
      out_file_ << fusionEKF.ekf_.x_(1) << "\t";
      out_file_ << fusionEKF.ekf_.x_(2) << "\t";
      out_file_ << fusionEKF.ekf_.x_(3) << "\t";


      //将当前的测量值从极坐标系变换到GUI坐标系
      measurementGUI = tools1.RadarPolarToGUI(measurement_pack_list[k].raw_measurements_);
      measurementGUI_list.push_back(measurementGUI);


      //将当前时刻的卡尔曼估计值从笛卡尔坐标系变换到极坐标系
      resultEKF = tools1.RadarCartesianToPolar(fusionEKF.ekf_.x_);
      resultEKFGUI = tools1.RadarPolarToGUI(resultEKF);  //GUI上的x,y,v
      resultEKFGUI_list.push_back(resultEKFGUI);

      // 输出测量值
      float ro = static_cast<float>(measurement_pack_list[k].raw_measurements_(0));
      float phi = static_cast<float>(measurement_pack_list[k].raw_measurements_(1));
      out_file_ << ro * cos(phi) << "\t";
      out_file_ << ro * sin(phi) << "\t";


      //输出真实值
      float gt_x = static_cast<float>(gt_pack_list[k].gt_values_(0));
      float gt_y = static_cast<float>(gt_pack_list[k].gt_values_(1));
      out_file_ << gt_x << "\t";
      out_file_ << gt_y << "\t";

      //输出卡尔曼估计值
      float rho_ekf = resultEKF(0);
      float phi_ekf = resultEKF(1);
      float rho_dot_ekf = resultEKF(2);
      out_file_ << rho_ekf << "\t";
      out_file_ << phi_ekf << "\t";
      out_file_ << rho_dot_ekf << "\n";
  }


  if (out_file_.is_open()) {
    out_file_.close();
  }
  if (in_file_.is_open()) {
    in_file_.close();
  }

  return 0;
}
*/






/**
 * @brief plotMeasurements
 * @param painter
 * @param measurement_pack_list
 * 说明:将测量值在GUI上打印出来
 */
void plotMeasurements(QPainter &painter, const vector<MeasurementPackage> &measurement_pack_list) {

    int x;
    int y;
    VectorXd  measurementGUI; //临时变量，存放转换到GUI上的坐标信息
    QColor qcolor_round(50,205,50);  //点的颜色
    painter.setPen(QPen(qcolor_round, 1, Qt::DotLine, Qt::RoundCap));
    size_t N = measurement_pack_list.size();
    for (size_t k = 0; k < N; ++k) {
        measurementGUI = tools1.RadarPolarToGUI(measurement_pack_list[k].raw_measurements_);
        x = static_cast<int>(measurementGUI(0));
        y = static_cast<int>(measurementGUI(1));
        painter.drawPoint(x,y);
    }
}


/**
 * @brief plotTracks;
 * @param painter
 * @param measurementGUI_list
 * 说明：将航迹信息track_data_output_pack_list在GUI上画出来 航迹信息中可不止一条航迹
 */
void plotTracks(QPainter &painter, const vector<TrackDataOutputPackage> &track_data_output_pack_list)
{

    int x;
    int y;
    VectorXd  trackGUI; //临时变量，存放转换到GUI上的坐标信息
    QColor qcolor_round_shi(220,20,60);  //实点的颜色
    painter.setPen(QPen(qcolor_round_shi, 1, Qt::DashDotLine, Qt::RoundCap));

    size_t N = track_data_output_pack_list.size();
    for (size_t n = 0; n < N; ++n) {
        if(track_data_output_pack_list[n].raw_track_data_output_(3) == 0) { //说明是实点
            trackGUI = tools1.RadarPolarToGUI(track_data_output_pack_list[n].track_dataout_point_);
            x = static_cast<int>(trackGUI(0));
            y = static_cast<int>(trackGUI(1));
            painter.drawPoint(x,y);
        } else {
            QColor qcolor_round_bu(255,255,0);  //补点的颜色
            painter.setPen(QPen(qcolor_round_bu, 1, Qt::DashDotLine, Qt::RoundCap));
            trackGUI = tools1.RadarPolarToGUI(track_data_output_pack_list[n].track_dataout_point_);
            x = static_cast<int>(trackGUI(0));
            y = static_cast<int>(trackGUI(1));
            painter.drawPoint(x,y);
        }

    }
}



/**
 * @brief MainWindow::DrawScanningGUI
 * 说明：绘制界面的主要函数
 */
void MainWindow::DrawScanningGUI() {

    QPixmap pixmap(kRectangleWidth,kRectangleHigh);//构造一个长方形图片

    //设置背景颜色
    pixmap.fill(background_color);
    QPainter painter(&pixmap);

    //坐标轴、斜线、十字架，圆等等在此绘制
    MainInterface(painter);

    /**
      @brief:雷达的测量值在此显示：
    */

    //先将画布移动到圆心
    painter.translate(QPoint((kRectangleWidth-kRoundDiameter)/2+kRoundDiameter/2,(kRectangleHigh - kRoundDiameter)/2+kRoundDiameter/2));
    plotMeasurements(painter, measurement_pack_list);

    /**
     * @brief 最终的航迹信息在此显示
     */
    plotTracks(painter, track_data_output_pack_list);


    /**
      @brief:  动态扫描的部分在此显示
    */
    static double MyScanLine ;
    MyScanLine += static_cast<double>(2);
    if (MyScanLine >= 360.0) {
        MyScanLine = 0;
    }
    painter.translate(-kRoundDiameter/2,-kRoundDiameter/2);
    QConicalGradient gradient(kRoundDiameter/2,kRoundDiameter/2, MyScanLine);
    gradient.setColorAt(0.2,QColor(47,79,79, 100));
    gradient.setColorAt(0.5,QColor(47,79,79, 0));
    painter.setBrush(gradient);
    painter.setPen(QPen(Qt::NoPen));
    painter.drawPie(0, 0, kRoundDiameter,kRoundDiameter, MyScanLine * 16, 60* 16);


    //画图结束
    painter.end();
    pixmap.scaled(ui->label_main->size(), Qt::KeepAspectRatio);
    ui->label_main->setPixmap(pixmap);
   // ui->tab->resize(pixmap.size());
}

/**
 * @brief SetGUIStyle
 * 说明：本函数用于设置GUI的外观
 */
void MainWindow::SetGUIStyle() {

    ui->label_main->setScaledContents(true);  //按比例填充满整个label框
    ui->centralWidget->setWindowState(Qt::WindowMaximized);  //窗口最大化显示
    //设置tab_GUI的样式
    /*ui->centralWidget->setStyleSheet("QWidget{""background-color: #DC143C ;"  // 背景
                                        "border:1px solid red;"     //边框的颜色
                                        "}");*/

    //设置tab_GUI的样式
    ui->tab_GUI->setStyleSheet("QWidget{""background-color: #000000  ;"  // 背景
                                         "border:0px solid Teal;"     //边框的颜色
                                         "}");
    //设置tab_SET的样式
    ui->tab_SET->setStyleSheet("QWidget{""background-color: #008B8B ;"  // 背景
                               "border:2px solid Teal;"     //边框的颜色
                               "}");

}

/**
 * @brief check_files
 * @param in_file
 * @param in_name
 * @param out_file
 * @param out_name
 * 检查打开输入输出文件是否正确，到上线时，就不用啦！！
 */
void check_files(ifstream& in_file, string& in_name,
                 ofstream& out_file, string& out_name) {
  if (!in_file.is_open()) {
    cerr << "Cannot open input file: " << in_name << endl;
    exit(EXIT_FAILURE);
  }

  if (!out_file.is_open()) {
    cerr << "Cannot open output file: " << out_name << endl;
    exit(EXIT_FAILURE);
  }
}




/*************************第二部分：控制参数设置**********************/





