#ifndef INCLUDE_RADARGUI_MAINWINDOW_H
#define INCLUDE_RADARGUI_MAINWINDOW_H

#include <QMainWindow>
#include <QMessageBox>
#include <QDateTime>
#include <QTimer>
#include <QPoint>
#include <QRectF>
#include <QPixmap>
#include <QPainter>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include "Dense"
#include "QDebug"
#include "QBitmap"
#include <vector>
#include <stdlib.h>
#include "ui_mainwindow.h"
#include "include/radarGUI/measurement_package.h"
#include "include/radarGUI/track_package.h"

#include "include/radarGUI/show.h"
#include "include/radarGUI/FusionEKF.h"
#include "include/radarGUI/ground_truth_package.h"
#include "include/radarGUI/tools.h"
#include "include/radarGUI/track.h"

//常量区对象
/**
 * @brief 这三个参数如何设置：
 * 因为圆外还要绘制刻度，所以圆的直径应该小于正方形的变长
 * 还有一个是tab_widget的大小600 *600
 * label_main（expending的）上有一个pixmap的大小应该和tab_widget相等。
 * 圆形的大小：525*525 要比pixmap的边长小75
 *
 */
const int kRectangleHigh = 700;   //常量对象：整体长方形的高度
const int kRectangleWidth = 1024; //整体长方形的宽度
const int kRoundDiameter = 700 - 75;   //中心显示界面直径

//static QColor background_color(47,79,79);  //设置GUI的背景颜色
static QColor background_color(0,0,0); //深蓝色
using namespace std;


//静态全局变量：只在mainwindow.cpp内有效
static vector<vector<MeasurementPackage>>data_list; //对方传过来的数据包 1s中70个
static vector<MeasurementPackage> measurement_pack_list ; //经过CFAR后的数据,70个包中的1个
static vector<TrackDataOutputPackage> track_data_output_pack_list;  //本批数据处理完成后，输出的航迹信息
static vector<TempPointPackage> temp_point_pack_list; //临时点迹数据
static vector<TrustTrackPackage> trust_track_pack_list; //可靠航迹数据
static double  time_accumulate = 0.0; //积累时间
static int number_of_track = 0; //航迹总数




//从本地读取的目标跟踪函数
//int object_track();
//目标跟踪函数
//void objectTrack();




//画测量值
void plotMeasurements(QPainter &painter, const vector<MeasurementPackage> &measurement_pack_list);
//画航迹信息
void plotTracks(QPainter &painter, const vector<TrackDataOutputPackage> &track_data_output_pack_list);

//在类中只是作为友元的声明，在类外还是要重新声明
void check_files(ifstream& in_file, string& in_name,
                 ofstream& out_file, string& out_name);

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

    friend void  MainInterface(QPainter &painter);
    friend void check_files(ifstream& in_file, string& in_name,
                     ofstream& out_file, string& out_name);
    friend int object_track();

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void SetGUIStyle();
private:
    Ui::MainWindow *ui;

private slots:
    void DrawScanningGUI();
};

#endif // INCLUDE_RADARGUI_MAINWINDOW_H
