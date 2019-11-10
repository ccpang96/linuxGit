#include "include/radarGUI/mainwindow.h"
#include <QApplication>
#include "Eigen/Dense"
#include "QStyleFactory"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    a.setStyle(QStyleFactory::create("Fusion"));
    //a.setStyleSheet(QString::fromUtf8("border:1px solid red"));
    MainWindow w;
    //w.setWindowFlags(w.windowFlags()& ~Qt::WindowMaximizeButtonHint&  ~Qt::WindowMinimizeButtonHint);  //去除最小化和最大化图标，为了偷懒！！！
    w.show();

    return a.exec();
}
