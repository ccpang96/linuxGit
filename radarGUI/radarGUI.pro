#-------------------------------------------------
#
# Project created by QtCreator 2019-09-18T07:54:48
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = radarGUI
TEMPLATE = app  #用于构建应用程序的Makefile

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS  #qmake添加作为编译器C预处理器宏

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

INCLUDEPATH += \
    src/Eigen

CONFIG += c++11#

SOURCES += \
    src/main.cpp \
    src/mainwindow.cpp \
    src/show.cpp \
    src/FusionEKF.cpp \
    src/kalman_filter.cpp \
    src/tools.cpp \
    src/track.cpp \
    src/control.cpp

HEADERS += \
    include/radarGUI/mainwindow.h \
    include/radarGUI/show.h \
    include/radarGUI/FusionEKF.h \
    include/radarGUI/kalman_filter.h \
    include/radarGUI/measurement_package.h \
    include/radarGUI/tools.h \
    include/radarGUI/tracking.h \
    include/radarGUI/ground_truth_package.h \
    include/radarGUI/track_package.h \
    include/radarGUI/track.h \
    include/radarGUI/control.h

FORMS += \
    mainwindow.ui

LIBS += -lshapelib

#下面这个是默认的部署命令
# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

#解决中文乱码问题
QMAKE_CXXFLAGS += /source-charset:utf-8 /execution-charset:utf-8
