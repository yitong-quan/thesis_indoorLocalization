#-------------------------------------------------
#
# Project created by QtCreator 2015-05-27T15:29:52
#
#-------------------------------------------------

QT       += core gui serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = Serialrealtimeplotold
TEMPLATE = app
RC_FILE = 	default.rc


SOURCES += main.cpp\
        mainwindow.cpp \
    qcustomplot.cpp \
    BW_LP_2.cpp \
    KF_IMU.cpp \
    MatMath.cpp \
    medianfilter.cpp \
    Qrien_gyro.cpp \
    Qrien_mag.cpp \
    Solver.cpp \
    calibrationdialog.cpp

HEADERS  += mainwindow.h \
    qcustomplot.h \
    BW_LP_2.h \
    KF_IMU.h \
    MatMath.h \
    medianfilter.h \
    Qrien_gyro.h \
    Qrien_mag.h \
    Solver.h \
    StdAfx.h \
    calibrationdialog.h

FORMS    += mainwindow.ui \
    calibrationdialog.ui

RESOURCES += \
    myresouces.qrc
