/***************************************************************************
 **                                                                        **
 **  QCustomPlot, an easy to use, modern plotting widget for Qt            **
 **  Copyright (C) 2011-2015 Emanuel Eichhammer                            **
 **                                                                        **
 **  This program is free software: you can redistribute it and/or modify  **
 **  it under the terms of the GNU General Public License as published by  **
 **  the Free Software Foundation, either version 3 of the License, or     **
 **  (at your option) any later version.                                   **
 **                                                                        **
 **  This program is distributed in the hope that it will be useful,       **
 **  but WITHOUT ANY WARRANTY; without even the implied warranty of        **
 **  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         **
 **  GNU General Public License for more details.                          **
 **                                                                        **
 **  You should have received a copy of the GNU General Public License     **
 **  along with this program.  If not, see http://www.gnu.org/licenses/.   **
 **                                                                        **
 ****************************************************************************
 **           Author: Emanuel Eichhammer                                   **
 **  Website/Contact: http://www.qcustomplot.com/                          **
 **             Date: 25.04.15                                             **
 **          Version: 1.3.1                                                **
 ****************************************************************************/

/************************************************************************************************************
 **                                                                                                         **
 **  This is the example code for QCustomPlot.                                                              **
 **                                                                                                         **
 **  It demonstrates basic and some advanced capabilities of the widget. The interesting code is inside     **
 **  the "setup(...)Demo" functions of MainWindow.                                                          **
 **                                                                                                         **
 **  In order to see a demo in action, call the respective "setup(...)Demo" function inside the             **
 **  MainWindow constructor. Alternatively you may call setupDemo(i) where i is the index of the demo       **
 **  you want (for those, see MainWindow constructor comments). All other functions here are merely a       **
 **  way to easily create screenshots of all demos for the website. I.e. a timer is set to successively     **
 **  setup all the demos and make a screenshot of the window area and save it in the ./screenshots          **
 **  directory.                                                                                             **
 **                                                                                                         **
 *************************************************************************************************************/

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QtCore>
#include <QLabel>
#include <QCoreApplication>
#include <QDebug>
#include <QDesktopWidget>
#include <QScreen>
#include <QMessageBox>
#include <QMetaEnum>
#include "testalg.h"
#include <QSerialPort>
#include <math.h>
#include <stdint.h>
#include <QInputDialog>
#include "MatMath.h"
#include "Qrien_mag.h"
#include "Qrien_gyro.h"
#include "medianfilter.h"
#include "KF_IMU.h"
#include "Solver.h"
#include "calibrationdialog.h"
#include <cassert>
#include <cstring>
//#include <windows.h>
#include "StdAfx.h"
#define _CRT_SECURE_NO_DEPRECATE
//#pragma warning (disable : 4996)

#define BAUD_RATE       115200 //500000
QSerialPort *serial = NULL;

int compnr = 0;
int refreshnr = 5;
int numchange = 0;
int expectedNumberOfBytes = 20; // length of data
int ena_ini_plot = 0;
int ena_ses = 0;
int ind_ini = 0;
int cali_ini = 0;
int ena_ini2 = 0;
int start_record = 0;
int start_com = 0;
int plotcounter = 0;
int basize = 0;
uint8_t value[40];
uint8_t valueforplot[20];
int16_t acc16;
int16_t gyr16;
int16_t mag16;

int8_t header;
float accxp[10];
float accyp[10];
float acczp[10];
float gyrxp[10];
float gyryp[10];
float gyrzp[10];
float magxp[10];
float magyp[10];
float magzp[10];
float accxmmm;
float accymmm;
float acczmmm;
float gyrxmmm;
float gyrymmm;
float gyrzmmm;
float magxmmm;
float magymmm;
float magzmmm;

float BBmag[3] =
{ 0.3284, 0.1925, -0.0348 };
float MMmag[3][3] =
{ 2.2573, 0.5683, 0.3903, -0.5032, 2.5769, -0.2341, -0.0501, -0.0900, 2.1454 };
float BBacc[3] =
{ 0.1642, -0.0497, -0.0039 };
float MMacc[3][3] =
{ 0.9568, -0.3543, 0.0763, 0.2869, 0.9533, 0.0666, -0.0939, -0.0268, 0.9950 };
float BBgyr[3] =
{ 0.0154, 0.0508, -0.0165 };
float MMgyr[3][3] =
{ 1.1319, 0.3780, 0.1138, -0.3658, 1.1331, -0.1256, -0.1448, 0.0825, 1.1663 };

int start_plot = 0;
int start_new_plot = 0;
int receivedNumberOfBytes = 0;
int datashift = 0;
int num_graphic = 0;
QByteArray ba, bashort;
    QByteArray output;

int N_node = 2;
int N_measurement = 5;
uint8_t valuenew [7];
int nodeid [5*4];
int nodepoint = 0;
int dist [5*5*3];
int dispoint = 0;
int nodeflag = 0;
int distflag = 0;
uint8_t ack [5];

typedef signed short SensorData;
typedef unsigned char SensorByte;

struct Sensor
{
    SensorData x, y, z;
}__attribute__ ((__packed__));

struct ImuFrame
{
    SensorByte head;
    Sensor mag, acc, gyr; // mag.x is a counter!
    SensorByte dummy;
}__attribute__ ((__packed__));

SensorData convertAcc(SensorData value)
{
    SensorByte* meas = (SensorByte*) &value;
    return meas[0] << 8 | meas[1];
}

void convertFrame(ImuFrame* frame)
{
    frame->acc.x = convertAcc(frame->acc.x);
    frame->acc.y = convertAcc(frame->acc.y);
    frame->acc.z = convertAcc(frame->acc.z);
    frame->gyr.x = convertAcc(frame->gyr.x);
    frame->gyr.y = convertAcc(frame->gyr.y);
    frame->gyr.z = convertAcc(frame->gyr.z);
    frame->mag.x = convertAcc(frame->mag.x);
    frame->mag.y = convertAcc(frame->mag.y);
    frame->mag.z = convertAcc(frame->mag.z);
}

const SensorByte MARK_START = 170;

QVector<double> x_new_plot(2000), y_new_plot(2000), y1_new_plot(2000),
        y2_new_plot(2000), y3_new_plot(2000), y4_new_plot(2000), y5_new_plot(
                2000), y6_new_plot(2000), y7_new_plot(2000), y8_new_plot(2000);
QFile file("imudata.txt");
QTextStream out(&file);

//float d;
//float A[1617][12];
//int row = 0;
//int col = 0;
float sensordata[9];
float posiplot[3];
float veloplot[3];
float orienplot[3];
float acc_linear[3];

MainWindow::MainWindow(QWidget *parent) :
        QMainWindow(parent), ui(new Ui::MainWindow)
{
    /*
     FILE* f = fopen("C:\\Users\\Zhang\\Documents\\QTcreator project\\Serialrealtimeplot_old\\25.txt", "r");
     while (!feof(f)) {
     fscanf(f, "%f", &d);
     A[row][col] = d;
     col++;
     if (col == 12) {
     row++;
     col = 0;
     }
     }
     fclose(f);
     */
    Solverini();
    ui->setupUi(this);
    ui->label->setPixmap(QPixmap(":/myfiles/logosmall.png"));
}

MainWindow::~MainWindow()
{
    delete ui;
    if (serial)
    {
        serial->close();
        serial = NULL;
    }
    start_com = 0;
    cali_ini = 0;
}

void MainWindow::setupDemo()
{
    setupQuadraticDemo(ui->customPlot);
    ui->customPlot->replot();
}

void MainWindow::setupQuadraticDemo(QCustomPlot *customPlot)
{

#if QT_VERSION < QT_VERSION_CHECK(4, 7, 0)
    QMessageBox::critical(this, "", "You're using Qt < 4.7, the realtime data demo needs functions that are available with Qt 4.7 to work properly");
#endif

    //double key = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;

    customPlot->plotLayout()->clear(); // clear default axis rect so we can start from scratch
    QCPAxisRect *accAxisRect = new QCPAxisRect(customPlot, false);
    //accAxisRect->setupFullAxesBox(true);
    accAxisRect->addAxes(QCPAxis::atBottom | QCPAxis::atLeft);
    QCPLayoutGrid *subLayout = new QCPLayoutGrid;
    QCPLayoutGrid *subLayout1 = new QCPLayoutGrid;
    customPlot->plotLayout()->addElement(0, 0, accAxisRect); // insert axis rect in first row
    customPlot->plotLayout()->addElement(1, 0, subLayout); // sub layout in second row (grid layout will grow accordingly)
    customPlot->plotLayout()->addElement(2, 0, subLayout1); // sub layout in second row (grid layout will grow accordingly)

    QCPAxisRect *gyrAxisRect = new QCPAxisRect(customPlot, false); // false means to not setup default axes
    subLayout->addElement(0, 0, gyrAxisRect);
    gyrAxisRect->addAxes(QCPAxis::atBottom | QCPAxis::atLeft);

    QCPAxisRect *magAxisRect = new QCPAxisRect(customPlot, false); // false means to not setup default axes
    subLayout1->addElement(0, 0, magAxisRect);
    magAxisRect->addAxes(QCPAxis::atBottom | QCPAxis::atLeft);

    QCPMarginGroup *marginGroup = new QCPMarginGroup(customPlot);
    accAxisRect->setMarginGroup(QCP::msLeft | QCP::msRight, marginGroup);
    gyrAxisRect->setMarginGroup(QCP::msLeft | QCP::msRight, marginGroup);
    magAxisRect->setMarginGroup(QCP::msLeft | QCP::msRight, marginGroup);

    // move newly created axes on "axes" layer and grids on "grid" layer:
    foreach(QCPAxisRect * rect, customPlot->axisRects())
    {
        foreach(QCPAxis * axis, rect->axes())
        {
            axis->setLayer("axes");
            axis->grid()->setLayer("grid");
            axis->grid()->setVisible(false);
        }
    }

    accAxisRect->axis(QCPAxis::atLeft)->setLabel("acceleration [m/s^2]");
    accAxisRect->axis(QCPAxis::atBottom)->setLabel("sample number");
    accAxisRect->axis(QCPAxis::atLeft)->setRange(-60, 60);
    accAxisRect->axis(QCPAxis::atBottom)->setRange(0, 2000); //setRange(key+0.25, 8, Qt::AlignRight);//

    gyrAxisRect->axis(QCPAxis::atLeft)->setLabel("angular rate [deg/s] "); //("orientation [rad]");
    gyrAxisRect->axis(QCPAxis::atBottom)->setLabel("sample number");
    gyrAxisRect->axis(QCPAxis::atLeft)->setRange(-2200, 2200);
    gyrAxisRect->axis(QCPAxis::atBottom)->setRange(0, 2000); //setRange(key+0.25, 8, Qt::AlignRight);//setRange(0, 2000);

    magAxisRect->axis(QCPAxis::atLeft)->setLabel("normalized magnetic field"); //("acceleration in G [m/s^2]");
    magAxisRect->axis(QCPAxis::atBottom)->setLabel("sample number");
    magAxisRect->axis(QCPAxis::atLeft)->setRange(-3, 3);
    magAxisRect->axis(QCPAxis::atBottom)->setRange(0, 2000); //setRange(key+0.25, 8, Qt::AlignRight);//setRange(0, 2000);

    QCPLegend *accLegend = new QCPLegend;
    accAxisRect->insetLayout()->addElement(accLegend,
            Qt::AlignTop | Qt::AlignLeft);
    accLegend->setLayer("legend");
    customPlot->setAutoAddPlottableToLegend(false); // would add to the main legend (in the primary axis rect)

    QCPLegend *gyrLegend = new QCPLegend;
    gyrAxisRect->insetLayout()->addElement(gyrLegend,
            Qt::AlignTop | Qt::AlignLeft);
    gyrLegend->setLayer("legend");
    customPlot->setAutoAddPlottableToLegend(false); // would add to the main legend (in the primary axis rect)

    QCPLegend *magLegend = new QCPLegend;
    magAxisRect->insetLayout()->addElement(magLegend,
            Qt::AlignTop | Qt::AlignLeft);
    magLegend->setLayer("legend");
    customPlot->setAutoAddPlottableToLegend(false); // would add to the main legend (in the primary axis rect)

    QCPGraph *mainGraph1 = customPlot->addGraph(
            accAxisRect->axis(QCPAxis::atBottom),
            accAxisRect->axis(QCPAxis::atLeft));
    mainGraph1->setPen(QPen(Qt::blue));
    mainGraph1->setName("x-axis");
    QCPGraph *mainGraph2 = customPlot->addGraph(
            accAxisRect->axis(QCPAxis::atBottom),
            accAxisRect->axis(QCPAxis::atLeft));
    mainGraph2->setPen(QPen(Qt::red));
    mainGraph2->setName("y-axis");
    QCPGraph *mainGraph3 = customPlot->addGraph(
            accAxisRect->axis(QCPAxis::atBottom),
            accAxisRect->axis(QCPAxis::atLeft));
    mainGraph3->setPen(QPen(Qt::green));
    mainGraph3->setName("z-axis");
    accLegend->addItem(new QCPPlottableLegendItem(accLegend, mainGraph1));
    accLegend->addItem(new QCPPlottableLegendItem(accLegend, mainGraph2));
    accLegend->addItem(new QCPPlottableLegendItem(accLegend, mainGraph3));

    QCPGraph *mainGraph4 = customPlot->addGraph(
            gyrAxisRect->axis(QCPAxis::atBottom),
            gyrAxisRect->axis(QCPAxis::atLeft));
    mainGraph4->setPen(QPen(Qt::blue));
    mainGraph4->setName("x-axis");
    QCPGraph *mainGraph5 = customPlot->addGraph(
            gyrAxisRect->axis(QCPAxis::atBottom),
            gyrAxisRect->axis(QCPAxis::atLeft));
    mainGraph5->setPen(QPen(Qt::red));
    mainGraph5->setName("y-axis");
    QCPGraph *mainGraph6 = customPlot->addGraph(
            gyrAxisRect->axis(QCPAxis::atBottom),
            gyrAxisRect->axis(QCPAxis::atLeft));
    mainGraph6->setPen(QPen(Qt::green));
    mainGraph6->setName("z-axis");
    gyrLegend->addItem(new QCPPlottableLegendItem(gyrLegend, mainGraph4));
    gyrLegend->addItem(new QCPPlottableLegendItem(gyrLegend, mainGraph5));
    gyrLegend->addItem(new QCPPlottableLegendItem(gyrLegend, mainGraph6));

    QCPGraph *mainGraph7 = customPlot->addGraph(
            magAxisRect->axis(QCPAxis::atBottom),
            magAxisRect->axis(QCPAxis::atLeft));
    mainGraph7->setPen(QPen(Qt::blue));
    mainGraph7->setName("x-axis");
    QCPGraph *mainGraph8 = customPlot->addGraph(
            magAxisRect->axis(QCPAxis::atBottom),
            magAxisRect->axis(QCPAxis::atLeft));
    mainGraph8->setPen(QPen(Qt::red));
    mainGraph8->setName("y-axis");
    QCPGraph *mainGraph9 = customPlot->addGraph(
            magAxisRect->axis(QCPAxis::atBottom),
            magAxisRect->axis(QCPAxis::atLeft));
    mainGraph9->setPen(QPen(Qt::green));
    mainGraph9->setName("z-axis");
    //QCPGraph *mainGraph10 = customPlot->addGraph(magAxisRect->axis(QCPAxis::atBottom), magAxisRect->axis(QCPAxis::atLeft));
    //mainGraph10->setPen(QPen(Qt::black));
    //mainGraph10->setName("x+y-axis");
    magLegend->addItem(new QCPPlottableLegendItem(magLegend, mainGraph7));
    magLegend->addItem(new QCPPlottableLegendItem(magLegend, mainGraph8));
    magLegend->addItem(new QCPPlottableLegendItem(magLegend, mainGraph9));

    for (int i = 0; i < 2000; ++i)
    {
        x_new_plot[i] = i; // x goes from -1 to 1
    }

    //connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot1()));
    //connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlouwb()));
    //dataTimer.setSingleShot(false);
    //dataTimer.start(10); // Interval 0 means to refresh as fast as possible

    /*
     // setup a timer that repeatedly calls MainWindow::realtimeDataSlot:
     connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot1()));
     dataTimer.start(10); // Interval 0 means to refresh as fast as possible
     //connect(serial,SIGNAL(readyRead()),this,SLOT(realtimeDataSlot1()));
     */
}
/*
void MainWindow::realtimeDataSlotuwb()
{
    if (serial)
        receivedNumberOfBytes = serial->bytesAvailable();
    if (receivedNumberOfBytes > 0)
    {
        dataTimer.stop();

        QString ts = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz");

        ba.append(serial->readAll());
        //QString te = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz");
        //basize = ba.size()/20;


        for (int i = 11; i < ba.size();i++){
            valuenew[0] = (uint8_t) ba.at(i-11);
            valuenew[1] = (uint8_t) ba.at(i-10);
            valuenew[2] = (uint8_t) ba.at(i-9);
            valuenew[3] = (uint8_t) ba.at(i-8);

            if (valuenew[0] == 78 && valuenew[1] == 79 && valuenew[2] == 68 && valuenew[3] == 69){
                nodeid[nodepoint*4+0] = valuenew[6];
                nodeid[nodepoint*4+1] = valuenew[7];
                nodeid[nodepoint*4+2] = valuenew[8];
                nodeid[nodepoint*4+3] = valuenew[9];
                nodepoint = (nodepoint+1)%N_node;
            }
            /*
            if (valuenew[0] == 68 && valuenew[1] == 73 && valuenew[2] == 83 && valuenew[3] == 84){
                dist[]
                nodeid[nodepoint*4+0] = valuenew[6];
                nodeid[nodepoint*4+1] = valuenew[7];
                nodeid[nodepoint*4+2] = valuenew[8];
                nodeid[nodepoint*4+3] = valuenew[9];
                nodepoint = (nodepoint+1)%N_node;
            }
            */
/*
        }
      }
}
*/

int totalCounter = 0;
void MainWindow::realtimeDataSlot1()
{
    // calculate two new data points:
#if QT_VERSION < QT_VERSION_CHECK(4, 7, 0)
    QMessageBox::critical(this, "", "You're using Qt < 4.7, the realtime data demo needs functions that are available with Qt 4.7 to work properly");
#endif

    if (serial)
        receivedNumberOfBytes = serial->bytesAvailable();
    if (receivedNumberOfBytes > 0)
    {
        dataTimer.stop();

        QString ts = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz");

        ba.append(serial->readAll());
        //QString te = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz");
        //basize = ba.size()/20;

        if (ba.size() >= 2 * expectedNumberOfBytes && ena_ini_plot == 0)
        {
            for (int i = 0; i < 2 * expectedNumberOfBytes; i++)
            {
                value[i] = (uint8_t) ba.at(i);
                if ((i >= expectedNumberOfBytes) && (value[i] == 170)
                        && (value[i - expectedNumberOfBytes] == 170))
                {
                    int index_start = i - expectedNumberOfBytes;
                    ba.remove(0, index_start);
                    ena_ini_plot = 1;
                    break;
                }
            }
            if (ena_ini_plot == 0)
            {
                ba.remove(0, 2 * expectedNumberOfBytes);
            }

        }

        while (ena_ini_plot == 1 && ba.size() >= 2 * expectedNumberOfBytes)
        {
            if (ba.at(0) == -86 && ba.at(expectedNumberOfBytes) == -86)
            {
                for (int j = 0; j < expectedNumberOfBytes; j++)
                {
                    valueforplot[j] = ba.at(j);
                }
                ena_ses = 1;
                ba.remove(0, expectedNumberOfBytes);
            }
            else
            {
                for (int i = 0; i < 2 * expectedNumberOfBytes; i++)
                {
                    value[i] = (uint8_t) ba.at(i);
                    if ((i >= expectedNumberOfBytes) && (value[i] == 170)
                            && (value[i - expectedNumberOfBytes] == 170))
                    {
                        int index_start = i - expectedNumberOfBytes;
                        ba.remove(0, index_start);
                        out << "losing data" << "\n";

                        for (int j = 0; j < expectedNumberOfBytes; j++)
                        {
                            valueforplot[j] = ba.at(j);
                        }
                        ena_ses = 1;
                        break;
                    }
                }
                if (ena_ses == 0)
                {
                    ba.remove(0, 2 * expectedNumberOfBytes);
                    break;
                }
            }

            if (ena_ses == 1)
            {

                mag16 = (valueforplot[1] << 8) + valueforplot[2];
                magyp[start_plot] = -mag16; //13000;
                magyp[start_plot] = magyp[start_plot] / 13000 - BBmag[1];
                mag16 = (valueforplot[3] << 8) + valueforplot[4];
                magxp[start_plot] = -mag16; //13000;
                magxp[start_plot] = magxp[start_plot] / 13000 - BBmag[0];
                mag16 = (valueforplot[5] << 8) + valueforplot[6];
                magzp[start_plot] = -mag16; //13000;
                magzp[start_plot] = magzp[start_plot] / 13000 - BBmag[2];
                acc16 = (valueforplot[7] << 8) + valueforplot[8];
                accxp[start_plot] = acc16 * 0.00059875 * 2 - BBacc[0];
                acc16 = (valueforplot[9] << 8) + valueforplot[10];
                accyp[start_plot] = acc16 * 0.00059875 * 2 - BBacc[1];
                acc16 = (valueforplot[11] << 8) + valueforplot[12];
                acczp[start_plot] = acc16 * 0.00059875 * 2 - BBacc[2];
                gyr16 = (valueforplot[13] << 8) + valueforplot[14];
                gyrxp[start_plot] = gyr16 / 14.375 - BBgyr[0]; //*0.061035-BBgyr[0];
                gyr16 = (valueforplot[15] << 8) + valueforplot[16];
                gyryp[start_plot] = gyr16 / 14.375 - BBgyr[1]; //*0.061035-BBgyr[1];
                gyr16 = (valueforplot[17] << 8) + valueforplot[18];
                gyrzp[start_plot] = gyr16 / 14.375 - BBgyr[2]; //*0.061035-BBgyr[2];

                //ba.remove(0, expectedNumberOfBytes);

                magxmmm = magxp[start_plot] * MMmag[0][0]
                        + magyp[start_plot] * MMmag[0][1]
                        + magzp[start_plot] * MMmag[0][2];
                magymmm = magxp[start_plot] * MMmag[1][0]
                        + magyp[start_plot] * MMmag[1][1]
                        + magzp[start_plot] * MMmag[1][2];
                magzmmm = magxp[start_plot] * MMmag[2][0]
                        + magyp[start_plot] * MMmag[2][1]
                        + magzp[start_plot] * MMmag[2][2];
                magxp[start_plot] = magxmmm;
                magyp[start_plot] = magymmm;
                magzp[start_plot] = magzmmm;

                accxmmm = accxp[start_plot] * MMacc[0][0]
                        + accyp[start_plot] * MMacc[0][1]
                        + acczp[start_plot] * MMacc[0][2];
                accymmm = accxp[start_plot] * MMacc[1][0]
                        + accyp[start_plot] * MMacc[1][1]
                        + acczp[start_plot] * MMacc[1][2];
                acczmmm = accxp[start_plot] * MMacc[2][0]
                        + accyp[start_plot] * MMacc[2][1]
                        + acczp[start_plot] * MMacc[2][2];
                accxp[start_plot] = accxmmm;
                accyp[start_plot] = accymmm;
                acczp[start_plot] = acczmmm;

                gyrxmmm = gyrxp[start_plot] * MMgyr[0][0]
                        + gyryp[start_plot] * MMgyr[0][1]
                        + gyrzp[start_plot] * MMgyr[0][2];
                gyrymmm = gyryp[start_plot] * MMgyr[1][0]
                        + gyryp[start_plot] * MMgyr[1][1]
                        + gyrzp[start_plot] * MMgyr[1][2];
                gyrzmmm = gyrzp[start_plot] * MMgyr[2][0]
                        + gyryp[start_plot] * MMgyr[2][1]
                        + gyrzp[start_plot] * MMgyr[2][2];
                gyrxp[start_plot] = gyrxmmm;
                gyryp[start_plot] = gyrymmm;
                gyrzp[start_plot] = gyrzmmm;

                sensordata[0] = accxmmm;
                sensordata[1] = accymmm;
                sensordata[2] = acczmmm;
                sensordata[3] = gyrxmmm / 180 * (4 * atan(1.0));
                sensordata[4] = gyrymmm / 180 * (4 * atan(1.0));
                sensordata[5] = gyrzmmm / 180 * (4 * atan(1.0));
                sensordata[6] = magxmmm;
                sensordata[7] = magymmm;
                sensordata[8] = magzmmm;

                /*
                ComputeSolution((float*) sensordata, numchange,
                        (float*) veloplot, (float*) posiplot,
                        (float*) orienplot, (float*) acc_linear);
                if (numchange < 1617)
                {
                    numchange++;
                }
                */

                if (start_record == 1)
                {
                    /*
                     out << QString::number(accxp[start_plot], 'f', 6) << " " << " ";
                     out << QString::number(accyp[start_plot], 'f', 6) << " " << " ";
                     out << QString::number(acczp[start_plot], 'f', 6) << " " << " ";
                     out << QString::number(gyrxp[start_plot], 'f', 6) << " " << " ";
                     out << QString::number(gyryp[start_plot], 'f', 6) << " " << " ";
                     out << QString::number(gyrzp[start_plot], 'f', 6) << " " << " ";
                     out << QString::number(magxp[start_plot], 'f', 6) << " " << " ";
                     out << QString::number(magyp[start_plot], 'f', 6) << " " << " ";
                     out << QString::number(magzp[start_plot], 'f', 6) << "\n";
                     */
                    out << ts << " " << " ";
                    //out << te << " " << " ";
                    //out << basize << " " << " ";

                    out << QString::number(sensordata[0], 'f', 6) << " " << " ";
                    out << QString::number(sensordata[1], 'f', 6) << " " << " ";
                    out << QString::number(sensordata[2], 'f', 6) << " " << " ";
                    out
                            << QString::number(
                                    sensordata[3] / (4 * atan(1.0)) * 180, 'f',
                                    6) << " " << " ";
                    out
                            << QString::number(
                                    sensordata[4] / (4 * atan(1.0)) * 180, 'f',
                                    6) << " " << " ";
                    out
                            << QString::number(
                                    sensordata[5] / (4 * atan(1.0)) * 180, 'f',
                                    6) << " " << " ";


                    out << QString::number(sensordata[6], 'f', 6) << " " << " ";
                    out << QString::number(sensordata[7], 'f', 6) << " " << " ";
                    out << QString::number(sensordata[8], 'f', 6) << "\n";//" " << " ";

                     //out << QString::number(orienplot[0]/(4*atan(1.0))*180, 'f', 6) << " " << " ";
                     //out << QString::number(orienplot[1]/(4*atan(1.0))*180, 'f', 6) << " " << " ";
                     //out << QString::number(orienplot[2]/(4*atan(1.0))*180, 'f', 6) << "\n";

                    /*


                    out
                            << QString::number(
                                    orienplot[0] / (4 * atan(1.0)) * 180, 'f',
                                    6) << " " << " ";
                    out
                            << QString::number(
                                    orienplot[1] / (4 * atan(1.0)) * 180, 'f',
                                    6) << " " << " ";
                    out
                            << QString::number(
                                    orienplot[2] / (4 * atan(1.0)) * 180, 'f',
                                    6) << "\n"; //" " << " ";
                    */
                    //out << QString::number(posiplot[0], 'f', 6) << " " << " ";
                    //out << QString::number(posiplot[1], 'f', 6) << " " << " ";
                    //out << QString::number(posiplot[2], 'f', 6) << "\n";
                }
                y_new_plot.append(accxp[start_plot]);
                y_new_plot.remove(0);
                y1_new_plot.append(accyp[start_plot]);
                y1_new_plot.remove(0);
                y2_new_plot.append(acczp[start_plot]);
                y2_new_plot.remove(0);


                y3_new_plot.append(gyrxp[start_plot]);
                y3_new_plot.remove(0);
                y4_new_plot.append(gyryp[start_plot]);
                y4_new_plot.remove(0);
                y5_new_plot.append(gyrzp[start_plot]);
                y5_new_plot.remove(0);


                /*
                 y3_new_plot.append(orienplot[0]/(4*atan(1.0))*180);
                 y3_new_plot.remove(0);
                 y4_new_plot.append(orienplot[1]/(4*atan(1.0))*180);
                 y4_new_plot.remove(0);
                 y5_new_plot.append(orienplot[2]/(4*atan(1.0))*180);
                 y5_new_plot.remove(0);
                */


                y6_new_plot.append(magxp[start_plot]);
                y6_new_plot.remove(0);
                y7_new_plot.append(magyp[start_plot]);
                y7_new_plot.remove(0);
                y8_new_plot.append(magzp[start_plot]);
                y8_new_plot.remove(0);


                /*
                 y6_new_plot.append(acc_linear[0]);
                 y6_new_plot.remove(0);
                 y7_new_plot.append(acc_linear[1]);
                 y7_new_plot.remove(0);
                 y8_new_plot.append(acc_linear[2]);
                 y8_new_plot.remove(0);
                */
                //}
                //start_plot = (start_plot+1)%10;
                //start_new_plot = (start_new_plot+1)%5;
                //qDebug()<<header;
                if (start_new_plot == 0)
                {

                    ui->customPlot->graph(0)->setData(x_new_plot, y_new_plot);
                    ui->customPlot->graph(1)->setData(x_new_plot, y1_new_plot);
                    ui->customPlot->graph(2)->setData(x_new_plot, y2_new_plot);
                    ui->customPlot->graph(3)->setData(x_new_plot, y3_new_plot);
                    ui->customPlot->graph(4)->setData(x_new_plot, y4_new_plot);
                    ui->customPlot->graph(5)->setData(x_new_plot, y5_new_plot);
                    ui->customPlot->graph(6)->setData(x_new_plot, y6_new_plot);
                    ui->customPlot->graph(7)->setData(x_new_plot, y7_new_plot);
                    ui->customPlot->graph(8)->setData(x_new_plot, y8_new_plot);
                    /*

                     ui->customPlot->graph(0)->addData(key, accxp[start_plot]);
                     ui->customPlot->graph(1)->addData(key, accyp[start_plot]);
                     ui->customPlot->graph(2)->addData(key, acczp[start_plot]);
                     ui->customPlot->graph(3)->addData(key, orienplot[0]/(4*atan(1.0))*180);
                     ui->customPlot->graph(4)->addData(key, orienplot[1]/(4*atan(1.0))*180);
                     ui->customPlot->graph(5)->addData(key, orienplot[2]/(4*atan(1.0))*180);
                     ui->customPlot->graph(6)->addData(key, acc_linear[0]);
                     ui->customPlot->graph(7)->addData(key, acc_linear[1]);
                     ui->customPlot->graph(8)->addData(key, acc_linear[2]);
                     */

                    ui->customPlot->replot();

                }

                start_plot = (start_plot + 1) % 10;
                start_new_plot = (start_new_plot + 1) % refreshnr;
                ena_ses = 0;
                //qDebug()<<ui->customPlot->legend->itemCount();//valueforplot;
            }
            else
            {
                break;
            }

        }
        dataTimer.start();

    }
    //ui->textEdit->setText(ba);
    //qDebug()<<ba;

}

void MainWindow::on_pushButton_clicked()
{
    if (cali_ini == 1)
    {
        if (start_com == 0)
        {
            compnr = QInputDialog::getInt(this, "IMUplot",
                    "input comport number:");
            QString comps = "COM" + QString::number(compnr);
            //ui->Comportnumber->setText(QString::number(compnr));
            serial = new QSerialPort(this);
            serial->setPortName(comps);
            //serial->open(QIODevice::ReadWrite);
            //serial->setPortName("COM11");
            serial->setBaudRate(BAUD_RATE);
            //serial->setBaudRate(QSerialPort::Baud115200);
            serial->setDataBits(QSerialPort::Data8);
            serial->setParity(QSerialPort::EvenParity); //Even  //TODO, might need to be changed here, Yitong
            serial->setStopBits(QSerialPort::OneStop);
            serial->setFlowControl((QSerialPort::NoFlowControl));   //TODO, check parameters here, Yitong
            if(serial->open(QIODevice::ReadWrite)){
               //serial->write("ok*");
               start_com = 1;
               setGeometry(800, 250, 810, 640);
               setupDemo();

              //start_record = 1;
              // output = "find 0x310F, 5\r\n";
              // serial->write(output);
              // serial->flush();
              // serial->waitForBytesWritten(100000);
              // ba.append(serial->readAll());
              /*
               out << "   time stamp     "
                   << "  Dist of 0x1C1C    "                //TODO, check if it is needed here, Yitong
                   << "  Dist of 0x2020    "
                   << "  Dist of 0x3E3E    "
                   << "  Dist of 0x4141    "
                   << "  Dist of 0x5A5A    "
                   << "\n";
            */
            }
            else
            {
               QMessageBox msgBox;
               msgBox.setWindowTitle("Warning!");
               msgBox.setText("Com is not open!!");
               msgBox.setStandardButtons(QMessageBox::Ok);
               msgBox.exec();
            }
        }
        else if (start_com == 1)
        {
            QMessageBox msgBox;
            msgBox.setWindowTitle("Warning!");
            msgBox.setText("Com is already open!!");
            msgBox.setStandardButtons(QMessageBox::Ok);
            msgBox.exec();
        }
    }
    else if (cali_ini == 0)
    {
        QMessageBox msgBox;
        msgBox.setWindowTitle("Warning!");
        msgBox.setText("Calibration parameter must be given firstly!!");
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
    }
}

void MainWindow::on_recordstart_clicked()
{
    if (start_com == 1)
    {
        if (start_record == 0)
        {
            file.open(QIODevice::WriteOnly | QIODevice::Text);
            //dmp = fopen("filedmp.dat", "wb");
            ui->recordstart->setText("stop record");
            start_record = 1;
            out << "   time stamp start       "
                 //<< "  time stamp stop   "
                    //<< "  size of package received   "
                    << "  acceleration x-y-z [m/s/s]   "
                    << "  angular rate x-y-z [deg/s]   "
                    << "    magnetic field x-y-z     "
                    << "\n";

             out << "   !!!!!0000000000 \n";         // TODO,delete. here is for debugging, Yitong

                    //<< "    orientation roll-pitch-yaw [deg]" << "\n";
        }
        else if (start_record == 1)
        {
            QString IMUfilename =
                    QFileDialog::getSaveFileName(this,
                            tr("Choose file to save"), "/home",
                            "Text file (*.txt);;Excel file (*.xls);;Log file (*.log);;Data file (*.dat);;All files (*.*)");
            /*
             QString IMUfilenamem = QInputDialog::getText(this ,"IMUplot","input file name:");
             QInputDialog::
             QString IMUfilename = QString("%1%2").arg(IMUfilenamem).arg(".txt");
             */
            if (!IMUfilename.isNull())
            {

                if (!IMUfilename.isEmpty())
                {

                    QFile filenewm(IMUfilename);
                    filenewm.remove(IMUfilename);
                    filenewm.close();
                }
                QFile filenew(IMUfilename);
                file.copy(filenew.fileName());
                start_record = 0;
                file.close();
                //if (dmp)
                //    fclose(dmp);
                filenew.close();
                ui->recordstart->setText("start record");
            }

        }
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setWindowTitle("Warning!");
        msgBox.setText("Com is not open!!");
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
    }

}
/*
 void MainWindow::on_recordstop_clicked()
 {
 start_record = 0;
 QString IMUfilenamem = QInputDialog::getText(this ,"IMUplot","input file name:");
 QString IMUfilename = QString("%1%2").arg(IMUfilenamem).arg(".txt");
 QFile filenew(IMUfilename);
 file.copy(filenew.fileName());
 file.close();
 filenew.close();
 }
 */
/*
 void MainWindow::on_record_clicked()
 {
 if (ui->record->isChecked()){
 file.open(QIODevice::WriteOnly | QIODevice::Text);
 start_record = 1;
 }
 if (!ui->record->isChecked()){
 start_record = 0;
 QString IMUfilenamem = QInputDialog::getText(this ,"IMUplot","input file name:");
 QString IMUfilename = QString("%1%2").arg(IMUfilenamem).arg(".txt");
 QFile filenew(IMUfilename);
 file.copy(filenew.fileName());
 file.close();
 filenew.close();
 }

 }
 */

void MainWindow::on_calibration_clicked()
{
    calibrationDialog inputwindow;
    inputwindow.setModal(true);
    //inputwindow.exec();
    if (cali_ini == 0)
    {
        if (inputwindow.exec() == QDialog::Accepted)
        {
            cali_ini = 1;
            inputwindow.readinput(BBmag, MMmag, BBacc, MMacc, BBgyr, MMgyr);
        }
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setWindowTitle("Warning!");
        msgBox.setText("Calibration parameter has already been given!!");
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
    }
}

void MainWindow::on_pushButton_displayspeed_clicked()
{
    refreshnr = QInputDialog::getInt(this, "IMUplot",
            "input display refresh speed parameter:",7,1,15);

}

void MainWindow::on_getdist_clicked()
{
    if (start_com == 1 && serial->isWritable())
    {
        QByteArray output;

        ba.clear();

        //output = "find 0x310F, 1\r\n";
        output = "find 0x310D, 1\r\n";  // changed by Yitong, 30,May,2017
        serial->write(output);
        serial->flush();
        serial->waitForBytesWritten(1000); //TODO: 1/frequency to be changed. Yitong

        ba.append(serial->readAll());
        QString ts = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz");

        //out << ts << " " << "\n";

        //std::string message(usb_data.constData(), usb_data.length());
        //QString qmessage = QString::fromStdString(message);

        if (ba.size()>7){
            for (int i = 0; i < 7;i++){
                valuenew[i] = (uint8_t) ba.at(i);
            }
            if (valuenew[0] == 82 && valuenew[1] == 101 && valuenew[2] == 99 && valuenew[3] == 101 && valuenew[4] == 105 && valuenew[5] == 118 && valuenew[6] == 101){
                //out << ts << " " << "\n";
                ba = ba.append(ts);
                ba = ba.append(QString("    !!!!!111111111111")); // TODO,delete. here is for debugging, Yitong
                ba = ba.append(QString("\n"));
                file.write(ba);
                //out << "\n";
            }
        }else{          // TODO, delete. this else is added by Yitong for debugging, no use at all.Yitong
                        // in 'home.txt' the string in this else{} is printed out
            ba = ba.append(ts);
            ba = ba.append(QString("    !!!!!2222222222")); // TODO,delete. here is for debugging, Yitong
            ba = ba.append(QString("\n"));
            file.write(ba);
        }

        /*for (int i = 0; i < ba.size();i++){
            valuenew[0] = (uint8_t) ba.at(i-11);
            valuenew[1] = (uint8_t) ba.at(i-10);
            valuenew[2] = (uint8_t) ba.at(i-9);
            valuenew[3] = (uint8_t) ba.at(i-8);
            valuenew[4] = (uint8_t) ba.at(i-7);
            valuenew[5] = (uint8_t) ba.at(i-6);
            valuenew[6] = (uint8_t) ba.at(i-5);
            valuenew[7] = (uint8_t) ba.at(i-4);
            valuenew[8] = (uint8_t) ba.at(i-3);
            valuenew[9] = (uint8_t) ba.at(i-2);
            valuenew[10] = (uint8_t) ba.at(i-1);
            valuenew[11] = (uint8_t) ba.at(i);

            if (valuenew[0] == 78 && valuenew[1] == 79 && valuenew[2] == 68 && valuenew[3] == 69){
                nodeid[nodepoint*4+0] = valuenew[8];
                nodeid[nodepoint*4+1] = valuenew[9];
                nodeid[nodepoint*4+2] = valuenew[10];
                nodeid[nodepoint*4+3] = valuenew[11];
                nodeflag = 1;
            }

            if (nodeflag == 1 && valuenew[0] == 68 && valuenew[1] == 73 && valuenew[2] == 83 && valuenew[3] == 84){
                dist[nodepoint*3*5+dispoint*3+0] = valuenew[6];
                dist[nodepoint*3*5+dispoint*3+1] = valuenew[7];
                dist[nodepoint*3*5+dispoint*3+2] = valuenew[8];
                if (dispoint == 5){
                    distflag = 1;
                    nodeflag = 0;
                }
                dispoint = (dispoint+1)%N_measurement;

            }

            if (distflag == 1){
                nodepoint = (nodepoint+1)%N_node;
                distflag = 0;
            }

        }
*/

        // make sure CR + LF
        // write: setNUMBER:"number of rx"
        // write: setNODES: [0x...., 0x...., 0x...., ...] % id of the node

        // write: find 0x310F % get the distance from rx to tag 0x310F
        // write: find 0x310F, "number of the measurement" %
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setWindowTitle("Warning!");
        msgBox.setText("Com is not open!!");
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
    }
}

void MainWindow::on_setnodeno_clicked()
{
    ba.clear();

    output = "setNUMBER: 5\r\n";         //NODES_NUMBER, Yitong
    serial->write(output);
    serial->flush();

    while(!serial->isWritable()){
        serial->waitForBytesWritten(1000);
    }
    ba.append(serial->readAll());
    if (ba.size()==50){
        for(int i = 0; i < 5;i++){
            ack[i] = (uint8_t) ba.at(i);
        }
        if(ack[0] == 87 && ack[1] == 114 && ack[2] == 105 && ack[3] == 116 && ack[4] == 101){
           ui->setnodeno->setText("done");
        }
    }
}

void MainWindow::on_setnodeID_clicked()
{
    ba.clear();
    output = "setNODES: [0x1C1C, 0x2020, 0x3E3E, 0x4141, 0x5A5A]\r\n"; //NODES_ID, Yitong
    //output = "setNODES: [0x2020, 0x4141]\r\n";
    serial->write(output);
    serial->flush();
    while(!serial->isWritable()){
        serial->waitForBytesWritten(1000);
    }
    ba.append(serial->readAll());

    if (ba.size()==5){
        for(int i = 0; i < ba.size();i++){
            ack[i] = (uint8_t) ba.at(i);
        }
        if(ack[0] == 65 && ack[1] == 67 && ack[2] == 75 && ack[3] == 13 && ack[4] == 10){
           file.open(QIODevice::WriteOnly| QIODevice::Text);
           //QString ts = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz");
           //out << ts << " " << "\n";
           ui->setnodeID->setText("done");
        }
    }
}

void MainWindow::on_stopgetdist_clicked()
{

}
