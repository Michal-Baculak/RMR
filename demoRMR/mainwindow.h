#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QMainWindow>
#include <QTimer>
#include <iostream>
// #include<arpa/inet.h>
// #include<unistd.h>
// #include<sys/socket.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <utility>
#include <vector>

// #include "ckobuki.h"
// #include "rplidar.h"

#include "robot.h"
#ifndef DISABLE_JOYSTICK
#include <QJoysticks.h>
#endif
namespace Ui {
class MainWindow;
}

/// toto je trieda s oknom.. ktora sa spusti ked sa spusti aplikacia.. su tu
/// vsetky gombiky a spustania...
class MainWindow : public QMainWindow {
  Q_OBJECT

public:
#ifndef DISABLE_OPENCV
  bool useCamera1;
  int actIndex;
  cv::Mat frame[3];
#endif

#ifndef DISABLE_SKELETON
  int updateSkeletonPicture;
  skeleton skeleJoints;
#endif
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();

  const double PIXELS_PER_METER = 100.0;
  private slots:
  void on_pushButton_9_clicked();

  void on_pushButton_2_clicked();

  void on_pushButton_3_clicked();

  void on_pushButton_6_clicked();

  void on_pushButton_5_clicked();

  void on_pushButton_4_clicked();

  void on_pushButton_clicked();

  int paintThisLidar(const std::vector<LaserData> &laserData);
#ifndef DISABLE_OPENCV
  int paintThisCamera(const cv::Mat &cameraData);
#endif
#ifndef DISABLE_SKELETON
  int paintThisSkeleton(const skeleton &skeledata);
#endif
  void on_IPComboBox_currentIndexChanged(int index);

  void on_pushButton_10_clicked();

  private:
  robot _robot;
  Ui::MainWindow *ui;
  int updateLaserPicture;
  std::vector<LaserData> copyOfLaserData;
  int datacounter;
  std::string ipaddress;
  QTimer *timer;
  double _setpointX = 1;
  double _setpointY = 0;

  void setSetpoint(double x, double y);

#ifndef DISABLE_JOYSTICK
  QJoysticks *instance;
#endif

  //--skuste tu nic nevymazat... pridavajte co chcete, ale pri odoberani by sa
  // mohol stat nejaky drobny problem, co bude vyhadzovat chyby
  void paintEvent(QPaintEvent *event); // Q_DECL_OVERRIDE;
  std::pair<double, double> widgetXYtoWorldXY(double x, double y);

  public slots:
  void setUiValues(double robotX, double robotY, double robotFi);

  protected:
  bool eventFilter(QObject *watched, QEvent *event) override;
};

#endif // MAINWINDOW_H
