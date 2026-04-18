#include "mainwindow.h"
#include <QEvent>
#include <QMouseEvent>
#include <QPainter>
#include "ui_mainwindow.h"
#include <math.h>

///Lukrativne ID: Baculak_Ondruska


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

    //tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
    // ipaddress
    // = "192.168.1.11"; //192.168.1.11toto je na niektory realny robot.. na lokal budete davat "127.0.0.1"
    on_IPComboBox_currentIndexChanged(0);

    ui->setupUi(this);
    datacounter=0;
#ifndef DISABLE_OPENCV
    actIndex=-1;
    useCamera1=false;

#endif


    datacounter=0;
    ui->widget->installEventFilter(this);

    _robot.path_tracker.setSetpoint(_setpointX, _setpointY);
}

MainWindow::~MainWindow()
{
    delete ui;
}

QPointF MainWindow::globalToRobotFrame(const QPointF &globalPoint) const
{
    double dx = globalPoint.x() - _robot.odom.getX();
    double dy = globalPoint.y() - _robot.odom.getY();

    // add pi/2 since positive X is displayed UP not left
    double rot = -_robot.odom.getRot() + PI / 2;

    double c = std::cos(rot);
    double s = std::sin(rot);

    double localX = dx * c - dy * s;
    double localY = dx * s + dy * c;

    return QPointF(localX, localY);
}

std::vector<QPointF> MainWindow::setpointsToPoints() const
{
    std::vector<QPointF> points;

    const auto &setpoints = _robot.path_tracker.getSetpoints();
    points.reserve(setpoints.size());

    for (const auto &sp : setpoints) {
        QPointF globalPoint(sp.x, sp.y);
        points.push_back(globalToRobotFrame(globalPoint));
    }

    return points;
}

std::vector<QPointF> MainWindow::laserDataToPoints() const
{
    std::vector<QPointF> points;
    points.reserve(copyOfLaserData.size());

    for (const auto &scan : copyOfLaserData) {
        double dist = (scan.scanDistance / 1000.0); // mm -> meters

        double angleRad = (360.0 - scan.scanAngle) * PI / 180.0;

        double x = -dist * std::sin(angleRad);
        double y = dist * std::cos(angleRad);

        points.emplace_back(x, y);
    }

    return points;
}

void MainWindow::drawPoints(QPainter &painter,
                            const QRect &rect,
                            const std::vector<QPointF> &points,
                            bool alreadyLocal)
{
    int cx = rect.width() / 2 + rect.topLeft().x();
    int cy = rect.height() / 2 + rect.topLeft().y();

    for (const auto &p : points) {
        QPointF point = alreadyLocal ? p : globalToRobotFrame(p);

        int xp = cx + point.x() * PIXELS_PER_METER;
        int yp = cy - point.y() * PIXELS_PER_METER;

        if (rect.contains(xp, yp))
            painter.drawEllipse(QPoint(xp, yp), 3, 3);
    }
}

void MainWindow::paintVFH(QPainter &painter, QPen pero, QRect rect)
{
    int cx = rect.width() / 2 + rect.topLeft().x();
    int cy = rect.height() / 2 + rect.topLeft().y();
    int r_inner = 30;
    int r_outer = 60;

    int numSectors = (int) _lastMHist.size();
    double sectorDeg = 360.0 / numSectors;

    for (int k = 0; k < numSectors; ++k) {
        double startDeg = 90.0 + k * sectorDeg;

        QColor color = (_lastMHist[k] == 0) ? QColor(0, 255, 0, 80) : QColor(255, 0, 0, 80);

        painter.setBrush(color);
        pero.setColor(color.darker(150));
        pero.setWidth(1);
        painter.setPen(pero);

        QRect pieRect(cx - r_outer, cy - r_outer, 2 * r_outer, 2 * r_outer);
        painter.drawPie(pieRect, (int) (startDeg * 16), (int) (sectorDeg * 16));
    }

    painter.setBrush(Qt::black);
    pero.setColor(Qt::green);
    pero.setWidth(3);
    painter.setPen(pero);
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    ///prekreslujem obrazovku len vtedy, ked viem ze mam nove data. paintevent sa
    /// moze pochopitelne zavolat aj z inych dovodov, napriklad zmena velkosti okna
    painter.setBrush(Qt::black);//cierna farba pozadia(pouziva sa ako fill pre napriklad funkciu drawRect)
    QPen pero;
    pero.setStyle(Qt::SolidLine);//styl pera - plna ciara
    pero.setWidth(3);//hrubka pera -3pixely
    pero.setColor(Qt::green);//farba je zelena
    QRect rect;
    rect= ui->widget->geometry();//ziskate porametre stvorca,do ktoreho chcete kreslit
    rect.translate(0,15);
    painter.drawRect(rect);
#ifndef DISABLE_OPENCV
    if(useCamera1==true && actIndex>-1)/// ak zobrazujem data z kamery a aspon niektory frame vo vectore je naplneny
    {
        std::cout<<actIndex<<std::endl;
        QImage image = QImage((uchar*)frame[actIndex].data, frame[actIndex].cols, frame[actIndex].rows, frame[actIndex].step, QImage::Format_RGB888  );//kopirovanie cvmat do qimage
        painter.drawImage(rect,image.rgbSwapped());
    }
    else
#endif
    {
        if (updateLaserPicture != 1) ///ak mam nove data z lidaru
        {
            return;
        }
        updateLaserPicture = 0;

        pero.setColor(Qt::red); //farba je zelena
        painter.setPen(pero);
        painter.drawEllipse(QPoint(rect.width() / 2 + rect.topLeft().x(),
                                   rect.height() / 2 + rect.topLeft().y()),
                            15,
                            15);

        if (!_lastMHist.empty()) {
            paintVFH(painter, pero, rect);
        }
        painter.drawLine(QPoint(rect.width() / 2 + rect.topLeft().x(),
                                rect.height() / 2 + rect.topLeft().y()),
                         QPoint(rect.width() / 2 + rect.topLeft().x(),
                                rect.height() / 2 + rect.topLeft().y() - 15));
        pero.setColor(Qt::green); //farba je zelena
        painter.setPen(pero);

        // laser data drawing
        auto laserPoints = laserDataToPoints();
        drawPoints(painter, rect, laserPoints, true);

        // setpoint drawing
        auto setpointPoints = setpointsToPoints();

        painter.setPen(Qt::blue);
        drawPoints(painter, rect, setpointPoints, true);
    }
#ifndef DISABLE_SKELETON
    if(updateSkeletonPicture==1 )
    {
        painter.setPen(Qt::red);
        for(int i=0;i<75;i++)
        {
            int xp=rect.width()-rect.width() * skeleJoints.joints[i].x+rect.topLeft().x();
            int yp= (rect.height() *skeleJoints.joints[i].y)+rect.topLeft().y();
            if(rect.contains(xp,yp))
                painter.drawEllipse(QPoint(xp, yp),2,2);
        }
    }
#endif
}


/// toto je slot. niekde v kode existuje signal, ktory je prepojeny. pouziva sa napriklad (v tomto pripade) ak chcete dostat data z jedneho vlakna (robot) do ineho (ui)
/// prepojenie signal slot je vo funkcii  on_pushButton_9_clicked
void  MainWindow::setUiValues(double robotX,double robotY,double robotFi, double omega, double v)
{
    ui->lineEdit_2->setText(QString::number(robotX));
    ui->lineEdit_3->setText(QString::number(robotY));
    ui->lineEdit_4->setText(QString::number(robotFi));
    ui->lineEdit_6->setText(QString::number(omega));
    ui->lineEdit_5->setText(QString::number(v));

    //_lastMHist = _robot.nav.getLastMHist();
}


void MainWindow::on_pushButton_9_clicked() //start button
{
    //ziskanie joystickov


    //tu sa nastartuju vlakna ktore citaju data z lidaru a robota



    connect(&_robot,SIGNAL(publishPosition(double,double,double,double,double)),this,SLOT(setUiValues(double,double,double,double,double)));
    connect(&_robot,SIGNAL(publishLidar(const std::vector<LaserData> &)),this,SLOT(paintThisLidar(const std::vector<LaserData> &)));
    connect(&_robot, SIGNAL(publishHistogram(const std::vector<int>&)), this, SLOT(onHistogramUpdated(const std::vector<int>&)));
#ifndef DISABLE_OPENCV
    connect(&_robot,SIGNAL(publishCamera(const cv::Mat &)),this,SLOT(paintThisCamera(const cv::Mat &)));
#endif
#ifndef DISABLE_SKELETON
    connect(&_robot,SIGNAL(publishSkeleton(const skeleton &)),this,SLOT(paintThisSkeleton(const skeleton &)));
#endif

    _robot.initAndStartRobot(ipaddress);

    #ifndef DISABLE_JOYSTICK
        instance = QJoysticks::getInstance();
    /// prepojenie joysticku s jeho callbackom... zas cez lambdu. neviem ci som to niekde spominal,ale lambdy su super. okrem toho mam este rad ternarne operatory a spolocneske hry ale to tiez nikoho nezaujima
    /// co vas vlastne zaujima? citanie komentov asi nie, inak by ste citali toto a ze tu je blbosti
    connect(
                instance, &QJoysticks::axisChanged,
                [this]( const int js, const int axis, const qreal value) {
        double forw=0, rot=0;
        if(/*js==0 &&*/ axis==1){forw=-value*300;}
        if(/*js==0 &&*/ axis==0){rot=-value*(3.14159/2.0);}
        this->_robot.setSpeedVal(forw,rot);
    }
    );
#endif
}
void MainWindow::onHistogramUpdated(const std::vector<int>& mHist)
{
    _lastMHist = mHist;
}

void MainWindow::on_pushButton_2_clicked() //forward
{
    //pohyb dopredu
    _robot.setSpeed(500,0);

}

void MainWindow::on_pushButton_3_clicked() //back
{
    _robot.setSpeed(-250,0);

}

void MainWindow::on_pushButton_6_clicked() //left
{
    _robot.setSpeed(0,3.14159/2);

}

void MainWindow::on_pushButton_5_clicked()//right
{
    _robot.setSpeed(0,-3.14159/2);

}

void MainWindow::on_pushButton_4_clicked() //stop
{
    _robot.setSpeed(0,0);

}




void MainWindow::on_pushButton_clicked()
{
#ifndef DISABLE_OPENCV
    if(useCamera1==true)
    {
        useCamera1=false;

        ui->pushButton->setText("use camera");
    }
    else
    {
        useCamera1=true;

        ui->pushButton->setText("use laser");
    }
#endif
}





int MainWindow::paintThisLidar(const std::vector<LaserData> &laserData)
{
    copyOfLaserData=laserData;
    //memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));
    updateLaserPicture = 1;

    update();
    return 0;
}

#ifndef DISABLE_OPENCV

///toto je calback na data z kamery, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa ked dojdu nove data z kamery
int MainWindow::paintThisCamera(const cv::Mat &cameraData)
{

    cameraData.copyTo(frame[(actIndex+1)%3]);//kopirujem do nasej strukury
    actIndex=(actIndex+1)%3;//aktualizujem kde je nova fotka


    updateLaserPicture=1;

    return 0;
}
#endif

#ifndef DISABLE_SKELETON
int MainWindow::paintThisSkeleton(const skeleton &skeledata)
{
    memcpy(&skeleJoints,&skeledata,sizeof(skeleton));

    updateSkeletonPicture=1;
    return 0;
}
#endif

void MainWindow::on_IPComboBox_currentIndexChanged(int index)
{
    std::cout << "new index of IP is selected: " << index << std::endl;
    switch (index) {
    case 0:
        ipaddress = "127.0.0.1";
        break;
    case 1:
        ipaddress = "192.168.1.11";
        break;
    }
    std::cout << "ipaddress is now: " << ipaddress << std::endl;
}
bool MainWindow::eventFilter(QObject *watched, QEvent *event)
{
    if (watched == ui->widget && event->type() == QEvent::MouseButtonPress) {
        QMouseEvent *mouseEvent = static_cast<QMouseEvent *>(event);
        if (mouseEvent->button() == Qt::LeftButton) {
            std::cout << "Widget left-clicked at position: " << mouseEvent->pos().x() << ","
                      << mouseEvent->pos().y() << std::endl;
            auto realPos = widgetXYtoWorldXY(mouseEvent->pos().x(), mouseEvent->pos().y());
            setSetpoint(realPos.first, realPos.second);
            std::cout << "real position: (" << _setpointX << ", " << _setpointY << ")" << std::endl;
        }
    }
    return QMainWindow::eventFilter(watched, event);
}

std::pair<double, double> MainWindow::widgetXYtoWorldXY(double x, double y)
{
    // get widget geometry
    QRect rect = ui->widget->geometry();
    int widgetCenterX = rect.width() / 2;
    int widgetCenterY = rect.height() / 2;

    // transform y to start from bottom-left:
    y = rect.height() - y;
    int dx = x - widgetCenterX;
    int dy = y - widgetCenterY;
    double r = sqrt(dx * dx + dy * dy);
    double phi = atan2(dy, dx);

    //convert pixel values to real values:
    phi += _robot.odom.getRot() - PI / 2;
    r /= PIXELS_PER_METER; //random val used by everyone

    double deltaX = r * cos(phi);
    double deltaY = r * sin(phi);

    double realX = _robot.odom.getX() + deltaX;
    double realY = _robot.odom.getY() + deltaY;
    return std::pair<double, double>(realX, realY);
}

void MainWindow::setSetpoint(double x, double y)
{
    _setpointX = x;
    _setpointY = y;
    std::string labelString = "Setpoint: (" + std::to_string(x) + ", " + std::to_string(y) + ")";
    ui->setpointLabel->setText(QString(labelString.c_str()));
}

void MainWindow::on_pushButton_10_clicked()
{
    //TODO: set the setpoint to the PathTracker and execute
    //TODO: later we only set the setpoint to the PathPlanner, let it compute the trajectory,
    // save the trajectory to PathTracker and then execute the sequence.
    _robot.path_tracker.setGoalSetpoint(_setpointX, _setpointY);
    _robot.path_tracker.start();
}

void MainWindow::on_pushButton_11_clicked()
{
    _robot.path_tracker.stop();
    _robot.setSpeed(0, 0);
}

void MainWindow::on_pushButton_12_clicked()
{
    _robot.exportMap();
}

void MainWindow::on_pushButton_13_clicked()
{
    _robot.importMap();
}

void MainWindow::on_pushButton_14_clicked()
{
    Point cur_position = {_robot.odom.getX(), _robot.odom.getY()};
    Point goal_position = {_setpointX, _setpointY};
    _robot.mapper.plan(cur_position, goal_position);
}
