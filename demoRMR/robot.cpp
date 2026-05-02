#include "robot.h"
#include "chrono"
#include "opencv2/core/hal/interface.h"
#include "opencv2/opencv.hpp"

robot::robot(QObject *parent) : QObject(parent)
{
    qRegisterMetaType<LaserMeasurement>("LaserMeasurement");
    #ifndef DISABLE_OPENCV
    qRegisterMetaType<cv::Mat>("cv::Mat");
#endif
#ifndef DISABLE_SKELETON
qRegisterMetaType<skeleton>("skeleton");
#endif
}

void robot::initAndStartRobot(std::string ipaddress)
{
    forwardspeed=0;
    rotationspeed=0;
    ///setovanie veci na komunikaciu s robotom/lidarom/kamerou.. su tam adresa porty a callback.. laser ma ze sa da dat callback aj ako lambda.
    /// lambdy su super, setria miesto a ak su rozumnej dlzky,tak aj prehladnost... ak ste o nich nic nepoculi poradte sa s vasim doktorom alebo lekarnikom...
    robotCom.setLaserParameters([this](const std::vector<LaserData>& dat)->int{return processThisLidar(dat);},ipaddress);
    robotCom.setRobotParameters([this](const TKobukiData& dat)->int{return processThisRobot(dat);},ipaddress);
  #ifndef DISABLE_OPENCV
    robotCom.setCameraParameters(std::bind(&robot::processThisCamera,this,std::placeholders::_1),"http://"+ipaddress+":8000/stream.mjpg");
#endif
   #ifndef DISABLE_SKELETON
      robotCom.setSkeletonParameters(std::bind(&robot::processThisSkeleton,this,std::placeholders::_1));
#endif
    ///ked je vsetko nasetovane tak to tento prikaz spusti (ak nieco nieje setnute,tak to normalne nenastavi.cize ak napr nechcete kameru,vklude vsetky info o nej vymazte)
      robotCom.robotStart();

      mapper.init();
}

void robot::setSpeedVal(double forw, double rots)
{
    forwardspeed=forw;
    rotationspeed=rots;
    useDirectCommands=0;
}

void robot::setSpeed(double forw, double rots)
{
    if(forw==0 && rots!=0)
        robotCom.setRotationSpeed(rots);
    else if(forw!=0 && rots==0)
        robotCom.setTranslationSpeed(forw);
    else if((forw!=0 && rots!=0))
        robotCom.setArcSpeed(forw,forw/rots);
    else
        robotCom.setTranslationSpeed(0);
    useDirectCommands=1;
}

///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
int robot::processThisRobot(const TKobukiData &robotdata)
{
    // check for concurrency with previous call
    if (!main_process_mutex.try_lock()) {
        std::cerr << "Failed to lock processThisRobot mutex, previous loop might have taken too "
                     "long to execute"
                  << std::endl;
    } else {
        main_process_mutex.unlock();
    }

    // mutex the shit out of this function
    std::lock_guard<std::mutex> main_process_lock(main_process_mutex);

    if (mapper.hasGoalPose() && (++planner_timing_counter % mapper.TICKS_TO_UPDATE_MAP) == 0
        && path_tracker.getSetpoints().size() > 1) {
        mapper.updatePlan({odom.getX(), odom.getY()});

        // remapping failed!
        if (!mapper.isPlanned()) {
            // path_tracker.stop();
            std::cerr << "Failed to remap path plan! " << std::endl;
        } else {
            // mappin successful
            auto trajectory = mapper.getPathPlan();
            path_tracker.setTrajectory(trajectory);
        }
    }

    ///tu mozete robit s datami z robota
    if (!odom.isInitialized())
        odom.init(robotdata);

    odom.update(robotdata);

    std::vector<LaserData> localLaser;
    // mutex LiDAR processing,
    {
        std::lock_guard<std::mutex> lidar_lock(lidar_data_mutex);
        localLaser = copyOfLaserData;
        bool run_mapper = new_lidar_data;
        if (run_mapper) {
            std::vector<XYQPoint> xyPointCloud;
            odom.compensateLidarScan(copyOfLaserData, xyPointCloud);
            // mapper.update(odom, copyOfLaserData);
            mapper.update(xyPointCloud);
            plotMap();
            emit publishLidar(copyOfLaserData);
            new_lidar_data = false;

            if (mcl.isInitialized()) {
                std::lock_guard<std::mutex> mclLock(mclMutex);

                Pose currOdom = {odom.getX(), odom.getY(), odom.getRot()};

                if (!_hasLastMclOdom) {
                    _lastMclOdom = currOdom;
                    _hasLastMclOdom = true;
                } else {

                    double dx_global = currOdom.x - _lastMclOdom.x;
                    double dy_global = currOdom.y - _lastMclOdom.y;
                    double dphi = currOdom.phi - _lastMclOdom.phi;

                    double cs = std::cos(_lastMclOdom.phi);
                    double sn = std::sin(_lastMclOdom.phi);
                    double dx_local = dx_global * cs + dy_global * sn;
                    double dy_local = -dx_global * sn + dy_global * cs;

                    double trans_eps = 0.005; // 5 mm
                    double rot_eps = 0.005;   // ~0.3 deg
                    if (std::sqrt(dx_local*dx_local + dy_local*dy_local) > trans_eps
                        || std::abs(dphi) > rot_eps) {

                        mcl.updateMotion(dx_local, dy_local, dphi);
                        mcl.updateWeights(copyOfLaserData);
                        mcl.resample();

                        cv::Mat vis = mcl.getVisualization();
                        if (!vis.empty()) {
                            cv::imshow("MCL", vis);
                            cv::waitKey(1);
                        }

                        _lastMclOdom = currOdom;
                    }
                }
            }
        }
    }
    // if (path_tracker.isRunning()) {
    //     path_tracker.update(odom);
    //     auto command = path_tracker.getCommand();
    //     std::cout << "Setting v to " << command.first << " m/s and w to " << command.second
    //               << "rad/s" << std::endl;
    //     setSpeed(command.first * 1000, command.second);

    if (path_tracker.isRunning() && !localLaser.empty())
    {
        double currentV = odom.getV();     // [m/s]
        double currentW = odom.getOmega(); // [rad/s]
        double rPhi     = odom.getRot();   // [rad] – globalny uhol robota

        double dx = path_tracker.getSetpointX() - odom.getX();
        double dy = path_tracker.getSetpointY() - odom.getY();
        double targetAngle = std::atan2(dy, dx); // [-pi, pi]

        std::optional<double> safeDir;
        {
            std::lock_guard<std::mutex> lock(navMutex);
            safeDir = nav.update(localLaser, rPhi, currentV, currentW, targetAngle);
        }

        if (!safeDir.has_value())
        {
            path_tracker.brake();
        }
        else {
            std::cout << "Is current heading safe? " << nav.isDirWithinCurrentSector(rPhi, rPhi)
                      << std::endl;
            path_tracker.updateVFH(odom, safeDir.value());
        }

        auto cmd = path_tracker.getCommand();
        setSpeed(cmd.first * 1000.0, cmd.second);

        // Vizualizacia histogramu (kazdy 5. tick)
        // if (datacounter % 5 == 0)
        {
            std::lock_guard<std::mutex> lock(navMutex);
            emit publishHistogram(nav.getLastMHist());
        }
    }

    ///TU PISTE KOD... TOTO JE TO MIESTO KED NEVIETE KDE ZACAT,TAK JE TO NAOZAJ TU. AK AJ TAK NEVIETE, SPYTAJTE SA CVICIACEHO MA TU NATO STRING KTORY DA DO HLADANIA XXX

    ///kazdy piaty krat, aby to ui moc nepreblikavalo..
    if (datacounter % 5 == 0) {
        ///ak nastavite hodnoty priamo do prvkov okna,ako je to na tychto zakomentovanych riadkoch tak sa moze stat ze vam program padne
        // ui->lineEdit_2->setText(QString::number(robotdata.EncoderRight));
        //ui->lineEdit_3->setText(QString::number(robotdata.EncoderLeft));
        //ui->lineEdit_4->setText(QString::number(robotdata.GyroAngle));
        /// lepsi pristup je nastavit len nejaku premennu, a poslat signal oknu na prekreslenie
        /// okno pocuva vo svojom slote a vasu premennu nastavi tak ako chcete. prikaz emit to presne takto spravi
        /// viac o signal slotoch tu: https://doc.qt.io/qt-5/signalsandslots.html
        ///posielame sem nezmysli.. pohrajte sa nech sem idu zmysluplne veci
        emit publishPosition(odom.getX() * 100, odom.getY() * 100, odom.getRot() * 180.0 / PI, odom.getOmega(), odom.getV());
        ///toto neodporucam na nejake komplikovane struktury.signal slot robi kopiu dat. radsej vtedy posielajte
        /// prazdny signal a slot bude vykreslovat strukturu (vtedy ju musite mat samozrejme ako member premmennu v mainwindow.ak u niekoho najdem globalnu premennu,tak bude cistit bludisko zubnou kefkou.. kefku dodam)
        /// vtedy ale odporucam pouzit mutex, aby sa vam nestalo ze budete pocas vypisovania prepisovat niekde inde
    }
    ///---tu sa posielaju rychlosti do robota... vklude zakomentujte ak si chcete spravit svoje
    if(useDirectCommands==0)
    {
        if(forwardspeed==0 && rotationspeed!=0)
            robotCom.setRotationSpeed(rotationspeed);
        else if(forwardspeed!=0 && rotationspeed==0)
            robotCom.setTranslationSpeed(forwardspeed);
        else if((forwardspeed!=0 && rotationspeed!=0))
            robotCom.setArcSpeed(forwardspeed,forwardspeed/rotationspeed);
        else
            robotCom.setTranslationSpeed(0);
    }
    datacounter++;

    return 0;

}

///toto je calback na data z lidaru, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa ked dojdu nove data z lidaru
int robot::processThisLidar(const std::vector<LaserData>& laserData)
{
    std::lock_guard<std::mutex> lidar_lock(lidar_data_mutex);
    new_lidar_data = true;
    copyOfLaserData = laserData;

    // ************** MOVED INTO PROCESSROBOT ***************************
    // std::vector<XYQPoint> xyPointCloud;
    // Odometry odom_copy = odom;
    // odom_copy.compensateLidarScan(copyOfLaserData, xyPointCloud);
    // mapper.update(odom_copy, copyOfLaserData);
    // plotMap();
    // ************** MOVED INTO PROCESSROBOT ***************************

    // ******************************** LiDAR Odometry ****************************************
    // if(!lidarOdom.isInitialized())
    //     lidarOdom.init(copyOfLaserData);

    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
    // updateLaserPicture=1;

    // lidarOdom.update(copyOfLaserData);

    //TODO: odometry fusion
    // emit publishPosition(lidarOdom.getX(),
    //                      lidarOdom.getY(),
    //                      lidarOdom.getRot()/PI*180);

    // ******************************** LiDAR Odometry ****************************************

    // emit publishLidar(copyOfLaserData);
    // update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia
    return 0;
}

cv::Mat getMatFromMap(Mapper mapper)
{
    cv::Mat mat_matrix = mapper.getMapFiltered();
    cv::Mat rotated;
    cv::flip(mat_matrix, rotated, 0);
    // cv::rotate(mat_matrix, rotated, cv::ROTATE_90_COUNTERCLOCKWISE);
    cv::Mat resized;
    size_t map_size = mapper.getMapSize();
    cv::resize(rotated, resized, cv::Size(map_size * 2, map_size * 2));

    return rotated;
}

void robot::plotMap()
{
    cv::imshow("Map", getMatFromMap(mapper));
    cv::waitKey(10);
}

void robot::exportMap()
{
    cv::imwrite("map.bmp", mapper.getMapFiltered());
}

void robot::importMap()
{
    cv::Mat map = cv::imread("map.bmp", cv::IMREAD_GRAYSCALE);
    if (map.empty()) {
        std::cerr << "Failed to load map.bmp - is it in the working directory?" << std::endl;
        return;
    }
    mapper.init(map);
    cv::Mat binMap = mapper.getMapFiltered();
    {
        std::lock_guard<std::mutex> mclLock(mclMutex);
        mcl.init(binMap, CELL_SIZE, N_PARTICLES);
        _hasLastMclOdom = false;
        cv::Mat vis = mcl.getVisualization();
        if (!vis.empty()) {
            cv::imshow("MCL", vis);
            cv::waitKey(1);
        }
    }
}
#ifndef DISABLE_OPENCV
///toto je calback na data z kamery, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa ked dojdu nove data z kamery
int robot::processThisCamera(cv::Mat cameraData)
{
    cameraData.copyTo(frame[(actIndex + 1) % 3]); //kopirujem do nasej strukury
    actIndex = (actIndex + 1) % 3;                //aktualizujem kde je nova fotka

    emit publishCamera(frame[actIndex]);
    return 0;
}
#endif

  #ifndef DISABLE_SKELETON
/// vola sa ked dojdu nove data z trackera
int robot::processThisSkeleton(skeleton skeledata)
{

    memcpy(&skeleJoints,&skeledata,sizeof(skeleton));

    emit publishSkeleton(skeleJoints);
    return 0;
}
#endif
