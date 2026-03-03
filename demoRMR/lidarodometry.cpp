#include "lidarodometry.h"

LidarOdometry::LidarOdometry()
{
    _posX = 0;
    _posY = 0;
    _rot = 0;
}

std::vector<Point2D> LidarOdometry::convertScan(std::vector<LaserData>& scan)
{
    std::vector<Point2D> points;
    if(scan.empty()) return points;

    double totalFov = scan[0].scanAngle;
    double angleInc = totalFov/static_cast<double>(scan.size());

    for(size_t i = 0; i < scan.size(); i++)
    {
        auto& l = scan[i];

        if(l.scanDistance > 1 && l.scanDistance < 2500)
        {
            double currAngle = i*angleInc;
            double angleRad = currAngle * (PI/180);
            Point2D p;
            p.x = l.scanDistance * cos(currAngle);
            p.y = l.scanDistance * sin(currAngle);
            points.push_back(p);
        }
    }

    return points;

    // for(auto& l : scan)
    // {
    //     if(l.scanDistance > 100 && l.scanDistance < 2500)
    //     {
    //         std::cout << l.scanAngle << std::endl;
    //         double angle = l.scanAngle * PI / 180.0;
    //         Point2D p;
    //         p.x = l.scanDistance * cos(angle);
    //         p.y = l.scanDistance * sin(angle);
    //         points.push_back(p);
    //     }
    // }
    // return points;
}

void LidarOdometry::icp(std::vector<Point2D>& source,
         std::vector<Point2D>& target,
         double& dx, double& dy, double& drot)
{
    dx = 0;
    dy = 0;
    drot = 0;

    if(source.size() < 10 || target.size() < 10)
        return;

    const int maxIters = 20;
    const double tolerance = 1e-4;
    const double maxMatchDist = 2.0;

    double tot_dx = 0;
    double tot_dy = 0;
    double tot_drot = 0;

    double prevError = 1e12;

    for(int iter = 0; iter < maxIters; iter++)
    {
        std::vector<Point2D> matchedSource;
        std::vector<Point2D> matchedTarget;

        for(auto& p : source)
        {
            double minDist = 1e9;
            Point2D best;

            for(auto& q : target)
            {
                double dist = (p.x-q.x)*(p.x-q.x) + (p.y-q.y)*(p.y-q.y);

                if(dist < minDist)
                {
                    minDist = dist;
                    best = q;
                }
            }

            if(minDist < maxMatchDist * maxMatchDist)
            {
                matchedSource.push_back(p);
                matchedTarget.push_back(best);
            }
        }

        if(matchedSource.size() < 10)
            break;

        double mx1=0,my1=0,mx2=0,my2=0;
        for(size_t i=0; i<matchedSource.size(); i++)
        {
            mx1+=matchedSource[i].x;
            my1+=matchedSource[i].y;
            mx2+=matchedTarget[i].x;
            my2+=matchedTarget[i].y;
        }

        mx1/=matchedSource.size();
        my1/=matchedSource.size();
        mx2/=matchedTarget.size();
        my2/=matchedTarget.size();

        double Sxx=0,Sxy=0;

        for(size_t i=0; i<matchedSource.size(); i++)
        {
            Sxx += (matchedSource[i].x - mx1)*(matchedTarget[i].x - mx2)+(matchedSource[i].y - my1)*(matchedTarget[i].y-my2);
            Sxy += (matchedSource[i].x - mx1)*(matchedTarget[i].y - my2)-(matchedSource[i].y - my1)*(matchedTarget[i].x-mx2);
        }

        double dth = atan2(Sxy, Sxx);
        double ddx = mx2 - (cos(dth)*mx1 - sin(dth)*my1);
        double ddy = my2 - (sin(dth)*mx1 + cos(dth)*my1);

        for(auto& p : source)
        {
            p.x = cos(dth)*p.x - sin(dth)*p.y + ddx;
            p.y = sin(dth)*p.x + cos(dth)*p.y + ddy;
        }

        tot_dx += cos(tot_drot)*ddx - sin(tot_drot)*ddy;
        tot_dy += sin(tot_drot)*ddx + cos(tot_drot)*ddy;
        tot_drot += dth;

        double error = 0;
        for(size_t i=0; i<matchedSource.size(); i++)
        {
            double ex = matchedTarget[i].x - matchedSource[i].x;
            double ey = matchedTarget[i].y - matchedSource[i].y;
            error += ex*ex + ey*ey;
        }

        error/=matchedSource.size();

        if(fabs(prevError - error) < tolerance)
            break;

        prevError = error;

        //std::cout << "Matched points: " << matchedSource.size() << std::endl;
    }

    dx = tot_dx;
    dy = tot_dy;
    drot = tot_drot;

}

void LidarOdometry::update(std::vector<LaserData>& scan)
{
    std::cout << "Scan size: " << scan.size() << std::endl;
    auto currentScan = convertScan(scan);
    std::cout << "Converted scan size: " << currentScan.size() << std::endl;

    double dx,dy,drot;
    icp(currentScan, _prevScan, dx, dy, drot);

    _posX += cos(_rot)*dx - sin(_rot)*dy;
    _posY += sin(_rot)*dx + cos(_rot)*dy;
    _rot += drot;

    while(_rot > PI) _rot -= 2*PI;
    while(_rot < -PI) _rot += 2*PI;

    _prevScan = currentScan;

    std::cout << "Lidar Odom posX = " << _posX << ", posY = " << _posY << ", rot = " << _rot << std::endl;
}

void LidarOdometry::init(std::vector<LaserData>& scan)
{
    _prevScan = convertScan(scan);
    _isInitialized = true;
}


