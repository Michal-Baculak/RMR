#include "mclocalizer.h"


mclocalizer::mclocalizer()
    : _rng(std::random_device{}())
{}

void mclocalizer::init(const cv::Mat &binObstacleMap, double cellSize, int numParticles)
{
    if (binObstacleMap.empty() || binObstacleMap.rows != binObstacleMap.cols)
    {
        std::cerr << "[MCL] init failed: map is empty or not square" << std::endl;
        return;
    }

    if (numParticles <= 0 || cellSize <= 0.0)
    {
        std::cerr << "[MCL] init failed: invalid numParticles or cellSize" << std::endl;
        return;
    }

    // save initial state and save a copy of map
    _obstacleMap = binObstacleMap.clone();
    _cellSize = cellSize;
    _midPoint = _obstacleMap.rows / 2;

    _bboxComputed = false;

    createDistanceField();

    // clear and reserve space for new particles with same weights

    _particles.clear();
    _particles.reserve(static_cast<size_t>(numParticles));

    const double initWeight = 1.0 / static_cast<double>(numParticles); // sum of all weights is 1

    for (int i = 0; i < numParticles; ++i) {
        Pose p = sampleRandomFreePose();
        _particles.push_back({p, initWeight});
    }

    _initialized = true;

    std::cout << "[MCL] initialized with " << numParticles << " particles. "
              << "Map: " << _obstacleMap.rows << "x" << _obstacleMap.cols
              << ", cellSize=" << _cellSize << " m, midPoint=" << _midPoint << std::endl;


}

cv::Point mclocalizer::poseToMapIndex(double x, double y) const
{
    int ix = static_cast<int>(std::round(x / _cellSize)) + _midPoint;
    int iy = static_cast<int>(std::round(y / _cellSize)) + _midPoint;
    return cv::Point(ix, iy);
}

bool mclocalizer::isFree(double x, double y) const
{
    // cv::Point idx = poseToMapIndex(x, y);
    // if (idx.x < 0 || idx.x >= _obstacleMap.cols || idx. y < 0 || idx.y >= _obstacleMap.rows) {
    //     return false;
    // }

    // return _obstacleMap.at<uint8_t>(idx.y, idx.x) == 0;
    cv::Point idx = poseToMapIndex(x, y);
    if (idx.x < 0 || idx.x >= _obstacleMap.cols || idx.y < 0 || idx.y >= _obstacleMap.rows) {
        return false;
    }

    // outside the mapped area is NOT a valid pose, even though there is no obstacle there
    if (!_insideMask.empty() && _insideMask.at<uint8_t>(idx.y, idx.x) == 0)
        return false;

    return _obstacleMap.at<uint8_t>(idx.y, idx.x) == 0;
}

Pose mclocalizer::sampleRandomFreePose()
{
    // check whether map is full of obstacles
    if (_validFreeCells.empty()) {
        std::cerr << "[MCL] no valid free cells available!" << std::endl;
        return {0.0, 0.0, 0.0};
    }

    std::uniform_int_distribution<size_t> idxDist(0, _validFreeCells.size() - 1);
    std::uniform_real_distribution<double> rotPhi(-PI, PI);

    cv::Point pt = _validFreeCells[idxDist(_rng)];
    double x = (pt.x - _midPoint) * _cellSize;
    double y = (pt.y - _midPoint) * _cellSize;
    return {x, y, rotPhi(_rng)};
}

void mclocalizer::createDistanceField()
{
    // // invert Map to properly use distanceTransform
    // cv::Mat inverted;
    // cv::bitwise_not(_obstacleMap, inverted);
    // cv::distanceTransform(inverted, _distanceField, cv::DIST_L2, 5); // distance to the nearest black px
    // _distanceField *= static_cast<float>(_cellSize);

    // // calculate bounding box
    // // distance field in WHOLE window
    // _bboxMin = cv::Point(0, 0);
    // _bboxMax = cv::Point(_obstacleMap.cols - 1, _obstacleMap.rows - 1);
    // _bboxComputed = true;

    // // obtain fields's extremes
    // double minVal, maxVal;
    // cv::minMaxLoc(_distanceField, &minVal, &maxVal);

    // // field of free cells - sampleFreePose uses this
    // _validFreeCells.clear();
    // for (int y = 0; y < _obstacleMap.rows; ++y) {
    //     for (int x = 0; x < _obstacleMap.cols; ++x) {
    //         if (_obstacleMap.at<uint8_t>(y, x) == 0)
    //             _validFreeCells.emplace_back(x, y);
    //     }
    // }
    // std::cout << "[MCL] valid free cells (full map): "
    //           << _validFreeCells.size() << std::endl;
    // std::cout << "[MCL] distance field: min=" << minVal
    //           << " m, max=" << maxVal << " m" << std::endl;

    // invert Map to properly use distanceTransform
    cv::Mat inverted;
    cv::bitwise_not(_obstacleMap, inverted);
    cv::distanceTransform(inverted, _distanceField, cv::DIST_L2, 5); // distance to the nearest black px
    _distanceField *= static_cast<float>(_cellSize);

    // detect the actual mapped area (the room interior)
    // Free cells form 1+ connected components. The component(s) that touch
    // the image border are "outside" of the room - we exclude them so that
    // particles cannot be sampled there.
    cv::Mat freeMask;
    cv::compare(_obstacleMap, 0, freeMask, cv::CMP_EQ); // 255 where free

    cv::Mat labels;
    int nComp = cv::connectedComponents(freeMask, labels, 8, CV_32S);
    (void) nComp;

    std::set<int> outsideLabels;
    for (int x = 0; x < labels.cols; ++x) {
        outsideLabels.insert(labels.at<int>(0, x));
        outsideLabels.insert(labels.at<int>(labels.rows - 1, x));
    }
    for (int y = 0; y < labels.rows; ++y) {
        outsideLabels.insert(labels.at<int>(y, 0));
        outsideLabels.insert(labels.at<int>(y, labels.cols - 1));
    }

    _insideMask = cv::Mat::zeros(_obstacleMap.size(), CV_8UC1);
    _validFreeCells.clear();

    int xMin = _obstacleMap.cols, yMin = _obstacleMap.rows, xMax = 0, yMax = 0;
    for (int y = 0; y < labels.rows; ++y) {
        for (int x = 0; x < labels.cols; ++x) {
            int lbl = labels.at<int>(y, x);
            // lbl == 0 => background (obstacle pixel in our freeMask)
            if (lbl > 0 && outsideLabels.count(lbl) == 0) {
                _insideMask.at<uint8_t>(y, x) = 255;
                _validFreeCells.emplace_back(x, y);
                if (x < xMin) xMin = x;
                if (y < yMin) yMin = y;
                if (x > xMax) xMax = x;
                if (y > yMax) yMax = y;
            }
        }
    }

    // Fallback: if the room is not fully enclosed, fall back to bounding box
    // of obstacles so we at least don't sample across the whole window.
    if (_validFreeCells.empty()) {
        std::cerr << "[MCL] no enclosed free area found, falling back to bbox of obstacles"
                  << std::endl;
        std::vector<cv::Point> obstaclePts;
        cv::findNonZero(_obstacleMap, obstaclePts);
        if (!obstaclePts.empty()) {
            cv::Rect bbox = cv::boundingRect(obstaclePts);
            xMin = bbox.x;
            yMin = bbox.y;
            xMax = bbox.x + bbox.width  - 1;
            yMax = bbox.y + bbox.height - 1;
            for (int y = yMin; y <= yMax; ++y) {
                for (int x = xMin; x <= xMax; ++x) {
                    if (_obstacleMap.at<uint8_t>(y, x) == 0) {
                        _insideMask.at<uint8_t>(y, x) = 255;
                        _validFreeCells.emplace_back(x, y);
                    }
                }
            }
        }
    }

    _bboxMin = cv::Point(xMin, yMin);
    _bboxMax = cv::Point(xMax, yMax);
    _bboxComputed = true;

    // obtain field's extremes (only over the inside area for nicer logs)
    double minVal = 0.0, maxVal = 0.0;
    if (!_validFreeCells.empty())
        cv::minMaxLoc(_distanceField, &minVal, &maxVal, nullptr, nullptr, _insideMask);
    else
        cv::minMaxLoc(_distanceField, &minVal, &maxVal);

    std::cout << "[MCL] valid free cells (inside room): "
              << _validFreeCells.size() << std::endl;
    std::cout << "[MCL] inside bbox: (" << _bboxMin.x << "," << _bboxMin.y
              << ") - (" << _bboxMax.x << "," << _bboxMax.y << ")" << std::endl;
    std::cout << "[MCL] distance field: min=" << minVal
              << " m, max=" << maxVal << " m" << std::endl;
}

double mclocalizer::distToNearestObstacle(double x, double y) const
{
    cv::Point idx = poseToMapIndex(x, y);
    if (idx.x < 0 || idx.x >= _distanceField.cols || idx.y < 0 || idx.y >= _distanceField.rows)
    {
        return -1.0;
    }

    return static_cast<double>(_distanceField.at<float>(idx.y, idx.x));
}

void mclocalizer::setMotionNoise(double a1, double a2, double a3, double a4)
{
    _a1 = a1;
    _a2 = a2;
    _a3 = a3;
    _a4 = a4;
}

//TODO: Motion model with Gaussian noise
void mclocalizer::updateMotion(double dx, double dy, double dPhi)
{
    // odometry motion model
    if (!_initialized || _particles.empty())
        return;

    const double dtrans = std::sqrt(dx * dx + dy * dy);

    double drot1, drot2;
    if (dtrans < TRANS_EPS) {
        drot1 = 0.0;
        drot2 = dPhi;
    } else {
        drot1 = std::atan2(dy, dx);
        drot2 = utility::wrap(dPhi - drot1);
    }

    // decompose motion to 3 parts
    // a1 to a4 define errors
    // rotation to target
    const double sigma_rot1 = std::sqrt(_a1 * utility::pow(drot1) + _a2 * utility::pow(dtrans));
    // translation
    const double sigma_trans = std::sqrt(_a3 * utility::pow(dtrans) + _a4 * (utility::pow(drot1) + utility::pow(drot2)));
    // final rot
    const double sigma_rot2 = std::sqrt(_a1 * utility::pow(drot2) + _a2 * utility::pow(dtrans));

    std::normal_distribution<double> n_rot1(0.0, sigma_rot1);
    std::normal_distribution<double> n_trans(0.0, sigma_trans);
    std::normal_distribution<double> n_rot2(0.0, sigma_rot2);

    for (auto &p : _particles) {
        // for each particle calculate new candidate pose
        const double rot1_hat = drot1 - n_rot1(_rng);
        const double trans_hat = dtrans - n_trans(_rng);
        const double rot2_hat = drot2 - n_rot2(_rng);

        const double phi_chckpnt = utility::wrap(p.pose.phi + rot1_hat);
        double nx = p.pose.x + trans_hat * std::cos(phi_chckpnt);
        double ny = p.pose.y + trans_hat * std::sin(phi_chckpnt);
        double nphi = utility::wrap(phi_chckpnt + rot2_hat);
        // if new pose of the current particle is possibly in wall, resample it to random new pos
        if (!isFree(nx, ny)) {
            Pose randPose = sampleRandomFreePose();
            p.pose = randPose;
        } else {
            p.pose.x = nx;
            p.pose.y = ny;
            p.pose.phi = nphi;
        }
    }
}

//TODO: update weigths from lidar measurement
void mclocalizer::updateWeights(const std::vector<LaserData> &laserData)
{
    if (!_initialized || _particles.empty() || laserData.empty())
        return;

    double sumWeights = 0.0;
    // calculate where should each beam be projected in map's global frame
    for (auto &p : _particles) {
        double errorSum = 0.0;
        int validBeams = 0;

        for (size_t i = 0; i < laserData.size(); i += LASER_STRIDE) {
            const auto &beam = laserData[i];
            double d = beam.scanDistance / 1000.0;
            if (d < LIDAR_MIN_M || d > LIDAR_MAX_M)
                continue;

            double angle = p.pose.phi - beam.scanAngle * PI / 180.0;
            double ox = p.pose.x + d * std::cos(angle);
            double oy = p.pose.y + d * std::sin(angle);

            // calculate distance to nearest obstacle
            double dist = distToNearestObstacle(ox, oy);
            if (dist < 0.0)
                continue;

            errorSum += dist;
            ++validBeams;
        }

        // calc weights
        //the smaller the total error (errorSum), the greater the particle's weight...
        if (validBeams == 0){
            p.weight = WEIGHT_EPS;
        } else {
            p.weight = 1.0 / (errorSum + WEIGHT_EPS);
        }

        sumWeights += p.weight;
    }

    // normalization to 0 - 1 to represent particle as percentual probability of robot being in the actual pose
    if (sumWeights > 0.0) {
        for (auto &p : _particles)
            p.weight /= sumWeights;
    } else {
        const double w = 1.0 / static_cast<double>(_particles.size());
        for (auto &p : _particles)
            p.weight = w;
    }
}

//TODO: resampling
void mclocalizer::resample()
{
    if (!_initialized || _particles.empty())
        return;

    int N = static_cast<int>(_particles.size());

    // calculate num of new random poses
    int n_random = static_cast<int>(std::round(_injectionRatio * N));
    n_random = std::clamp(n_random, 0, N);
    int n_resample = N - n_random;

    std::vector<Particle> newParticles;
    newParticles.reserve(N);

    // selection
    if (n_resample > 0) {
        double step = 1.0 / static_cast<double>(n_resample);
        std::uniform_real_distribution<double> u(0.0, step);
        double rand_pose = u(_rng);
        double cumulative = _particles[0].weight;
        int i = 0;

        for (int m = 0; m < n_resample; ++m) {
            double target = rand_pose + m * step;
            while (target > cumulative && i < N - 1) {
                ++i;
                cumulative += _particles[i].weight;
            }
            newParticles.push_back({_particles[i].pose, 1.0 / N});
        }
    }

    for (int k = 0; k < n_random; ++k) {
        Pose p = sampleRandomFreePose();
        newParticles.push_back({p, 1.0 / N});
    }

    _particles = std::move(newParticles);
}

Pose mclocalizer::getBestPose() const
{
    // return particle with the highest weight
    if (_particles.empty())
        return {0.0, 0.0, 0.0};

    auto it = std::max_element(_particles.begin(), _particles.end(), [](const Particle &a, const Particle &b){return a.weight < b.weight; });
    return it->pose;
}

cv::Mat mclocalizer::getVisualization() const
{
    if (!_initialized || _distanceField.empty())
        return cv::Mat();

    // distance field -> color map, blue = close to obstacle, red = far
    // cv::Mat validMask = _distanceField >= 0;
    // double minVal, maxVal;
    // cv::minMaxLoc(_distanceField, &minVal, &maxVal, nullptr, nullptr, validMask);
    // if (maxVal <= minVal) maxVal = minVal + 1.0;

    // cv::Mat dfNorm;
    // _distanceField.convertTo(dfNorm, CV_8U,
    //                          255.0 / (maxVal - minVal),
    //                          -255.0 * minVal / (maxVal - minVal));
    // cv::Mat dfColor;
    // cv::applyColorMap(dfNorm, dfColor, cv::COLORMAP_JET);
    //dfColor.setTo(cv::Scalar(40, 40, 40), ~validMask);

    cv::Mat scaleMask = (!_insideMask.empty()) ? _insideMask : (_distanceField >= 0);
    double minVal, maxVal;
    cv::minMaxLoc(_distanceField, &minVal, &maxVal, nullptr, nullptr, scaleMask);
    if (maxVal <= minVal) maxVal = minVal + 1.0;

    cv::Mat dfNorm;
    _distanceField.convertTo(dfNorm, CV_8U,
                             255.0 / (maxVal - minVal),
                             -255.0 * minVal / (maxVal - minVal));
    cv::Mat dfColor;
    cv::applyColorMap(dfNorm, dfColor, cv::COLORMAP_JET);

    // grey-out the area outside the mapped room so it is visually obvious
    // that MCL only operates inside the actual map
    if (!_insideMask.empty())
        dfColor.setTo(cv::Scalar(40, 40, 40), ~_insideMask);

    // max weight due to particles color scaling
    double maxW = 0.0;
    for (const auto &p : _particles)
        if (p.weight > maxW) maxW = p.weight;
    if (maxW <= 0.0) maxW = 1.0;

    // the greater the weight, the lighter dot
    for (const auto &p : _particles) {
        cv::Point idx = poseToMapIndex(p.pose.x, p.pose.y);
        if (idx.x < 0 || idx.x >= dfColor.cols || idx.y < 0 || idx.y >= dfColor.rows)
            continue;
        int v = static_cast<int>(255.0 * (p.weight / maxW));
        cv::circle(dfColor, idx, 1, cv::Scalar(v, v, 255), -1);
    }

    // best pose - red arrow
    if (!_particles.empty()) {
        Pose best = getBestPose();
        cv::Point b = poseToMapIndex(best.x, best.y);
        if (b.x >= 0 && b.x < dfColor.cols && b.y >= 0 && b.y < dfColor.rows) {
            cv::Point tip(b.x + static_cast<int>(15.0 * std::cos(best.phi)),
                          b.y + static_cast<int>(15.0 * std::sin(best.phi)));
            cv::arrowedLine(dfColor, b, tip, cv::Scalar(0, 0, 255), 2, cv::LINE_AA, 0, 0.4);
            cv::circle(dfColor, b, 4, cv::Scalar(0, 0, 255), 2);
        }
    }

    cv::Mat flipped, resized;
    cv::flip(dfColor, flipped, 0);
    cv::resize(flipped, resized, cv::Size(), 2.0, 2.0, cv::INTER_NEAREST);

    if (!_particles.empty()) {
    Pose best = getBestPose();
    char buf[128];
    std::snprintf(buf, sizeof(buf),
                  "x=%.2f m  y=%.2f m  phi=%.1f deg",
                  best.x, best.y, best.phi * 180.0 / PI);
    cv::putText(resized, buf, cv::Point(10, 25),
                cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

    std::string st = "particles: " + std::to_string(_particles.size());
    cv::putText(resized, st, cv::Point(10, 50),
                cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
    }

    return resized;
}


bool mclocalizer::isLocalized() const
{
    if (!_initialized || _particles.size() < 10)
        return false;

    // sort particles by weights, drop low 10 % (injection)
    std::vector<Particle> sorted = _particles;
    std::sort(sorted.begin(), sorted.end(),
              [](const Particle &a, const Particle &b){ return a.weight > b.weight; });
    size_t keep = static_cast<size_t>(0.9 * sorted.size());
    if (keep < 10) keep = sorted.size();

    double sumW = 0.0;
    for (size_t i = 0; i < keep; ++i) sumW += sorted[i].weight;
    if (sumW <= 0.0) {
        // post-resample, uniform weights, use average
        sumW = static_cast<double>(keep);
        for (size_t i = 0; i < keep; ++i) sorted[i].weight = 1.0;
    }

    // calculate mean value and variance
    double mx=0.0, my=0.0, sSin=0.0, sCos=0.0;
    for (size_t i = 0; i < keep; ++i) {
        double w = sorted[i].weight / sumW;
        mx   += w * sorted[i].pose.x;
        my   += w * sorted[i].pose.y;
        sSin += w * std::sin(sorted[i].pose.phi);
        sCos += w * std::cos(sorted[i].pose.phi);
    }
    double mphi = std::atan2(sSin, sCos);

    double vx=0.0, vy=0.0, vphi=0.0;
    for (size_t i = 0; i < keep; ++i) {
        double w  = sorted[i].weight / sumW;
        double dx = sorted[i].pose.x - mx;
        double dy = sorted[i].pose.y - my;
        double dp = utility::wrap(sorted[i].pose.phi - mphi);
        vx   += w * dx * dx;
        vy   += w * dy * dy;
        vphi += w * dp * dp;
    }

    //std dev, particles close to each other = small deviation
    double sx = std::sqrt(vx);
    double sy = std::sqrt(vy);
    double sp = std::sqrt(vphi);

    std::cout << "[MCL] sx=" << sx << " sy=" << sy << " sphi=" << sp << std::endl;
    return (sx < _locStdXY && sy < _locStdXY && sp < _locStdPhi);
}

// otazky:
// 1. mapper zapinat tlacidlom nezavisle od ostatnych vrstiev?
// 2. Lokalizovat len raz a po konvergencii lokalizaciu vypnut, aby sa nahodou neprepisovala poloha?
// 3. Ako to bude prebiehat na sutazi? ked polozime robot v nejakej mape, bude mat on sam cas na to, aby sa lokalizoval? Pripadne budeme mu vediet urcit,
// ze kde sme ho polozili?