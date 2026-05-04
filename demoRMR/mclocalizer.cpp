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

    _obstacleMap = binObstacleMap.clone();
    _cellSize = cellSize;
    _midPoint = _obstacleMap.rows / 2;

    _bboxComputed = false;

    createDistanceField();

    _particles.clear();
    _particles.reserve(static_cast<size_t>(numParticles));

    const double initWeight = 1.0 / static_cast<double>(numParticles);

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
    cv::Point idx = poseToMapIndex(x, y);
    if (idx.x < 0 || idx.x >= _obstacleMap.cols || idx. y < 0 || idx.y >= _obstacleMap.rows) {
        return false;
    }

    return _obstacleMap.at<uint8_t>(idx.y, idx.x) == 0;
}

Pose mclocalizer::sampleRandomFreePose()
{
    // double minX = (-_midPoint) * _cellSize;
    // double maxX = (_obstacleMap.cols - 1 - _midPoint) * _cellSize;
    // double minY = (-_midPoint) * _cellSize;
    // double maxY = (_obstacleMap.rows - 1 - _midPoint) * _cellSize;

    // std::uniform_real_distribution<double> distX(minX, maxX);
    // std::uniform_real_distribution<double> distY(minY, maxY);
    // std::uniform_real_distribution<double> rotPhi(-PI, PI);

    // for (int i = 0; i < MAX_ATTEMPTS; ++i) {
    //     double x = distX(_rng);
    //     double y = distY(_rng);
    //     double phi = rotPhi(_rng);

    //     if (isFree(x, y))
    //         return {x , y, phi};
    // }
    // std::cerr << "[MCL] failed to sample a free pose after " << MAX_ATTEMPTS
    //             << " attempts. Map too occupied?" << std::endl;

    // return {distX(_rng), distY(_rng), rotPhi(_rng)};

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
    // cv::Mat inverted;
    // cv::bitwise_not(_obstacleMap, inverted);

    // cv::distanceTransform(inverted, _distanceField, cv::DIST_L2, 5);
    // _distanceField *= static_cast<float>(_cellSize);

    // double minVal, maxVal;
    // cv::minMaxLoc(_distanceField, &minVal, &maxVal);

    // std::cout << "[MCL] distance field built: min=" << minVal << " m, max=" << maxVal
    //           << " m" << std::endl;

    cv::Mat inverted;
    cv::bitwise_not(_obstacleMap, inverted);
    cv::distanceTransform(inverted, _distanceField, cv::DIST_L2, 5);
    _distanceField *= static_cast<float>(_cellSize);

    // Bbox prekážok (+ margin) - použije sa aj pri samplingu
    std::vector<cv::Point> obstaclePts;
    cv::findNonZero(_obstacleMap, obstaclePts);
    if (!obstaclePts.empty()) {
        cv::Rect bb = cv::boundingRect(obstaclePts);
        const int margin = 5;
        _bboxMin.x = std::max(0, bb.x - margin);
        _bboxMin.y = std::max(0, bb.y - margin);
        _bboxMax.x = std::min(_obstacleMap.cols - 1, bb.x + bb.width  + margin);
        _bboxMax.y = std::min(_obstacleMap.rows - 1, bb.y + bb.height + margin);
    } else {
        _bboxMin = cv::Point(0, 0);
        _bboxMax = cv::Point(_obstacleMap.cols - 1, _obstacleMap.rows - 1);
    }
    _bboxComputed = true;

    for (int y = 0; y < _distanceField.rows; ++y) {
        for (int x = 0; x < _distanceField.cols; ++x) {
            if (x < _bboxMin.x || x > _bboxMax.x ||
                y < _bboxMin.y || y > _bboxMax.y) {
                _distanceField.at<float>(y, x) = 5; //-1.0f;
            }
        }
    }

    double minVal, maxVal;
    cv::Mat validMask = _distanceField >= 0;
    cv::minMaxLoc(_distanceField, &minVal, &maxVal, nullptr, nullptr, validMask);

    _validFreeCells.clear();
    for (int y = _bboxMin.y; y <= _bboxMax.y; ++y) {
        for (int x = _bboxMin.x; x <= _bboxMax.x; ++x) {
            if (_obstacleMap.at<uint8_t>(y, x) == 0)
                _validFreeCells.emplace_back(x, y);
        }
    }
    std::cout << "[MCL] valid free cells in bbox: "
              << _validFreeCells.size() << std::endl;

    std::cout << "[MCL] distance field built (valid range): min=" << minVal
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

    double sigma_rot1 = std::sqrt(_a1 * utility::pow(drot1) + _a2 * utility::pow(dtrans));
    double sigma_trans = std::sqrt(_a3 * utility::pow(dtrans)
                                   + _a4 * (utility::pow(drot1) + utility::pow(drot2)));
    double sigma_rot2 = std::sqrt(_a1 * utility::pow(drot2) + _a2 * utility::pow(dtrans));

    sigma_rot1 = std::max(sigma_rot1, SIGMA_MIN);
    sigma_trans = std::max(sigma_trans, SIGMA_MIN);
    sigma_rot2 = std::max(sigma_rot2, SIGMA_MIN);

    std::normal_distribution<double> n_rot1(0.0, sigma_rot1);
    std::normal_distribution<double> n_trans(0.0, sigma_trans);
    std::normal_distribution<double> n_rot2(0.0, sigma_rot2);

    for (auto &p : _particles) {
        const double rot1_hat = drot1 - n_rot1(_rng);
        const double trans_hat = dtrans - n_trans(_rng);
        const double rot2_hat = drot2 - n_rot2(_rng);

        const double phi_chckpnt = utility::wrap(p.pose.phi + rot1_hat);
        p.pose.x += trans_hat * std::cos(phi_chckpnt);
        p.pose.y += trans_hat * std::sin(phi_chckpnt);
        p.pose.phi = utility::wrap(phi_chckpnt + rot2_hat);
    }
}

//TODO: update weigths from lidar measurement
void mclocalizer::updateWeights(const std::vector<LaserData> &laserData)
{
    if (!_initialized || _particles.empty() || laserData.empty())
        return;

    double sumWeights = 0.0;
    for (auto &p : _particles) {
        //double cs = std::cos(p.pose.phi);
        //double sn = std::sin(p.pose.phi);

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

            double dist = distToNearestObstacle(ox, oy);
            if (dist < 0.0)
                continue;

            errorSum += dist;
            ++validBeams;
        }

        if (validBeams == 0){
            p.weight = WEIGHT_EPS;
        } else {
            p.weight = 1.0 / (errorSum + WEIGHT_EPS);
        }

        sumWeights += p.weight;
    }

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

    int n_random = static_cast<int>(std::round(_injectionRatio * N));
    n_random = std::clamp(n_random, 0, N);
    int n_resample = N - n_random;

    std::vector<Particle> newParticles;
    newParticles.reserve(N);

    if (n_resample > 0) {
        double step = 1.0 / static_cast<double>(n_resample);
        std::uniform_real_distribution<double> u(0.0, step);
        double r = u(_rng);
        double cumulative = _particles[0].weight;
        int i = 0;

        for (int m = 0; m < n_resample; ++m) {
            double target = r + m * step;
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
    if (_particles.empty())
        return {0.0, 0.0, 0.0};

    auto it = std::max_element(_particles.begin(), _particles.end(), [](const Particle &a, const Particle &b){return a.weight < b.weight; });
    return it->pose;
}

cv::Mat mclocalizer::getVisualization() const
{
    if (!_initialized || _distanceField.empty())
        return cv::Mat();

    // 1) distance field -> farebná mapa (modré = blízko prekážky, červené = ďaleko)
    cv::Mat validMask = _distanceField >= 0;
    double minVal, maxVal;
    cv::minMaxLoc(_distanceField, &minVal, &maxVal, nullptr, nullptr, validMask);
    if (maxVal <= minVal) maxVal = minVal + 1.0;

    cv::Mat dfNorm;
    _distanceField.convertTo(dfNorm, CV_8U,
                             255.0 / (maxVal - minVal),
                             -255.0 * minVal / (maxVal - minVal));
    cv::Mat dfColor;
    cv::applyColorMap(dfNorm, dfColor, cv::COLORMAP_JET);
    dfColor.setTo(cv::Scalar(40, 40, 40), ~validMask);

    // 2) max váha kvôli farebnému škálovaniu častíc
    double maxW = 0.0;
    for (const auto &p : _particles)
        if (p.weight > maxW) maxW = p.weight;
    if (maxW <= 0.0) maxW = 1.0;

    // 3) častice: čím vyššia váha, tým belšia bodka
    for (const auto &p : _particles) {
        cv::Point idx = poseToMapIndex(p.pose.x, p.pose.y);
        if (idx.x < 0 || idx.x >= dfColor.cols || idx.y < 0 || idx.y >= dfColor.rows)
            continue;
        int v = static_cast<int>(255.0 * (p.weight / maxW));
        cv::circle(dfColor, idx, 1, cv::Scalar(v, v, 255), -1);
    }

    // 4) best pose ako červená šípka
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
    return resized;
}
