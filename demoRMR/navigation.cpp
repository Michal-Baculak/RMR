#include "navigation.h"

Navigation::Navigation()
    : _prev_binary_hist(NUM_SECTORS, 0)
    , _prev_dir(0)
{}

int Navigation::circularDist(int k1, int k2) const
{
    // circular distance of sectors (minimum distance of sectors 2 & 34 is 3, not 32)
    int diff = std::abs(k1 - k2);
    return std::min(diff, NUM_SECTORS-diff);
}

double Navigation::update(double rX, double rY, double rPhi, double targetX, double targetY, const std::vector<LaserData> &laserData)
{
    std::vector<double> pHist(NUM_SECTORS, 0.0); // primary histogram

    //primary polar histogram
    for (int i = 0; i < laserData.size(); ++i) {
        double d_i = laserData[i].scanDistance / 1000.0; // distance to obstacle
        double alpha_i = -laserData[i].scanAngle; // angle between robot heading and obstacle's centre of mass

        if (d_i > 0.05 && d_i < 3.0){
            double m_i = _a - _b * d_i; // obstacle's magnitude
            if (m_i < 0) m_i = 0;

            double gamma_i = std::asin(std::min(1.0, RS / d_i)) * 180 / PI; // magnification angle

            for (int k = 0; k < NUM_SECTORS; ++k){
                double p_k = SIGMA * k;
                double diff = std::fmod(std::abs(p_k - alpha_i) + 180.0, 360.0) - 180.0;
                if(std::abs(diff) < gamma_i){
                    pHist[k] += m_i;
                }

                // if(std::max(k*SIGMA, alpha_i - gamma_i) <= std::min((k+1)*SIGMA, alpha_i + gamma_i))
                // {
                //     pHist[k] += m_i;
                // }
            }
        }
    }

    //binary polar histogram
    std::vector<int> bHist(NUM_SECTORS, 0);

    for(int k = 0; k < NUM_SECTORS; ++k) {
        if (pHist[k] > _t_high) bHist[k] = 1;
        else if (pHist[k] < _t_low) bHist[k] = 0;
        else bHist[k] = _prev_binary_hist[k];
    }

    _prev_binary_hist = bHist;

    //masked polar histogram
    std::vector<int> mHist = bHist;

    for (int k = 0; k < NUM_SECTORS; k++){
        if (bHist[k] == 1) continue;

        double alpha_k = k * SIGMA * PI / 180.0;
        if (alpha_k > PI) alpha_k -= 2.0 * PI;
        if (alpha_k < -PI) alpha_k += 2.0 * PI;

        if (std::abs(alpha_k) < 1e-6) continue;

        double cy = (alpha_k > 0) ? R_MIN : -R_MIN;

        double start_ang = std::atan2(-cy, 0.0);

        for (int i = 0; i < laserData.size(); ++i) {
            double d_ij = laserData[i].scanDistance / 1000.0;
            double beta_rad = -laserData[i].scanAngle * PI / 180.0;

            if (d_ij < 0.05 || d_ij >= 3.0) continue;

            double ox = d_ij * std::cos(beta_rad);
            double oy = d_ij * std::sin(beta_rad);

            if (alpha_k > 0 && oy <= 0.0) continue;
            if (alpha_k < 0 && oy >= 0.0) continue;

            double dist_to_centre = std::sqrt(ox*ox + (oy - cy)*(oy - cy));
            double inner = std::max(0.0, R_MIN - RS);
            if (dist_to_centre < R_MIN - RS || dist_to_centre > R_MIN + RS) continue;


            double obs_ang = std::atan2(oy - cy, ox);
            bool in_arc;

            if(alpha_k > 0) {
                double sweep = obs_ang - start_ang;
                while (sweep < 0) sweep += 2.0*PI;
                while (sweep > 2.0 * PI) sweep -= 2.0*PI;

                in_arc= (sweep <= alpha_k + 0.05);
            } else {
                double sweep = start_ang - obs_ang;
                while (sweep < 0)        sweep += 2.0 * PI;
                while (sweep > 2.0 * PI) sweep -= 2.0 * PI;
                in_arc = (sweep <= -alpha_k + 0.05);
            }

            if (in_arc) { mHist[k] = 1; break; }
        }
    }

    _last_mHist = mHist;

    double targetAngleGlob = atan2(targetY - rY, targetX - rX) * 180.0 / PI;
    double targetAngleRel = targetAngleGlob - (rPhi * 180.0 / PI);
    targetAngleRel = std::fmod(targetAngleRel, 360.0);
    if (targetAngleRel < 0) targetAngleRel += 360.0;
    int k_target = static_cast<int>(targetAngleRel / SIGMA) % NUM_SECTORS;

    std::vector<int> candidates;

    if(mHist[k_target] == 0)
        candidates.push_back(k_target);

    bool all_free = true;
    for (int k = 0; k < NUM_SECTORS; ++k)
        if (mHist[k] == 1) {all_free = false; break;}

    if (all_free) {
        candidates.push_back(k_target);
    } else {
        int scan_start = 0;
        for (int k = 0; k < NUM_SECTORS; ++k){
            if (mHist[k] == 1 && mHist[(k+1) % NUM_SECTORS] == 0) {
                scan_start = (k+1) % NUM_SECTORS;
                break;
            }
        }

        int k = scan_start;
        int iter = 0;
        while (iter < NUM_SECTORS) {
            if (mHist[k] == 0) {
                int v_start = k;
                int width = 0;

                while (mHist[k] == 0 && width < NUM_SECTORS) {
                    k = (k + 1) % NUM_SECTORS;
                    width++;
                    iter++;
                }

                if (width <= S_MAX) {
                    int mid = (v_start + width / 2) % NUM_SECTORS;
                    candidates.push_back(mid);
                } else {
                    int c_right = (v_start + S_MAX / 2) % NUM_SECTORS;
                    int c_left = (v_start + width - 1 - S_MAX / 2 + NUM_SECTORS) % NUM_SECTORS;
                    candidates.push_back(c_right);
                    candidates.push_back(c_left);
                }
            } else {
                k = (k + 1) % NUM_SECTORS;
                iter++;
            }
        }
    }

    if (candidates.empty()) return rPhi; // everything blocked = return current rotation angle

    int best_k = -1;
    double min_cost = 1e9;

    for (int c : candidates) {
        double cost = _mu1 * circularDist(c, k_target)
                      + _mu2 * circularDist(c, rPhi)
                      + _mu3 * circularDist(c, _prev_dir);
        if (cost < min_cost) {
            min_cost = cost;
            best_k = c;
        }
    }

    if (best_k == -1) return rPhi;
    _prev_dir = best_k;

    double finalAngleDeg = best_k * SIGMA;
    if (finalAngleDeg > 180.0) finalAngleDeg -= 360.0;
    double finalAngleRad = finalAngleDeg * PI / 180.0;

    //std::cout << "final angle: " << rPhi + finalAngleRad * 180.0 / PI << std::endl;

    return rPhi + finalAngleRad;
}

void Navigation::printLaserData(const std::vector<LaserData> &laser)
{
    std::cout << "Size: " << laser.size() << std::endl;

    for (size_t i = 0; i < laser.size(); i++) {
        if (i % 50 == 0) {
            std::cout << "Bod [" << i << "]: "
                      << "Uhol = " << laser[i].scanAngle << ", "
                      << "Vzdialenost = " << laser[i].scanDistance/1000.0 << "mm"
                      << std::endl;
        }
    }
}

