#include "navigation.h"
#include "algorithm"

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

double Navigation::update(double rX, double rY, double rPhi, double targetX, double targetY, const std::vector<LaserData> &laserData, double currentV, double currentW)
{
    std::vector<double> pHist(NUM_SECTORS, 0.0); // primary histogram

    // TODO: ratat polomer otacania dynamicky z rychlosti, pripadne v aktualnom natoceni si urcit bod niekolko mm dopredu a poslat to
    // ako setpoint pre regulator
    double r_min;
    if (std::abs(currentV) < 0.001) {
        r_min = RADIUS;
    } else {
        r_min = std::abs(currentV) / 2*PI;
        r_min = std::clamp(r_min, RADIUS, R_MIN_STATIC);
    }

    //primary polar histogram
    for (int i = 0; i < (int)laserData.size(); ++i) {
        double d_i = laserData[i].scanDistance / 1000.0; // distance to obstacle
        double alpha_i = -laserData[i].scanAngle; // angle between robot heading and obstacle's centre of mass

        alpha_i = std::fmod(alpha_i, 360.0);
        if (alpha_i < 0.0) alpha_i += 360.0;

        if (d_i > 0.05 && d_i < 3.0){
            double m_i = _a - _b * d_i; // obstacle's magnitude
            if (m_i < 0.0) m_i = 0.0;

            double gamma_i = std::asin(std::min(1.0, (RADIUS + RS) / d_i)) * 180.0 / PI; // magnification angle

            for (int k = 0; k < NUM_SECTORS; ++k){
                double p_k = SIGMA * k;
                double diff = std::fmod(std::abs(p_k - alpha_i) + 540.0, 360.0) - 180.0;
                if(std::abs(diff) <= gamma_i){
                    pHist[k] += m_i;
                }
            }
        }
    }

    //binary polar histogram
    std::vector<int> bHist(NUM_SECTORS, 0);

    for(int k = 0; k < NUM_SECTORS; ++k) {
        if (pHist[k] >= _t_high) bHist[k] = 1;
        else if (pHist[k] <= _t_low) bHist[k] = 0;
        else bHist[k] = _prev_binary_hist[k];
    }

    _prev_binary_hist = bHist;

    //masked polar histogram
    std::vector<int> mHist = bHist;

    for (int k = 0; k < NUM_SECTORS; ++k){
        if (mHist[k] == 1) continue;

        double sector_deg = k*SIGMA;
        if (sector_deg > 180.0) sector_deg -= 360.0;

        double alpha_k = sector_deg * PI / 180.0;
        // if (alpha_k > PI) alpha_k -= 2.0 * PI;
        // if (alpha_k < -PI) alpha_k += 2.0 * PI;

        // if (std::abs(alpha_k) < 1e-6) continue;

        double cy = (alpha_k > 0.0) ? r_min : -r_min;

        double start_ang = std::atan2(0.0-cy, 0.0);
        double total_sweep = std::abs(alpha_k);

        for (int i = 0; i < int(laserData.size()); ++i) {
            double d_ij = laserData[i].scanDistance / 1000.0;
            double beta_rad = -laserData[i].scanAngle * PI / 180.0;

            if (d_ij < 0.05 || d_ij >= 3.0) continue;

            double ox = d_ij * std::cos(beta_rad);
            double oy = d_ij * std::sin(beta_rad);

            // if (alpha_k > 0 && oy <= 0.0) continue;
            // if (alpha_k < 0 && oy >= 0.0) continue;

            //double dist_to_centre = std::sqrt(ox*ox + (oy - cy)*(oy - cy));
            double dist_to_centre = std::hypot(ox - 0.0, oy - cy);
            if (dist_to_centre < r_min - RS || dist_to_centre > r_min + RS) continue;

            double obs_ang = std::atan2(oy - cy, ox - 0.0);
            //bool in_arc;
            double sweep;
            if(alpha_k > 0.0) {
                sweep = obs_ang - start_ang;

            } else {
                sweep = start_ang - obs_ang;
            }

            while (sweep < 0.0) sweep += 2.0 * PI;
            while (sweep > 2.0 * PI) sweep -= 2.0 * PI;

            if (sweep <= total_sweep + 0.1) {
                mHist[k] = 1;
                break;
            }
        }
    }

    _last_mHist = mHist;

    double targetAngleGlob = std::atan2(targetY - rY, targetX - rX) * 180.0 / PI;
    double targetAngleRel = targetAngleGlob - (rPhi * 180.0 / PI);

    targetAngleRel = std::fmod(targetAngleRel, 360.0);
    if (targetAngleRel < 0.0) targetAngleRel += 360.0;

    int k_target = static_cast<int>(targetAngleRel / SIGMA) % NUM_SECTORS;

    std::vector<int> candidates;

    if(mHist[k_target] == 0)
        candidates.push_back(k_target);

    bool all_free = true;
    for (int k = 0; k < NUM_SECTORS; ++k)
        if (mHist[k] == 1) {all_free = false; break;}

    if (all_free) {
        if (candidates.empty())
            candidates.push_back(k_target);
        } else {
            int scan_start = 0;
            bool found = false;
            for (int k = 0; k < NUM_SECTORS; ++k){
                int prev = (k - 1 + NUM_SECTORS) % NUM_SECTORS;
                if (mHist[prev] == 1 && mHist[k] == 0) {
                    scan_start = k;
                    found = true;
                    break;
            }
        }

        if (!found) {
            for (int k = 0; k < NUM_SECTORS; ++k) {
               if (mHist[k] == 0) { scan_start = k; break; }
            }
        }

        int k = scan_start;
        int iter = 0;
        while (iter < NUM_SECTORS) {
            if (mHist[k] == 0) {
                int v_start = k;
                int width = 0;

                while (width < NUM_SECTORS && mHist[(k + width) % NUM_SECTORS] == 0 && iter + width < NUM_SECTORS) {
                    width++;
                }

                if (width > 0) {
                    if (width <= S_MAX) {
                        int mid = (v_start + width / 2) % NUM_SECTORS;
                        candidates.push_back(mid);
                    } else {
                        int c_right = (v_start + S_MAX / 2) % NUM_SECTORS;
                        int c_left = (v_start + width - 1 - S_MAX / 2 + NUM_SECTORS) % NUM_SECTORS;
                        candidates.push_back(c_right);
                        candidates.push_back(c_left);
                        int dist_from_start = (k_target - v_start + NUM_SECTORS) % NUM_SECTORS;
                        if (dist_from_start < width){
                            candidates.push_back(k_target);
                        }
                    }
                }
                k = (k + width) % NUM_SECTORS;
                iter += width;
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
                      + _mu2 * circularDist(c, 0)
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

