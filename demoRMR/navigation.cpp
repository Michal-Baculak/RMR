#include "navigation.h"
#include "utility.h"

Navigation::Navigation()
    : _prev_binary_hist(NUM_SECTORS, 0)
    , _prev_dir(0)
    , _last_mHist(NUM_SECTORS, 0)
{}

int Navigation::circularDist(int k1, int k2) const
{
    // circular distance of sectors (minimum distance of sectors 2 & 34 is 3, not 32)
    int diff = std::abs(k1 - k2);
    return std::min(diff, NUM_SECTORS-diff);
}

std::optional<double> Navigation::update(const std::vector<LaserData>  &laserData, double rPhi, double currentV, double currentW, double targetAngle)
{
    std::vector<double> pHist = calcPHist(laserData, rPhi);

    // -----------------------------------Binarny Histogram-----------------------------------

    std::vector<int> bHist(NUM_SECTORS, 0);

    for (int k = 0; k < NUM_SECTORS; k++)
    {
        if (pHist[k] < _t_low) bHist[k] = 0;
        else if (pHist[k] > _t_high) bHist[k] = 1;
        else bHist[k] = _prev_binary_hist[k];
    }

    _prev_binary_hist = bHist;

    // std::cout << "bHist = (";
    // for (int k = 0; k < NUM_SECTORS; k++)
    // {
    //     std::cout << bHist[k] << ", ";
    // }
    // std::cout << ")" << bHist.size() << std::endl;

    // -----------------------------------Maskovany Histogram-----------------------------------

    double r_min; // minimalny polomer otacania

    if (std::fabs(currentW) < 1e-6)
        r_min = 1e6; // stoji/ide rovno
    else
        r_min = std::fabs(currentV/currentW); // po kruznici s polomerom otacania

    std::vector<int> mHist = bHist;

    for (const auto &ld : laserData)
    {
        double d_i = ld.scanDistance / 1000.0;
        if (d_i <= RADIUS || d_i > WIN_SIZE) continue;

        double angle_deg_local = -ld.scanAngle;
        // while(angle_deg_local < 0.0) angle_deg_local += 360.0;
        // while(angle_deg_local >= 360.0) angle_deg_local -= 360.0;

        double angle_rad = angle_deg_local * PI / 180.0;
        double ox = d_i * std::cos(angle_rad);
        double oy = d_i * std::sin(angle_rad);

        double cr_y = -r_min;
        double cl_y = r_min;

        double dist_R = std::sqrt(ox*ox + (oy - cr_y)*(oy - cr_y));
        double dist_L = std::sqrt(ox*ox + (oy - cl_y)*(oy - cl_y));

        int k_obs = angleToSector(angle_deg_local);

        double ratio = (RADIUS + DS) / d_i;
        if (ratio > 1.0) ratio = 1.0;
        double gamma_deg = std::asin(ratio) * 180.0 / PI;
        int gamma_sectors = static_cast<int>(std::ceil(gamma_deg / SIGMA));

        if (oy > 0.0 && dist_L < (r_min + RADIUS + DS))
        {
            for (int dk = -gamma_sectors; dk <= gamma_sectors; ++dk)
            {
                int kk = (k_obs + dk + NUM_SECTORS) % NUM_SECTORS;
                mHist[kk] = 1;
            }
        }

        if (oy <= 0.0 && dist_R < (r_min + RADIUS + DS))
        {
            for (int dk = -gamma_sectors; dk <= gamma_sectors; ++dk)
            {
                int kk = (k_obs + dk + NUM_SECTORS) % NUM_SECTORS;
                mHist[kk] = 1;
            }
        }

        // if (dist_L < (r_min + RADIUS + DS))
        // {
        //     for (int dk = -gamma_sectors; dk <= 0; ++dk)
        //     {
        //         int kk = (k_obs + dk + NUM_SECTORS) % NUM_SECTORS;
        //         mHist[kk] = 1;
        //     }
        // }

        // if (dist_R < (r_min + RADIUS + DS))
        // {
        //     for (int dk = 0; dk <= gamma_sectors; ++dk)
        //     {
        //         int kk = (k_obs + dk + NUM_SECTORS) % NUM_SECTORS;
        //         mHist[kk] = 1;
        //     }
        // }
    }

    // std::cout << "mHist = (";
    // for (int k = 0; k < NUM_SECTORS; k++)
    // {
    //     std::cout << mHist[k] << (k < NUM_SECTORS - 1 ? ", " : " ");
    // }
    // std::cout << ")" << mHist.size() << std::endl;
    _last_mHist = mHist;

    // -------------------------------Kandidatske smery------------------

    double targetAngle_deg = targetAngle * 180.0 / PI;
    double robotAngle_deg  = rPhi * 180.0 / PI;

    double local_target_deg = targetAngle_deg - robotAngle_deg;
    if (local_target_deg < 0.0) local_target_deg += 360.0;
    if (local_target_deg >= 360.0) local_target_deg -= 360.0;

    int k_target = angleToSector(local_target_deg);
    int k_robot_heading = angleToSector(0.0);

    std::vector<int> candidates = findCandidateSectors(mHist, k_target);

    // int best_k;

    if (candidates.empty())
    {
        std::cout << "[VFH+] Ziadne volne udolie! Zastavujem robota." << std::endl;
        return std::nullopt;
    }


    int best_k = selectBestSector(candidates, k_target, k_robot_heading);

    _prev_dir = best_k;

    double safe_heading_rad = sectorToSafeHeading(best_k, robotAngle_deg);

    // double local_best_deg = sectorToAngle(best_k);
    // double global_best_deg = local_best_deg + robotAngle_deg;

    // global_best_deg = std::fmod(global_best_deg, 360.0);
    // if (global_best_deg > 180.0) global_best_deg -= 360.0;
    // if (global_best_deg < -180.0) global_best_deg += 360.0;

    // double safe_heading_rad = global_best_deg * PI / 180.0;

    std::cout << "[VFH+] k_target=" << k_target
           << " best_k=" << best_k
           << " safe_heading=" << safe_heading_rad * 180.0 / PI << " deg" << std::endl;

    return safe_heading_rad;
}

std::vector<double> Navigation::calcPHist(const std::vector<LaserData>  &laserData, double rPhi)
{
    std::vector<double> hist(NUM_SECTORS, 0.0);

    for (int i = 0; i < (int)laserData.size(); i++)
    {
        double d_i = laserData[i].scanDistance / 1000.0;

        if (d_i <= RADIUS || d_i > WIN_SIZE) continue;

        double angle_deg_laser = laserData[i].scanAngle;
        double angle_i = -angle_deg_laser;
        while (angle_i < 0.0) angle_i += 360.0;
        while (angle_i >= 360.0) angle_i -= 360.0;

        double ratio = (DS) / d_i;
        if (ratio > 1.0) ratio = 1.0;

        double gamma_i = std::asin(ratio) * 180.0 / PI;// zvacsovaci uhol

        double m_i = (_a - _b * d_i); // magnituda prekazky
        if (m_i < 0.0) m_i = 0.0;

        double alpha_low = angle_i - gamma_i;
        double alpha_high = angle_i + gamma_i;

        int k_min = (int)std::floor(alpha_low / SIGMA);
        int k_max = (int)std::floor(alpha_high / SIGMA);

        for (int k = k_min; k <= k_max; k++)
        {
            int kk = ((k % NUM_SECTORS) + NUM_SECTORS) % NUM_SECTORS;

            double sector_low = k * SIGMA;
            double sector_high = (k + 1) * SIGMA;

            if (std::max(sector_low, alpha_low) <= std::min(sector_high, alpha_high))
            {
                hist[kk] += m_i;
            }
        }

    }

    return hist;
}

std::vector<int> Navigation::findCandidateSectors(const std::vector<int> &mHist, int k_target) const
{
    std::vector<int> candidates;

    bool allFree = true;
    for (int k = 0; k < NUM_SECTORS; k++)
    {
        if (mHist[k] != 0)
        {
            allFree = false;
            break;
        }
    }

    if (allFree)
    {
        candidates.push_back(k_target);
        return candidates;
    }

    int start_search = 0;
    for (int i = 0; i < NUM_SECTORS; i++)
    {
        if (mHist[i] == 0 && mHist[(i - 1 + NUM_SECTORS) % NUM_SECTORS] == 1) // ak aktualny 0 a predosly 1, zacne hladat kandidatske sektory
        {
            start_search = i;
            break;
        }
    }

    int i = start_search;
    int checked = 0;
    while (checked < NUM_SECTORS)
    {
        if (mHist[i % NUM_SECTORS] == 0)
        {
            int valley_start = i % NUM_SECTORS; // start udolia
            int length = 0;

            while (checked + length < NUM_SECTORS && mHist[(i + length) % NUM_SECTORS] == 0)
            {
                length++; // pridavame dlzku udolia az kym nasledujuci sektor nie je rovny 1
            }

            int valley_end = (i + length - 1) % NUM_SECTORS; // koniec udolia

            if (length <= S_MAX)
            {
                int valley_center = (i + length / 2) % NUM_SECTORS; // stred udolia
                candidates.push_back(valley_center); // uzke udolie, kandidatsky smer je v strede
            }
            else //siroke udolie
            {
                int k_right = (valley_start + static_cast<int>(S_MAX/3)) % NUM_SECTORS; // kandidat je start + 1 sektor
                int k_left  = (valley_end - static_cast<int>(S_MAX/3) + NUM_SECTORS) % NUM_SECTORS; // kandidat je end + 1 sektor
                candidates.push_back(k_right);
                candidates.push_back(k_left);

                for (int d = 0; d < length; d++) // hladame cielovy sektor pozdlz dlzky udolia
                {
                    if ((i + d) % NUM_SECTORS == k_target)
                    {
                        candidates.push_back(k_target); // cielovy sektor sa nachadza v prechode
                        break;
                    }
                }
            }

            checked += length;
            i += length;
        }
        else
        {
            checked++;
            i++;
        }
    }
    return candidates;
}

int Navigation::angleToSector(double angle_deg) const
{
    while (angle_deg < 0.0) angle_deg += 360.0;
    while (angle_deg >= 360.0) angle_deg -= 360.0;
    int k = static_cast<int>(angle_deg / SIGMA);
    if (k < 0) k = 0;
    if (k >= NUM_SECTORS) k = NUM_SECTORS - 1;
    return k;
}

double Navigation::sectorToAngle(int k) const
{
    return (k + 0.5) * SIGMA;
}

double Navigation::sectorToSafeHeading(int sector, double robot_angle_deg) const
{
    double local_best_deg = sectorToAngle(sector);
    double global_best_deg = local_best_deg + robot_angle_deg;

    global_best_deg = std::fmod(global_best_deg, 360.0);
    if (global_best_deg > 180.0)
        global_best_deg -= 360.0;
    if (global_best_deg < -180.0)
        global_best_deg += 360.0;

    double safe_heading_rad = global_best_deg * PI / 180.0;
    return safe_heading_rad;
}

int Navigation::selectBestSector(const std::vector<int> &candidates, int k_target, int k_robot_heading) const
{
    int best_k = candidates[0];
    double best_cost = std::numeric_limits<double>::max();

    for (int k_c : candidates)
    {
        double cost = _mu1 * circularDist(k_c, k_target)
                    + _mu2 * circularDist(k_c, k_robot_heading)
                    + _mu3 * circularDist(k_c, _prev_dir);

        if (cost < best_cost)
        {
            best_cost = cost;
            best_k = k_c;
        }
    }

    return best_k;
}

bool Navigation::isDirWithinCurrentSector(double dir, double robot_rot) const
{
    // safe heading is the middle of the current sector
    double safe_heading = sectorToSafeHeading(_prev_dir, robot_rot * PI / 180.0);
    double angle_diff = utility::wrap(dir - safe_heading);
    return abs(angle_diff) < SIGMA / 2;
}
